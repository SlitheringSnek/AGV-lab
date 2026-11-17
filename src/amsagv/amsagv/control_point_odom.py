#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''Control to a point / pose / sequence of points, using odometry.

Usage (always restart `agv` first, so the odom origin is the current robot pose):
  point:   ros2 run amsagv control_point_odom --ros-args -p x_ref:=0.5 -p y_ref:=0.0
  pose:    ros2 run amsagv control_point_odom --ros-args -p x_ref:=0.5 -p y_ref:=0.3 -p phi_ref:=90.0
  points:  ros2 run amsagv control_point_odom --ros-args -p points:="[0.42, 0.15, 0.72, -0.15]"
Coordinates are for the swivel joint of the front cart, which starts at x = D = 0.121 m.
Write all numbers as floats (0.0, not 0).
'''
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv.ams import msg_to_pose, wrap_to_pi
from math import sin, cos, atan2, sqrt, degrees, radians, copysign



def main():
    done = False
    targets = [] # Sequence of target points [(x, y, name)]; the robot stops at the last one
    i = 0        # Index of the current target



    # Handle odometry
    def handle_odometry(msg):
        nonlocal done, i
        x, y, phi = msg_to_pose(msg.pose.pose)
        gamma = msg.pose.pose.position.z

        # The driven wheels are on the front cart, so we control the cart:
        # position of the swivel joint (D in front of the rear axle) and cart heading.
        xc = x + param_d * cos(phi)
        yc = y + param_d * sin(phi)
        theta = wrap_to_pi(phi + gamma)

        # Distance and bearing error to the current target point
        x_t, y_t, name = targets[i]
        dx, dy = x_t - xc, y_t - yc
        dist = sqrt(dx**2 + dy**2)
        alpha = wrap_to_pi(atan2(dy, dx) - theta)
        last = (i == len(targets) - 1)

        # "Passed": target behind the cart while still close. Counts as reached, otherwise a
        # few-mm overshoot makes the robot drive a full circle back (the cart can't turn the
        # body on the spot).
        passed = dist < param_capture and cos(alpha) < 0.0

        # Intermediate target reached -> switch to the next one without stopping
        if not last and not done and (dist < param_ip_tol or passed):
            print(f'>>> {name} reached (x={xc:.3f} y={yc:.3f}, error {dist*100:.1f} cm), next: {targets[i+1][2]}')
            i += 1
            return

        if done or (last and (dist < param_tol or passed)):
            # Goal reached -> stop and stay stopped
            vs, ws = 0.0, 0.0
            if not done:
                print(f'Goal reached: x={xc:.3f} y={yc:.3f} (error {dist*100:.1f} cm)')
                if pose_mode:
                    print(f'  heading: body phi={degrees(phi):+.1f}°, cart th={degrees(theta):+.1f}°, '
                          f'reference {degrees(phi_ref):+.1f}° -> body error '
                          f'{degrees(wrap_to_pi(phi - phi_ref)):+.1f}°')
            done = True
        else:
            # Turn towards the target; drive forward only when roughly facing it
            ws = param_kw * alpha
            vs = min(param_kv * dist * max(0.0, cos(alpha)), param_v_max)
            # Don't slow down towards intermediate targets (only towards the last one)
            if not last:
                vs = param_v_max * max(0.0, cos(alpha))
            # Cart-angle limit: never let the cart fold round (|gamma| -> 180 deg jams the robot).
            # At the limit, instead of turning the cart further, creep forward and hold gamma, so
            # the body follows the cart around the tightest arc. Hold term:
            #   d(gamma)/dt = ws - vs*sin(gamma)/D  ->  ws = vs*sin(gamma)/D - kg*(gamma excess)
            if abs(gamma) > param_gamma_max and ws * gamma > 0.0:
                vs = max(vs, param_v_turn)
                excess = gamma - copysign(param_gamma_max, gamma)
                ws = vs * sin(gamma) / param_d - param_kg * excess
            # Saturation (vs is already limited above, the gamma-hold term needs the real vs)
            vs = min(vs, param_v_max)
            ws = max(-param_w_max, min(param_w_max, ws))

        if param_debug:
            print(f'{name:6s} cart: x={xc:+.3f} y={yc:+.3f} phi={degrees(phi):+6.1f}° th={degrees(theta):+6.1f}° '
                  f'gamma={degrees(gamma):+6.1f}° | '
                  f'dist={dist:.3f} alpha={degrees(alpha):+6.1f}° | vs={vs:.3f} ws={ws:+.2f}')

        # Velocity commands message
        msg_cmd_vel.linear.x = vs
        msg_cmd_vel.angular.z = ws
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)



    rclpy.init()
    node = Node('control_point_odom')

    try:
        # Reference point in the odom frame (odom origin = AGV pose when agv was started)
        param_x_ref = node.declare_parameter('x_ref', 0.5).value
        param_y_ref = node.declare_parameter('y_ref', 0.0).value
        # Optional reference heading in degrees -> control to a POSE. Default 999 = point only.
        param_phi_ref = node.declare_parameter('phi_ref', 999.0).value
        # Optional sequence of points [x1, y1, x2, y2, ...] (overrides x_ref/y_ref)
        param_points = node.declare_parameter('points', Parameter.Type.DOUBLE_ARRAY).value
        # Intermediate point of the pose control: distance r behind the goal. r must be a few
        # times D: the body only lines up with the cart after driving straight for a while
        # (simulation: r=0.15 -> body error up to 52 deg, r=0.35 -> 11 deg).
        param_r = node.declare_parameter('r', 0.30).value
        # Distance at which an intermediate target counts as reached
        param_ip_tol = node.declare_parameter('ip_tol', 0.04).value
        # Controller parameters
        param_kv = node.declare_parameter('kv', 1.0).value          # 1/s
        param_kw = node.declare_parameter('kw', 2.0).value          # 1/s
        param_v_max = node.declare_parameter('v_max', 0.1).value    # m/s
        param_w_max = node.declare_parameter('w_max', 2.0).value    # rad/s
        param_tol = node.declare_parameter('tol', 0.02).value       # Goal tolerance, m
        # If the goal is behind the cart and closer than this, count it as reached (no circling back)
        param_capture = node.declare_parameter('capture', 0.05).value
        # Cart-angle limit (deg), forward creep speed at the limit (m/s) and hold gain (1/s)
        param_gamma_max = radians(node.declare_parameter('gamma_max', 70.0).value)
        param_v_turn = node.declare_parameter('v_turn', 0.05).value
        param_kg = node.declare_parameter('kg', 3.0).value
        # Distance from the rear axle to the swivel joint (same as in agv.py)
        param_d = node.declare_parameter('joint_dist', 0.1207).value
        param_debug = node.declare_parameter('debug', True).value

        # Build the list of targets
        pose_mode = abs(param_phi_ref) <= 360.0
        phi_ref = radians(param_phi_ref) if pose_mode else 0.0
        if param_points:
            pts = list(param_points)
            if len(pts) % 2:
                raise ValueError('points needs pairs: [x1, y1, x2, y2, ...]')
            targets = [(pts[k], pts[k+1], f'P{k//2+1}') for k in range(0, len(pts), 2)]
        else:
            targets = [(param_x_ref, param_y_ref, 'goal')]
        if pose_mode:
            # Intermediate point r behind the last target, along the reference heading
            x_g, y_g, _ = targets[-1]
            targets.insert(len(targets) - 1, (x_g - param_r * cos(phi_ref), y_g - param_r * sin(phi_ref), 'ip'))
        print('Targets: ' + ', '.join(f'{n}=({tx:.3f}, {ty:.3f})' for tx, ty, n in targets)
              + (f', final heading {param_phi_ref:.0f}°' if pose_mode else ''))

        msg_cmd_vel = Twist()

        # Velocity commands publisher.
        pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos_cmd)
        # Odometry subscriber
        sub_odom = node.create_subscription(Odometry, 'odom', handle_odometry, qos_sensor)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Stop the robot
        try:
            pub_cmd_vel.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
