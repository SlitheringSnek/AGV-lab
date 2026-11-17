#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv.ams import msg_to_pose, wrap_to_pi
from math import sin, cos, atan2, sqrt, degrees, radians, copysign



def main():
    done = False



    # Handle odometry
    def handle_odometry(msg):
        nonlocal done
        x, y, phi = msg_to_pose(msg.pose.pose)
        gamma = msg.pose.pose.position.z

        # The driven wheels are on the front cart, so we control the cart:
        # position of the swivel joint (D in front of the rear axle) and cart heading.
        xc = x + param_d * cos(phi)
        yc = y + param_d * sin(phi)
        theta = wrap_to_pi(phi + gamma)

        # Distance and bearing error to the reference point
        dx, dy = param_x_ref - xc, param_y_ref - yc
        dist = sqrt(dx**2 + dy**2)
        alpha = wrap_to_pi(atan2(dy, dx) - theta)

        # Goal reached: inside the tolerance, OR we just passed it (goal behind the cart) while
        # still close. Otherwise a few-mm overshoot makes the robot drive a full circle back
        # (the cart can't turn the body on the spot).
        passed = dist < param_capture and cos(alpha) < 0.0
        if dist < param_tol or passed or done:
            # Goal reached -> stop and stay stopped
            vs, ws = 0.0, 0.0
            if not done:
                print(f'Goal reached: x={xc:.3f} y={yc:.3f} (error {dist*100:.1f} cm)')
            done = True
        else:
            # Turn towards the goal; drive forward only when roughly facing it
            ws = param_kw * alpha
            vs = min(param_kv * dist * max(0.0, cos(alpha)), param_v_max)
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
            print(f'cart: x={xc:+.3f} y={yc:+.3f} th={degrees(theta):+6.1f}° '
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
