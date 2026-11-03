#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
from amsagv.ams import pose_to_msg, get_frame_prefix
from amsagv.agvapi import Agv, find_line_edges
from amsagv.ams import wrap_to_pi
from math import cos, sin, pi, degrees


# Heading (front cart) encoder: 8192 ticks per revolution (measured ~2045 ticks per 90 deg)
HEADING_TICKS_PER_RAD = 8192.0 / (2.0 * pi)



def main():
    with Agv() as robot:
        # Handle velocity commands
        def handle_cmd_vel(msg):
            nonlocal cmd_v, cmd_w
            cmd_v, cmd_w = msg.linear.x, msg.angular.z
            robot.set_vel(msg.linear.x, msg.angular.z)



        # Main loop, executed at 50 Hz
        def handle_timer():
            nonlocal x, y, phi, gamma, fd, enc_prev, enc_print_prev, k
            t = node.get_clock().now().to_msg()

            # Read sensors
            robot.read_sensors()

            #
            # Odometry
            #

            # Encoders
            enc_left, enc_right, enc_heading = robot.get_encoders()

            # Encoder increments since the previous step
            if enc_prev is None:
                enc_prev = (enc_left, enc_right, enc_heading)
            d_left = enc_left - enc_prev[0]
            d_right = enc_right - enc_prev[1]
            enc_prev = (enc_left, enc_right, enc_heading)

            # Wheel displacements in m (positive = forward).
            # The left encoder counts down when driving forward, hence the minus sign.
            ds_left = -d_left / param_ticks_per_m
            ds_right = d_right / param_ticks_per_m

            # The driven wheels are on the FRONT CART (differential drive), the rear wheels
            # of the body are passive. The cart is attached to the body with a swivel joint
            # at distance D in front of the rear axle; gamma is the cart angle w.r.t. the body.
            # Pose (x, y, phi) is the middle of the rear axle.

            # Front-cart angle relative to the AGV body (positive = turned left)
            gamma = wrap_to_pi((enc_heading - param_heading_offset) / HEADING_TICKS_PER_RAD)

            # Travelled distance of the front cart (middle between the driven wheels)
            ds_s = (ds_right + ds_left) / 2.0
            fd += ds_s

            # Body kinematics: the joint moves with v_s in the cart direction (phi + gamma).
            # The rear axle can only move along the body: v = v_s*cos(gamma), w = v_s*sin(gamma)/D
            ds = ds_s * cos(gamma)                      # Travelled distance of the rear axle
            dphi = ds_s * sin(gamma) / param_joint_dist # Change of the body orientation

            # Integration of the pose (2nd order Runge-Kutta: midpoint orientation)
            x += ds * cos(phi + dphi / 2.0)
            y += ds * sin(phi + dphi / 2.0)
            phi = wrap_to_pi(phi + dphi)

            # Debug print, every 10th step (5 Hz)
            k += 1
            if param_debug and k % 10 == 0:
                if enc_print_prev is None:
                    enc_print_prev = (enc_left, enc_right, enc_heading)
                # Change since the previous printed line (10 steps = 0.2 s)
                pL = enc_left - enc_print_prev[0]
                pR = enc_right - enc_print_prev[1]
                enc_print_prev = (enc_left, enc_right, enc_heading)
                print(f'enc: L={enc_left:8d} R={enc_right:8d} H={enc_heading:6d} '
                      f'(dL={pL:+6d} dR={pR:+6d}) | '
                      f'x={x:+.3f} y={y:+.3f} phi={degrees(phi):+7.1f}° '
                      f'gamma={degrees(gamma):+6.1f}° fd={fd:.3f}')

            # Odometry message
            msg_odom.header.stamp = t
            msg_odom.pose.pose = pose_to_msg(x, y, phi)
            msg_odom.pose.pose.position.z = gamma
            # Publish odometry message
            pub_odom.publish(msg_odom)

            #
            # Line sensor
            #

            # Line-sensor values
            line_values = robot.get_line_values()
            # Left and right line edge
            edge_left, edge_right = find_line_edges(line_values)

            # Line-sensor message
            msg_line.header.stamp = t
            msg_line.line.values = line_values
            msg_line.line.left = float(edge_left) if edge_left is not None else float('nan')
            msg_line.line.right = float(edge_right) if edge_right is not None else float('nan')
            msg_line.line.heading = gamma
            msg_line.line.distance = fd
            # Publish line-sensor message
            pub_line.publish(msg_line)



        rclpy.init()
        node = Node('agv')
        
        try:
            prefix = get_frame_prefix(node)
            # Name of the odometry frame
            param_odom_frame_id = node.declare_parameter('odom_frame_id', f'{prefix}odom').value
            # Name of the AGV frame
            param_agv_frame_id = node.declare_parameter('agv_frame_id', f'{prefix}agv').value

            # Odometry publisher
            pub_odom = node.create_publisher(Odometry, 'odom', qos_sensor)
            # Line sensor publisher
            pub_line = node.create_publisher(LineStamped, 'line', qos_sensor)
            # Velocity commands subscriber.
            sub_cmd_vel = node.create_subscription(Twist, 'cmd_vel', handle_cmd_vel, qos_cmd)

            # Line-sensor message
            msg_line = LineStamped()

            # Odometry message
            msg_odom = Odometry()
            msg_odom.header.frame_id = param_odom_frame_id
            msg_odom.child_frame_id = param_agv_frame_id

            # Odometry initial state
            x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
            fd = 0.0 # Travelled distance of the front cart

            # Odometry parameters (measured on the robot)
            # Wheel-encoder ticks per metre (measured on a clean straight run: 55602 ticks over 0.50 m)
            param_ticks_per_m = node.declare_parameter('ticks_per_m', 111200.0).value
            # NOTE: distance between the driven wheels L = 0.0487 m (360 deg cart rotation gave
            # dsR - dsL = 0.306 m; ruler: 49 mm). Not needed for odometry, because the cart
            # angle is measured directly by the heading encoder.
            # Heading-encoder value when the front cart is straight.
            # The encoder wraps every 8192 ticks (one turn); wrap_to_pi() handles that.
            param_heading_offset = node.declare_parameter('heading_offset', 4660).value
            # D = distance from the REAR (passive) axle to the swivel joint of the front cart, in m
            # (TODO: measure with a ruler)
            param_joint_dist = node.declare_parameter('joint_dist', 0.12).value

            # Debug state
            param_debug = node.declare_parameter('debug', True).value # Print odometry
            enc_prev = None # Previous encoder readings
            enc_print_prev = None # Encoder readings at the previous debug print
            k = 0 # Step counter
            cmd_v, cmd_w = 0.0, 0.0 # Last received velocity command

            timer = node.create_timer(1.0/50.0, handle_timer)

            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()



if __name__ == '__main__':
    main()
