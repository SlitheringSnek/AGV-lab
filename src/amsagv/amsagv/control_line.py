#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor, qos_event
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from amsagv.world import MTAG



def main():
    tag = None
    e_prev = 0.0 # Previous line-position error
    lost = 0     # Number of consecutive samples without the line



    # Handle line sensor
    def handle_line(msg):
        nonlocal e_prev, lost

        left, right = msg.line.left, msg.line.right

        if not isnan(left) and not isnan(right):
            # Line position under the sensor, in [-1, 1] (0 = line in the middle).
            # Positive = line is on the left side of the sensor.
            e = param_sign * (left + right) / 2.0
            lost = 0
        else:
            # Line lost: keep the last error for a short time (e.g. small gaps), then stop
            e = e_prev
            lost += 1

        if lost > param_lost_max:
            v, w = 0.0, 0.0
        else:
            # PD controller of the angular velocity: turn the cart towards the line
            de = (e - e_prev) * 50.0 # Sensor messages come at 50 Hz
            w = param_kp * e + param_kd * de
            # Slow down when the line is far from the centre (sharp turn)
            v = param_v * max(0.3, 1.0 - abs(e))
            if lost > 0:
                v *= 0.5

        e_prev = e

        if param_debug:
            print(f'left={left:+.2f} right={right:+.2f} e={e:+.2f} lost={lost:3d} | v={v:.3f} w={w:+.2f}')

        # Velocity commands message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = v
        msg_cmd_vel.angular.z = w
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)



    # Handle tag message
    def handle_tag(msg):
        nonlocal tag
        tag = MTAG.get(msg.tag.id, None)
        print(f'New tag: {msg.tag.id} -> {tag}')



    rclpy.init()
    node = Node('control_line')

    try:
        # Controller parameters
        param_v = node.declare_parameter('v', 0.1).value              # Forward velocity, m/s
        param_kp = node.declare_parameter('kp', 2.0).value            # Proportional gain
        param_kd = node.declare_parameter('kd', 0.0).value            # Derivative gain
        param_sign = node.declare_parameter('sign', 1.0).value        # Set to -1.0 if it steers away from the line
        param_lost_max = node.declare_parameter('lost_max', 15).value # Samples without line before stop (15 = 0.3 s)
        param_debug = node.declare_parameter('debug', True).value

        # Velocity commands publisher.
        pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos_cmd)
        # Line sensor subscriber
        sub_line = node.create_subscription(LineStamped, 'line', handle_line, qos_sensor)
        # Tag subscriber
        sub_tag = node.create_subscription(TagStamped, 'tag', handle_tag, qos_event)

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
