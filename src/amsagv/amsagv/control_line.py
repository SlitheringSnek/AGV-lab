#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor, qos_event, qos_actions
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped, ActionsStamped
from math import pi, sin, cos, isnan, radians, degrees
from amsagv.world import MTAG


# Line-sensor geometry: 7 sensors, index 0 = right side, index 6 = left side, 3 = middle.
# The tape is bright (~160-190), the floor dark (~25-55).
MID = 3.0
MIN_CONTRAST = 80   # same thresholds as find_line_edges()
ALL_BRIGHT = 150

NO_LINE, CROSSING, LINE = 'no line', 'crossing', 'line'



def analyse_line(values):
    '''Analyse the raw line-sensor values.

    Returns (state, centre, left_edge, right_edge), positions in sensor-index units (0..6):
      state = LINE     - line found; centre = centroid of the bright sensors,
                         left_edge = outer edge of the left-most bright patch (6.5 = beyond the sensor),
                         right_edge = outer edge of the right-most bright patch (-0.5 = beyond the sensor)
      state = CROSSING - all sensors bright (crossroads / wide patch)
      state = NO_LINE  - all sensors dark
    '''
    values = [float(v) for v in values]
    vmin, vmax = min(values), max(values)
    if vmax - vmin <= MIN_CONTRAST:
        return (CROSSING if vmax > ALL_BRIGHT else NO_LINE), None, None, None

    thr = (vmin + vmax) / 2.0
    bright = [i for i, v in enumerate(values) if v > thr]

    weights = [max(0.0, v - thr) for v in values]
    centre = sum(i * w for i, w in enumerate(weights)) / sum(weights)

    i = bright[-1] # Left edge: interpolate between the last bright and the next dark sensor
    left_edge = 6.5 if i == 6 else i + (values[i] - thr) / (values[i] - values[i + 1])
    j = bright[0]  # Right edge
    right_edge = -0.5 if j == 0 else j - (values[j] - thr) / (values[j] - values[j - 1])

    return LINE, centre, left_edge, right_edge



def main():
    tag = None
    e_prev = 0.0     # Previous line-position error
    was_lost = True
    fd = 0.0         # Travelled distance (from the line message)
    cross_start = None # fd where the crossing (all bright) started
    mode = None      # 'center', 'left' or 'right' (set from the parameter below)
    actions = []     # Pending path actions: [(name, tag_id, distance)]
    switch = None    # (name, fd at which to switch) after the action's tag was seen



    def set_mode(name):
        nonlocal mode, v_stop
        name = name.lower()
        if name in ('straight', 'center', 'centre'):
            mode = 'center'
        elif name in ('left', 'right'):
            mode = name
        elif name == 'stop':
            v_stop = True
        else:
            print(f'Unknown action "{name}", keeping mode {mode}')
            return
        print(f'>>> mode = {mode}{" (STOP)" if v_stop else ""}')



    # Handle line sensor
    def handle_line(msg):
        nonlocal e_prev, was_lost, fd, cross_start, switch

        fd = msg.line.distance
        gamma = msg.line.heading # Front-cart angle w.r.t. the body (+ = left)

        # Pending action: switch the mode once the distance after the tag is travelled
        if switch is not None and fd >= switch[1]:
            set_mode(switch[0])
            switch = None

        state, centre, left_edge, right_edge = analyse_line(msg.line.values)

        status = state
        e = None
        if state == LINE:
            cross_start = None
            if mode == 'left':
                pos = left_edge - param_half_width   # keep the left edge half a line width left of the middle
            elif mode == 'right':
                pos = right_edge + param_half_width
            else:
                pos = centre
            e = param_sign * (pos - MID) / MID      # [-1, 1], + = line on the left
        elif state == CROSSING:
            # Whole sensor on tape: drive straight across, for at most cross_dist
            if cross_start is None:
                cross_start = fd
            if fd - cross_start < param_cross_dist:
                e = 0.0
                status = f'crossing {100*(fd - cross_start):.0f} cm'
            else:
                status = 'crossing too long -> stop'
        else:
            cross_start = None

        if e is None or v_stop:
            # Off the line (or stop action): stop immediately
            v, w = 0.0, 0.0
            was_lost = True
            if v_stop:
                status = 'STOP action'
        else:
            if was_lost:
                e_prev = e # no derivative kick when the line is (re)acquired
                was_lost = False

            # PD controller of the angular velocity: turn the cart towards the line
            de = (e - e_prev) * 50.0 # Sensor messages come at 50 Hz
            w = param_kp * e + param_kd * de
            w = max(-param_w_max, min(param_w_max, w))
            # Slow down when the line is far from the centre (sharp turn), but keep moving
            # forward, otherwise the cart only spins in place
            v = param_v * max(0.5, 1.0 - abs(e))

            # Cart-angle limit: never turn the cart further than gamma_max, otherwise it
            # folds and the robot drives backwards
            if abs(gamma) > param_gamma_stop:
                v, w = 0.0, 0.0
                status = f'CART FOLDED (|gamma|>{degrees(param_gamma_stop):.0f}) -> stop, straighten it'
            elif abs(gamma) > param_gamma_max and w * gamma > 0.0:
                w = 0.0
                status += ', gamma limit'
            e_prev = e

        if param_debug:
            vals = ' '.join(f'{int(x):3d}' for x in msg.line.values)
            es = f'{e:+.2f}' if e is not None else '  ---'
            print(f'[{vals}] {mode:6s} e={es} gamma={degrees(gamma):+6.1f} '
                  f'fd={fd:+.3f} | v={v:.3f} w={w:+.2f} {status}')

        # Velocity commands message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = v
        msg_cmd_vel.angular.z = w
        # Publish velocity commands
        pub_cmd_vel.publish(msg_cmd_vel)



    # Handle tag message
    def handle_tag(msg):
        nonlocal tag, switch
        tag = MTAG.get(msg.tag.id, None)
        print(f'New tag: {msg.tag.id} -> {tag}')
        # Next path action belongs to this tag: switch mode after its distance
        if actions and actions[0][1] == msg.tag.id:
            name, _, dist = actions.pop(0)
            switch = (name, fd + dist)
            print(f'>>> action "{name}" in {dist:.3f} m')



    # Handle path actions (from path_planning): list of (name, tag id, distance after the tag)
    def handle_actions(msg):
        nonlocal actions, v_stop
        actions = [(a.action.name, a.action.id, a.action.distance) for a in msg.actions]
        v_stop = False
        print(f'New path actions: {actions}')



    rclpy.init()
    node = Node('control_line')

    try:
        # Controller parameters (run with: ros2 run amsagv control_line --ros-args -p kp:=1.5 ...)
        # Pass floats as floats (1.0, not 1).
        param_v = node.declare_parameter('v', 0.05).value             # Forward velocity, m/s (0 = steering-only test)
        param_kp = node.declare_parameter('kp', 1.5).value            # Proportional gain
        param_kd = node.declare_parameter('kd', 0.0).value            # Derivative gain
        param_w_max = node.declare_parameter('w_max', 1.5).value      # Max angular velocity, rad/s
        param_sign = node.declare_parameter('sign', 1.0).value        # Set to -1.0 if it steers away from the line
        param_gamma_max = radians(node.declare_parameter('gamma_max', 60.0).value)   # Cart-angle limit, deg
        param_gamma_stop = radians(node.declare_parameter('gamma_stop', 100.0).value) # Stop if the cart is folded, deg
        # Which part of the line to follow: center, left (takes left branches) or right
        param_mode = node.declare_parameter('mode', 'center').value
        # Half of the tape width, in sensor spacings (measured: edges at 2.47 and 5.38 -> 1.45)
        param_half_width = node.declare_parameter('half_width', 1.45).value
        # Max distance driven straight while the whole sensor is bright (crossroads), m
        param_cross_dist = node.declare_parameter('cross_dist', 0.06).value
        param_debug = node.declare_parameter('debug', True).value

        v_stop = False
        set_mode(param_mode)

        # Velocity commands publisher.
        pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos_cmd)
        # Line sensor subscriber
        sub_line = node.create_subscription(LineStamped, 'line', handle_line, qos_sensor)
        # Tag subscriber
        sub_tag = node.create_subscription(TagStamped, 'tag', handle_tag, qos_event)
        # Path actions subscriber
        sub_actions = node.create_subscription(ActionsStamped, 'path_actions', handle_actions, qos_actions)

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
