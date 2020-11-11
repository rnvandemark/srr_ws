#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

HELP_MSG = """
---------------------------
**Displaying movement controls is a TODO**
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
---------------------------
"""

NODE_SLEEP_MS = 10

INC_SPEED   = 1.0
INC_ROT     = 1.0
MAG_WHEELS  = 12.5
MAG_CHASSIS = 1.0

BINDINGS_MOVE = {
# L=Left, R=Right, F=Front, B=Back/Rear, W=Wheels, C=Chassis
#         LW  RW  FLC FRC BLC BRC
    'w': (-1, -1,  0,  0,  0,  0),
    's': (+1, +1,  0,  0,  0,  0),
    'a': (+1, -1,  0,  0,  0,  0),
    'd': (-1, +1,  0,  0,  0,  0),
    'o': ( 0,  0, +1, +1, +1, +1),
    'p': ( 0,  0, -1, -1, -1, -1),
    'i': ( 0,  0, -1, -1, +1, +1),
    'k': ( 0,  0, +1, +1, -1, -1),
    'j': ( 0,  0, -1, +1, -1, +1),
    'l': ( 0,  0, +1, -1, +1, -1)
}

BINDINGS_SPEED = {
    't': (0.9, 0.0),
    'y': (1.1, 0.0),
    'g': (0.0, 0.9),
    'h': (0.0, 1.1)
}

def create_pub(name):
    return rospy.Publisher(
        "/srr_integrated/{0}_joint_controller/command".format(name),
        Float64,
        queue_size=10
    )

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("srr_vehicle_teleop")

    pub_front_left_wheel    = create_pub("front_left_wheel")
    pub_front_right_wheel   = create_pub("front_right_wheel")
    pub_rear_left_wheel     = create_pub("rear_left_wheel")
    pub_rear_right_wheel    = create_pub("rear_right_wheel")
    pub_front_left_chassis  = create_pub("front_left")
    pub_front_right_chassis = create_pub("front_right")
    pub_rear_left_chassis   = create_pub("rear_left")
    pub_rear_right_chassis  = create_pub("rear_right")

    speed_left_wheels         = 0
    speed_right_wheels        = 0
    speed_front_left_chassis  = 0
    speed_front_right_chassis = 0
    speed_rear_left_chassis   = 0
    speed_rear_right_chassis  = 0

    try:
        print HELP_MSG
        #print vels(speed_left, speed_right)
        while(1):
            key = get_key()
            if (key == '\x03'):
                break
            elif key in BINDINGS_MOVE.keys():
                speed_left_wheels         = BINDINGS_MOVE[key][0] * MAG_WHEELS
                speed_right_wheels        = BINDINGS_MOVE[key][1] * MAG_WHEELS
                speed_front_left_chassis  = BINDINGS_MOVE[key][2] * MAG_CHASSIS
                speed_front_right_chassis = BINDINGS_MOVE[key][3] * MAG_CHASSIS
                speed_rear_left_chassis   = BINDINGS_MOVE[key][4] * MAG_CHASSIS
                speed_rear_right_chassis  = BINDINGS_MOVE[key][5] * MAG_CHASSIS
            else:
                speed_left_wheels         = 0
                speed_right_wheels        = 0
                speed_front_left_chassis  = 0
                speed_front_right_chassis = 0
                speed_rear_left_chassis   = 0
                speed_rear_right_chassis  = 0

            pub_front_left_wheel.publish(speed_left_wheels)
            pub_front_right_wheel.publish(speed_right_wheels)
            pub_rear_left_wheel.publish(speed_left_wheels)
            pub_rear_right_wheel.publish(speed_right_wheels)
            pub_front_left_chassis.publish(speed_front_left_chassis)
            pub_front_right_chassis.publish(speed_front_right_chassis)
            pub_rear_left_chassis.publish(speed_rear_left_chassis)
            pub_rear_right_chassis.publish(speed_rear_right_chassis)

    except KeyboardInterrupt as e:
        print "Exiting..."

    finally:
        pub_front_left_wheel.publish(speed_left_wheels)
        pub_front_right_wheel.publish(speed_right_wheels)
        pub_rear_left_wheel.publish(speed_left_wheels)
        pub_rear_right_wheel.publish(speed_right_wheels)
        pub_front_left_chassis.publish(speed_front_left_chassis)
        pub_front_right_chassis.publish(speed_front_right_chassis)
        pub_rear_left_chassis.publish(speed_rear_left_chassis)
        pub_rear_right_chassis.publish(speed_rear_right_chassis)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
