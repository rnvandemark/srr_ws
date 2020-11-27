#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from threading import Lock
from pynput import keyboard
from signal import signal, SIGTERM
from time import sleep
from curses import initscr, cbreak, noecho, endwin

HELP_MSG = """
---------------------------
**Displaying movement controls is a TODO**
space key : force stop
Press ESCAPE to quit
---------------------------
"""

INC_SPEED   = 1.0
INC_ROT     = 1.0
MAG_WHEELS  = 125
MAG_CHASSIS = 300.0

PREVIOUS_SIGTERM_CALLBACK = None
THREAD_LOCK = None
SHUTDOWN_REQUESTED = False
ACTIVE_KEYS = None
NODE_SLEEP_S = 0.05

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

def exit_gracefully(signum, curr_stack_frame):
    global SHUTDOWN_REQUESTED
    THREAD_LOCK.acquire()
    try:
        SHUTDOWN_REQUESTED = True
    finally:
        THREAD_LOCK.release()
    if PREVIOUS_SIGTERM_CALLBACK != None and type(PREVIOUS_SIGTERM_CALLBACK) != int:
        PREVIOUS_SIGTERM_CALLBACK()

def create_pub(name):
    return rospy.Publisher(
        "/srr_integrated/{0}_joint_controller/command".format(name),
        Float64,
        queue_size=10
    )

def handle_key_press(key):
    if key not in ACTIVE_KEYS:
        ACTIVE_KEYS[key] = 1

def handle_key_release(key):
    global SHUTDOWN_REQUESTED
    if key in ACTIVE_KEYS:
        ACTIVE_KEYS[key] = -1
    if key == keyboard.Key.esc or SHUTDOWN_REQUESTED:
        THREAD_LOCK.acquire()
        try:
            SHUTDOWN_REQUESTED = True
        finally:
            THREAD_LOCK.release()
        return False

def update_commanded_joint_efforts():
    global SHUTDOWN_REQUESTED, THREAD_LOCK, ACTIVE_KEYS, BINDINGS_MOVE, NODE_SLEEP_S

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
        shutdown_requested = False
        while not shutdown_requested:
            THREAD_LOCK.acquire()
            try:
                shutdown_requested = SHUTDOWN_REQUESTED
                forced_stop = False
                for key, event_effect in ACTIVE_KEYS.items():
                    if key == keyboard.Key.space:
                        speed_left_wheels         = 0
                        speed_right_wheels        = 0
                        speed_front_left_chassis  = 0
                        speed_front_right_chassis = 0
                        speed_rear_left_chassis   = 0
                        speed_rear_right_chassis  = 0
                        forced_stop = True
                        break
                    elif type(key) is keyboard.KeyCode:
                        ch = key.char
                        if ch in BINDINGS_MOVE.keys():
                            speed_left_wheels         = speed_left_wheels         + (BINDINGS_MOVE[ch][0] * event_effect * MAG_WHEELS)
                            speed_right_wheels        = speed_right_wheels        + (BINDINGS_MOVE[ch][1] * event_effect * MAG_WHEELS)
                            speed_front_left_chassis  = speed_front_left_chassis  + (BINDINGS_MOVE[ch][2] * event_effect * MAG_CHASSIS)
                            speed_front_right_chassis = speed_front_right_chassis + (BINDINGS_MOVE[ch][3] * event_effect * MAG_CHASSIS)
                            speed_rear_left_chassis   = speed_rear_left_chassis   + (BINDINGS_MOVE[ch][4] * event_effect * MAG_CHASSIS)
                            speed_rear_right_chassis  = speed_rear_right_chassis  + (BINDINGS_MOVE[ch][5] * event_effect * MAG_CHASSIS)

                    if event_effect == 1:
                        ACTIVE_KEYS[key] = 0
                    elif event_effect == -1:
                        ACTIVE_KEYS.pop(key)

                if forced_stop:
                    ACTIVE_KEYS.clear()
            finally:
                THREAD_LOCK.release()

            pub_front_left_wheel.publish(speed_left_wheels)
            pub_front_right_wheel.publish(speed_right_wheels)
            pub_rear_left_wheel.publish(speed_left_wheels)
            pub_rear_right_wheel.publish(speed_right_wheels)
            pub_front_left_chassis.publish(speed_front_left_chassis)
            pub_front_right_chassis.publish(speed_front_right_chassis)
            pub_rear_left_chassis.publish(speed_rear_left_chassis)
            pub_rear_right_chassis.publish(speed_rear_right_chassis)

            sleep(NODE_SLEEP_S)

    finally:
        SHUTDOWN_REQUESTED = True

if __name__ == "__main__":
    PREVIOUS_SIGTERM_CALLBACK = signal(SIGTERM, exit_gracefully)
    THREAD_LOCK = Lock()
    ACTIVE_KEYS = dict()

    key_listener = keyboard.Listener(on_press=handle_key_press, on_release=handle_key_release)

    print HELP_MSG

    scr = initscr()
    cbreak()
    noecho()

    key_listener.start()
    update_commanded_joint_efforts()
    key_listener.join()

    endwin()
