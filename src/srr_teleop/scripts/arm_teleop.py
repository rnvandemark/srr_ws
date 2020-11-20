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

MAG_JOINT = 12.5

PREVIOUS_SIGTERM_CALLBACK = None
THREAD_LOCK = None
SHUTDOWN_REQUESTED = False
ACTIVE_KEYS = None
NODE_SLEEP_S = 0.05

BINDINGS_MOVE = {
# Sets the direction of movement for the Base, Shoulder, Elbow, and Wrist joints
    'q': (+1,  0,   0,   0),
    'a': (-1,  0,   0,   0),
    'w': ( 0, +1,   0,   0),
    's': ( 0, -1,   0,   0),
    'e': ( 0,  0,  +1,   0),
    'd': ( 0,  0,  -1,   0),
    'r': ( 0,  0,   0,  +1),
    'f': ( 0,  0,   0,  -1)
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
        "/sra_integrated/{0}_joint_controller/command".format(name),
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

    rospy.init_node("sra_arm_teleop")

    pub_base     = create_pub("base")
    pub_shoulder = create_pub("shoulder")
    pub_elbow    = create_pub("elbow")
    pub_wrist    = create_pub("wrist")

    speed_base     = 0
    speed_shoulder = 0
    speed_elbow    = 0
    speed_wrist    = 0

    try:
        shutdown_requested = False
        while not shutdown_requested:
            THREAD_LOCK.acquire()
            try:
                shutdown_requested = SHUTDOWN_REQUESTED
                forced_stop = False
                for key, event_effect in ACTIVE_KEYS.items():
                    if key == keyboard.Key.space:
                        speed_base     = 0
                        speed_shoulder = 0
                        speed_elbow    = 0
                        speed_wrist    = 0
                        forced_stop    = True
                        break
                    elif type(key) is keyboard.KeyCode:
                        ch = key.char
                        if ch in BINDINGS_MOVE.keys():
                            speed_base     = speed_base     + (BINDINGS_MOVE[ch][0] * event_effect * MAG_JOINT)
                            speed_shoulder = speed_shoulder + (BINDINGS_MOVE[ch][1] * event_effect * MAG_JOINT)
                            speed_elbow    = speed_elbow    + (BINDINGS_MOVE[ch][2] * event_effect * MAG_JOINT)
                            speed_wrist    = speed_wrist    + (BINDINGS_MOVE[ch][3] * event_effect * MAG_JOINT)

                    if event_effect == 1:
                        ACTIVE_KEYS[key] = 0
                    elif event_effect == -1:
                        ACTIVE_KEYS.pop(key)

                if forced_stop:
                    ACTIVE_KEYS.clear()
            finally:
                THREAD_LOCK.release()

            pub_base.publish(speed_base)
            pub_shoulder.publish(speed_shoulder)
            pub_elbow.publish(speed_elbow)
            pub_wrist.publish(speed_wrist)

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
