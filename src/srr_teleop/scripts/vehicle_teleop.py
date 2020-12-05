#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from srr_msgs.srv import GetDirectionOfRotation

from enum import Enum
from threading import Lock
from pynput import keyboard
from signal import signal, SIGTERM
from time import sleep
from curses import initscr, cbreak, noecho, endwin

# L=Left, R=Right, F=Front, B=Back/Rear, W=Wheels, C=Chassis, S=Steer
class SRRJoints(Enum):
    FLW = 1
    FRW = 2
    BLW = 3
    BRW = 4
    FLS = 5
    FRS = 6
    FLC = 7
    FRC = 8
    BLC = 9
    BRC = 10

class CntrlType(Enum):
    POS = 1
    EFF = 2
    VEL = 3

HELP_MSG = """
---------------------------
**Displaying movement controls is a TODO**
space key : force stop
Press ESCAPE to quit
---------------------------
"""

INC_SPEED = 1.0
INC_ROT   = 1.0

MAG_WHEELS  = 12.5
MAG_CHASSIS = 0.1
MAG_STEER   = 100.0

PREVIOUS_SIGTERM_CALLBACK = None
THREAD_LOCK = None
SHUTDOWN_REQUESTED = False
ACTIVE_KEYS = None
NODE_SLEEP_S = 0.05
NOISE_THRESHOLD = 0.001

def create_joint_dict(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9):
    return {
        SRRJoints.FLW: v0,
        SRRJoints.FRW: v1,
        SRRJoints.BLW: v2,
        SRRJoints.BRW: v3,
        SRRJoints.FLS: v4,
        SRRJoints.FRS: v5,
        SRRJoints.FLC: v6,
        SRRJoints.FRC: v7,
        SRRJoints.BLC: v8,
        SRRJoints.BRC: v9
    }

def create_joint_dict_mag(m, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9):
    return create_joint_dict(
        m * v0, m * v1, m * v2, m * v3, m * v4,
        m * v5, m * v6, m * v7, m * v8, m * v9
    )

SRR_JOINT_NAMES = {
    "FL_Wheel_Joint": SRRJoints.FLW,
    "FR_Wheel_Joint": SRRJoints.FRW,
    "RL_Wheel_Joint": SRRJoints.BLW,
    "RR_Wheel_Joint": SRRJoints.BRW,
    "FL_Rev_Joint":   SRRJoints.FLS,
    "FR_Rev_Joint":   SRRJoints.FRS,
    "FL_Joint":       SRRJoints.FLC,
    "FR_Joint":       SRRJoints.FRC,
    "RL_Joint":       SRRJoints.BLC,
    "RR_Joint":       SRRJoints.BRC
}

CONTROLLER_TYPES = create_joint_dict(
    CntrlType.VEL, CntrlType.VEL, CntrlType.VEL, CntrlType.VEL,
    CntrlType.POS, CntrlType.POS,
    CntrlType.POS, CntrlType.POS, CntrlType.POS, CntrlType.POS
)

BINDINGS_MOVE = {                          #FLW FRW BLW BRW FLS FRS FLC FRC BLC BRC
    'w': create_joint_dict_mag(MAG_WHEELS,  +1, +1, +1, +1,  0,  0,  0,  0,  0,  0),
    's': create_joint_dict_mag(MAG_WHEELS,  -1, -1, -1, -1,  0,  0,  0,  0,  0,  0),
    'a': create_joint_dict_mag(MAG_STEER,    0,  0,  0,  0, +1, +1,  0,  0,  0,  0),
    'd': create_joint_dict_mag(MAG_STEER,    0,  0,  0,  0, -1, -1,  0,  0,  0,  0),
    'o': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, +1, +1, +1, +1),
    'p': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, -1, -1, -1, -1),
    'i': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, -1, -1, +1, +1),
    'k': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, +1, +1, -1, -1),
    'j': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, -1, +1, -1, +1),
    'l': create_joint_dict_mag(MAG_CHASSIS,  0,  0,  0,  0,  0,  0, +1, -1, +1, -1)
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

    srv_dir_rotation = rospy.ServiceProxy("/srr_integrated/get_direction_of_rotation", GetDirectionOfRotation)

    joint_pubs = create_joint_dict(
        create_pub("front_left_wheel"),
        create_pub("front_right_wheel"),
        create_pub("rear_left_wheel"),
        create_pub("rear_right_wheel"),
        create_pub("front_left_steer"),
        create_pub("front_right_steer"),
        create_pub("front_left"),
        create_pub("front_right"),
        create_pub("rear_left"),
        create_pub("rear_right")
    )

    joint_vals = create_joint_dict(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    try:
        wheel_joint_names = ["FL_Wheel_Joint", "FR_Wheel_Joint", "RL_Wheel_Joint", "RR_Wheel_Joint"]
        srv_response = srv_dir_rotation(joint_names=wheel_joint_names)
        wheel_directions = srv_response.joint_directions
        for ch in ['w', 's']:
            for i in range(len(wheel_joint_names)):
                wjn = SRR_JOINT_NAMES[wheel_joint_names[i]]
                BINDINGS_MOVE[ch][wjn] = BINDINGS_MOVE[ch][wjn] * wheel_directions[i]

        shutdown_requested = False
        while not shutdown_requested:
            THREAD_LOCK.acquire()
            try:
                shutdown_requested = SHUTDOWN_REQUESTED
                forced_stop = False
                for key, event_effect in ACTIVE_KEYS.items():
                    if key == keyboard.Key.space:
                        for j in SRRJoints:
                            joint_vals[j] = 0
                        forced_stop = True
                        break
                    elif type(key) is keyboard.KeyCode:
                        ch = key.char
                        if ch in BINDINGS_MOVE.keys():
                            for j in SRRJoints:
                                move = BINDINGS_MOVE[ch][j]
                                cntrl = CONTROLLER_TYPES[j]
                                if cntrl == CntrlType.POS:
                                    joint_vals[j] = joint_vals[j] + move
                                elif cntrl == CntrlType.EFF:
                                    pass
                                elif cntrl == CntrlType.VEL:
                                    joint_vals[j] = joint_vals[j] + (move * event_effect)
                                    if abs(joint_vals[j]) < NOISE_THRESHOLD:
                                        joint_vals[j] = 0

                    if event_effect == 1:
                        ACTIVE_KEYS[key] = 0
                    elif event_effect == -1:
                        ACTIVE_KEYS.pop(key)

                if forced_stop:
                    ACTIVE_KEYS.clear()
            finally:
                THREAD_LOCK.release()

            for j in SRRJoints:
                joint_pubs[j].publish(joint_vals[j])

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
