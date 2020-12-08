#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
#from srr_msgs.srv import GetDirectionOfRotation

from enum import Enum
from threading import Lock
from pynput import keyboard
from signal import signal, SIGTERM
from time import sleep
from curses import initscr, cbreak, noecho, endwin

class SRAJoints(Enum):
    BASE = 1
    SHOULDER = 2
    ELBOW = 3
    WRIST = 4
    FINGER_L = 5
    FINGER_R = 6

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

MAG_ARM          = 12.5
POS_FINGER_OPEN  = 0
POS_FINGER_CLOSE = 0

PREVIOUS_SIGTERM_CALLBACK = None
THREAD_LOCK = None
SHUTDOWN_REQUESTED = False
ACTIVE_KEYS = None
NODE_SLEEP_S = 0.05
NOISE_THRESHOLD = 0.001

def create_joint_dict(v0, v1, v2, v3, v4, v5):
    return {
        SRAJoints.BASE: v0,
        SRAJoints.SHOULDER: v1,
        SRAJoints.ELBOW: v2,
        SRAJoints.WRIST: v3,
        SRAJoints.FINGER_L: v4,
        SRAJoints.FINGER_R: v5
    }

def create_joint_dict_mag(m, v0, v1, v2, v3, v4, v5):
    return create_joint_dict(m * v0, m * v1, m * v2, m * v3, m * v4, m * v5)

SRR_JOINT_NAMES = {
    "BaseJoint":     SRAJoints.BASE,
    "ShoulderJoint": SRAJoints.SHOULDER,
    "ElbowJoint":    SRAJoints.ELBOW,
    "WristJoint":    SRAJoints.WRIST,
    "LFingerJoint":  SRAJoints.FINGER_L,
    "RFingerJoint":  SRAJoints.FINGER_R
}

CONTROLLER_TYPES = create_joint_dict(
    CntrlType.VEL, CntrlType.VEL, CntrlType.VEL, CntrlType.VEL,
    CntrlType.POS, CntrlType.POS
)

BINDINGS_MOVE = {
# Sets the direction of movement for the Base, Shoulder, Elbow, and Wrist joints
    'q': create_joint_dict_mag(MAG_ARM, +1,  0,  0,  0,  0,  0),
    'a': create_joint_dict_mag(MAG_ARM, -1,  0,  0,  0,  0,  0),
    'w': create_joint_dict_mag(MAG_ARM,  0, +1,  0,  0,  0,  0),
    's': create_joint_dict_mag(MAG_ARM,  0, -1,  0,  0,  0,  0),
    'e': create_joint_dict_mag(MAG_ARM,  0,  0, +1,  0,  0,  0),
    'd': create_joint_dict_mag(MAG_ARM,  0,  0, -1,  0,  0,  0),
    'r': create_joint_dict_mag(MAG_ARM,  0,  0,  0, +1,  0,  0),
    'f': create_joint_dict_mag(MAG_ARM,  0,  0,  0, -1,  0,  0),
    't': create_joint_dict_mag(MAG_ARM,  0,  0,  0,  0,  0,  0),
    'g': create_joint_dict_mag(MAG_ARM,  0,  0,  0,  0,  0,  0)
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

    #srv_dir_rotation = rospy.ServiceProxy("/sra_integrated/get_direction_of_rotation", GetDirectionOfRotation)

    joint_pubs = create_joint_dict(
        create_pub("base"),
        create_pub("shoulder"),
        create_pub("elbow"),
        create_pub("wrist"),
        create_pub("finger_left"),
        create_pub("finger_right")
    )

    joint_vals = create_joint_dict(0, 0, 0, 0, 0, 0)

    try:
        #joint_names = list(SRR_JOINT_NAMES.keys())
        #print joint_names
        #srv_response = srv_dir_rotation(joint_names=joint_names)
        #joint_directions = srv_response.joint_directions
        #for ch in BINDINGS_MOVE:
        #    for i in range(len(joint_names)):
        #        wjn = SRR_JOINT_NAMES[joint_names[i]]
        #        BINDINGS_MOVE[ch][wjn] = BINDINGS_MOVE[ch][wjn] * joint_directions[i]

        shutdown_requested = False
        while not shutdown_requested:
            THREAD_LOCK.acquire()
            try:
                shutdown_requested = SHUTDOWN_REQUESTED
                forced_stop = False
                for key, event_effect in ACTIVE_KEYS.items():
                    if key == keyboard.Key.space:
                        for j in SRAJoints:
                            joint_vals[j] = 0
                        forced_stop = True
                        break
                    elif type(key) is keyboard.KeyCode:
                        ch = key.char
                        if ch in BINDINGS_MOVE.keys():
                            for j in SRAJoints:
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

            for j in SRAJoints:
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
