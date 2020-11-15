#!/usr/bin/env python
import rospy
from argparse import ArgumentParser
from tf import transformations as tft
from math import radians as rads

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SetModelConfiguration

NODE_NAME = "srr_teleop_force_set_positions"
SUB_MODEL_STATES_TOPIC = "/gazebo/model_states"
CLI_SET_MODEL_STATE_TOPIC = "/gazebo/set_model_state"
CLI_SET_MODEL_CONFIGURATION_TOPIC = "/gazebo/set_model_configuration"

APPLICATION_HELP = """
Use this script to force individual joints of the SRR vehicle model
 to specified positions. In the event that gravity is acting on the
 model and positions want to be maintained, the service can be called
 periodically to maintain the illusion that the specified joints are
 statically set. Example usage for a model whose joint names includes
 'FL_Joint' and 'FR_Joint' and wants the service called 30 times a
 second: \"rosrun srr_teleop force_set_positions.py --joint FL_Joint
 -90 --joint FR_Joint -90 --hz 30\"
"""

class ROSWrapper(object):

    model_name_is_set        = False
    model_name               = None
    init_vehicle_pose_is_set = False
    init_vehicle_pose        = None

    call_rate       = None
    joint_names     = None
    joint_positions = None
    modelX          = None
    modelY          = None
    modelZ          = None
    modelAxisAngle  = None

    objective_model_state_is_set = None
    objective_model_state        = None

    cli_set_model_state         = None
    cli_set_model_configuration = None

    def __init__(self, hz, joint_names, joint_positions, modelX, modelY, modelZ, modelAxisAngle):
        rospy.init_node(NODE_NAME)

        self.model_name_is_set,        self.model_name        = ROSWrapper.get_rosparam("/srr_model_name")
        self.init_vehicle_pose_is_set, self.init_vehicle_pose = ROSWrapper.get_rosparam("/srr_init_pose")

        self.call_rate       = None if hz < 1 else rospy.Rate(hz)
        self.joint_names     = joint_names
        self.joint_positions = joint_positions
        self.modelX          = modelX
        self.modelY          = modelY
        self.modelZ          = modelZ
        self.modelAxisAngle  = modelAxisAngle

        self.cli_set_model_state         = rospy.ServiceProxy(CLI_SET_MODEL_STATE_TOPIC, SetModelState)
        self.cli_set_model_configuration = rospy.ServiceProxy(CLI_SET_MODEL_CONFIGURATION_TOPIC, SetModelConfiguration)

        self.objective_model_state        = None
        self.objective_model_state_is_set = self.build_goal_model_state()

    def has_call_rate(self):
        return self.call_rate is not None

    def build_goal_model_state(self):
        msg = None
        try:
            msg = rospy.wait_for_message(SUB_MODEL_STATES_TOPIC, ModelStates, timeout=2)
        except rospy.exceptions.ROSException:
            print "WARNING: Did not hear model state on {0}.".format(SUB_MODEL_STATES_TOPIC)
            return False

        for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
                self.objective_model_state = ModelState(model_name=msg.name[i], pose=msg.pose[i], twist=msg.twist[i], reference_frame="map")
                break

        if self.objective_model_state is None:
            return False
        else:
            cx = self.objective_model_state.pose.position.x; cy = self.objective_model_state.pose.position.y; cz = self.objective_model_state.pose.position.z
            cR, cP, cY = tft.euler_from_quaternion([
                self.objective_model_state.pose.orientation.x,
                self.objective_model_state.pose.orientation.y,
                self.objective_model_state.pose.orientation.z,
                self.objective_model_state.pose.orientation.w
            ])
            ix = None; iy = None; iz = None
            iR = None; iP = None; iY = None
            if self.init_vehicle_pose_is_set:
                try:
                    vals = {}
                    for val in self.init_vehicle_pose.split("-"):
                        val = val.strip()
                        if len(val) > 0:
                            space = val.index(" ")
                            vals[val[0:space].strip()] = float(val[space+1:].strip())
                    ix = vals["x"];       iy = vals["y"];       iz = vals["z"]
                    iR = rads(vals["R"]); iP = rads(vals["P"]); iY = rads(vals["Y"])
                except:
                    print "Error parsing init pose \"{0}\", assuming no initial pose provided.".format(self.init_vehicle_pose)

            if self.modelX == "current":
                self.objective_model_state.pose.position.x = cx
            elif self.modelX == "init":
                if ix is None:
                    print "Could not get inital X position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.x = cx
                else:
                    self.objective_model_state.pose.position.x = ix
            else:
                try:
                    self.objective_model_state.pose.position.x = rads(float(self.modelX))
                except ValueError as e:
                    print "Could not set inital X position from {0}, defaulting to current.".format(self.modelX)
                    self.objective_model_state.pose.position.x = cx

            if self.modelY == "current":
                self.objective_model_state.pose.position.y = cy
            elif self.modelY == "init":
                if iy is None:
                    print "Could not get inital Y position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.y = cy
                else:
                    self.objective_model_state.pose.position.y = iy
            else:
                try:
                    self.objective_model_state.pose.position.y = rads(float(self.modelY))
                except ValueError as e:
                    print "Could not set inital Y position from {0}, defaulting to current.".format(self.modelY)
                    self.objective_model_state.pose.position.y = cy

            if self.modelZ == "current":
                self.objective_model_state.pose.position.z = cz
            elif self.modelZ == "init":
                if iz is None:
                    print "Could not get inital Z position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.z = cz
                else:
                    self.objective_model_state.pose.position.z = iz
            else:
                try:
                    self.objective_model_state.pose.position.z = rads(float(self.modelZ))
                except ValueError as e:
                    print "Could not set inital Z position from {0}, defaulting to current.".format(self.modelZ)
                    self.objective_model_state.pose.position.z = cz

            oR = None; oP = None; oY = None
            if self.modelAxisAngle == "current":
                oR = cR; oP = cP; oY = cY
            elif self.modelAxisAngle == "init":
                if iz is None:
                    print "Could not get inital rotation from global rosparam server, defaulting to current."
                    oR = cR; oP = cP; oY = cY
                else:
                    oR = iR; oP = iP; oY = iY
            else:
                try:
                    aT, aX, aY, aZ = [float(o) for o in self.modelAxisAngle.split(",")]
                    aT = rads(aT)
                    oR, oP, oY = tft.euler_from_matrix(tft.rotation_matrix(aT, [aX, aY, aZ]))
                except (ValueError, IndexError), e:
                    print "Could not set inital rotation from {0}, defaulting to current.".format(self.modelAxisAngle)
                    oR = cR; oP = cP; oY = cY

            self.objective_model_state.pose.orientation.x, \
            self.objective_model_state.pose.orientation.y, \
            self.objective_model_state.pose.orientation.z, \
            self.objective_model_state.pose.orientation.w = tft.quaternion_from_euler(oR, oP, oY)

            return True

    def call_cli_set_model_configuration(self):
        return self.cli_set_model_configuration(
            model_name=self.model_name,
            urdf_param_name=self.model_name,
            joint_names=self.joint_names,
            joint_positions=self.joint_positions
        )

    def call_cli_set_model_state(self):
        return self.cli_set_model_state(model_state=self.objective_model_state)

    def is_shutdown(self):
        return rospy.is_shutdown()

    @staticmethod
    def get_rosparam(name):
        value = None
        is_set = rospy.has_param(name)
        if is_set:
            value = rospy.get_param(name)
        return is_set, value

def main():
    parser = ArgumentParser(description=APPLICATION_HELP)
    parser.add_argument(
        "--hz",
        help="the number of calls to the service to make every second",
        type=int,
        default=-1
    )
    parser.add_argument(
        "-j", "--joint",
        help="""Usage: '--joint MyJointName JointPosDegrees (the name of the joint and the decimal position (in degrees) to
 set it to, can be repeated as many times as desired / as many joints are available to be set)""",
        action="append",
        nargs="*"
    )
    parser.add_argument(
        "--mx",
        help="""The x position to set the model to. Options: (1) a decimal position relative to the world frame, (2) \"init\"
 (without the quotes) to set it back to the initial position, (3) \"current\" (without the quotes) to not change it (this is
 the default value)""",
        default="current"
    )
    parser.add_argument(
        "--my",
        help="The y position to set the model to. The options for --mx apply here too.",
        default="current"
    )
    parser.add_argument(
        "--mz",
        help="The z position to set the model to. The options for --mx apply here too.",
        default="current"
    )
    parser.add_argument(
        "--maa",
        help="""The axis-angle rotation to set the model to. Options: (1) \"th,x,y,z\" (without the quotes, separated by commas,
 no spaces), where th is the rotation about the axis with components x, y, and z, (2) \"init\" (without the quotes) to set it
 back to the initial position, (3) \"current\" (without the quotes) to not change it (this is the default value)""",
        default="current"
    )

    args = parser.parse_args()
    joint_names     = []
    joint_positions = []
    if args.joint is not None:
        for j in args.joint:
            if len(j) != 2:
                print "Each value for --joint must be a joint name and a position (in degrees) separated by a space. Exiting."
                return
            joint_names.append(j[0])
            joint_positions.append(rads(float(j[1])))

    wrapper = ROSWrapper(args.hz, joint_names, joint_positions, args.mx, args.my, args.mz, args.maa)

    if not wrapper.model_name_is_set:
        print "Model name is not set in global rosparam server."
        return

    try:
        while not wrapper.is_shutdown():
            if wrapper.objective_model_state_is_set:
                srv_response = wrapper.call_cli_set_model_state()
                if not srv_response.success:
                    print "Exiting, as ROS service call to {0} failed: {1}".format(CLI_SET_MODEL_STATE_TOPIC, srv_response.status_message)
                    break
            else:
                print "WARNING: Objective model state not set. Skipping."
            srv_response = wrapper.call_cli_set_model_configuration()
            if not srv_response.success:
                print "Exiting, as ROS service call to {0} failed: {1}".format(CLI_SET_MODEL_CONFIGURATION_TOPIC, srv_response.status_message)
                break
            if not wrapper.has_call_rate():
                print "Task finished successfully. Exiting."
                break
            else:
                wrapper.call_rate.sleep()
    except rospy.service.ServiceException as e:
        print "Caught a service exception: {0}".format(e)
    except rospy.exceptions.ROSInterruptException:
        print "Exiting."

if __name__ == "__main__":
    main()
