#!/usr/bin/env python
import rospy
from argparse import ArgumentParser
from tf import transformations as tft
from math import radians as rads

from srr_msgs.srv import CalculatePositionWithOffsets
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SetModelConfiguration

NODE_NAME = "srr_teleop_force_set_positions"
SUB_MODEL_STATES_TOPIC = "/gazebo/model_states"
CLI_SET_MODEL_STATE_TOPIC = "/gazebo/set_model_state"
CLI_CALCULATE_POSITION_WITH_OFFSETS_TOPIC_SUFFIX = "calculate_position_with_offsets"
CLI_SET_MODEL_CONFIGURATION_TOPIC = "/gazebo/set_model_configuration"

APPLICATION_HELP = """
Use this script to force individual joints of the SRR vehicle model
 and/or SRA arm model to specified positions. In the event that gravity
 is acting on the model(s) and positions want to be maintained, the
 service can be called periodically to maintain the illusion that the
 specified joints are statically set. Example usage for a model whose
 joint names from the vehicle includes 'FL_Joint' and 'FR_Joint', as
 well as joint names from a mounted arm include 'BaseJoint' and
 'WristJoint', and also wants the service called 30 times a second:
 \"rosrun srr_teleop force_set_positions.py --jr FL_Joint -90 --jr
 FR_Joint -90 --ja BaseJoint 45 --ja WristJoint -30 --hz 30\"
"""

class ROSWrapper(object):

    vehicle_model_name  = None
    arm_model_name      = None
    combined_model_name = None
    init_model_pose     = None

    vehicle_joint_names     = None
    vehicle_joint_positions = None
    vehicle_fixed_positions = None
    arm_joint_names         = None
    arm_joint_positions     = None
    arm_fixed_positions     = None

    call_rate      = None
    modelX         = None
    modelY         = None
    modelZ         = None
    modelAxisAngle = None

    cli_set_model_state                         = None
    cli_set_model_configuration                 = None
    cli_calculate_vehicle_position_with_offsets = None
    cli_calculate_arm_position_with_offsets     = None
    objective_model_state                       = None

    def __init__(self,
                 hz,
                 vehicle_joint_names,
                 vehicle_joint_positions,
                 arm_joint_names,
                 arm_joint_positions,
                 modelX,
                 modelY,
                 modelZ,
                 modelAxisAngle
    ):
        rospy.init_node(NODE_NAME)

        self.vehicle_model_name = ROSWrapper.get_rosparam("/srr_model_name")
        if self.vehicle_model_name:
            self.init_model_pose = ROSWrapper.get_rosparam("/{0}/init_pose".format(self.vehicle_model_name))

        self.arm_model_name = ROSWrapper.get_rosparam("/sra_model_name")
        if self.arm_model_name and not self.init_model_pose:
            self.init_model_pose = ROSWrapper.get_rosparam("/{0}/init_pose".format(self.arm_model_name))

        self.combined_model_name = ROSWrapper.get_rosparam("/combined_model_name")

        self.vehicle_joint_names     = vehicle_joint_names
        self.vehicle_joint_positions = vehicle_joint_positions
        self.vehicle_fixed_positions = None
        self.arm_joint_names         = arm_joint_names
        self.arm_joint_positions     = arm_joint_positions
        self.arm_fixed_positions     = None

        self.call_rate      = None if hz < 1 else rospy.Rate(hz)
        self.modelX         = modelX
        self.modelY         = modelY
        self.modelZ         = modelZ
        self.modelAxisAngle = modelAxisAngle

        self.cli_set_model_state = rospy.ServiceProxy(CLI_SET_MODEL_STATE_TOPIC, SetModelState)
        self.cli_set_model_configuration = rospy.ServiceProxy(CLI_SET_MODEL_CONFIGURATION_TOPIC, SetModelConfiguration)
        if self.vehicle_model_name:
            self.cli_calculate_vehicle_position_with_offsets = rospy.ServiceProxy(
                "/{0}/{1}".format(self.vehicle_model_name, CLI_CALCULATE_POSITION_WITH_OFFSETS_TOPIC_SUFFIX),
                CalculatePositionWithOffsets
            )
            srv_response = self.cli_calculate_vehicle_position_with_offsets(
                joint_names=self.vehicle_joint_names,
                goal_positions=self.vehicle_joint_positions
            )
            self.vehicle_fixed_positions = srv_response.joint_positions
        if self.arm_model_name:
            self.cli_calculate_arm_position_with_offsets = rospy.ServiceProxy(
                "/{0}/{1}".format(self.arm_model_name, CLI_CALCULATE_POSITION_WITH_OFFSETS_TOPIC_SUFFIX),
                CalculatePositionWithOffsets
            )
            srv_response = self.cli_calculate_arm_position_with_offsets(
                joint_names=self.arm_joint_names,
                goal_positions=self.arm_joint_positions
            )
            self.arm_fixed_positions = srv_response.joint_positions

        self.build_goal_model_state()

    def has_call_rate(self):
        return self.call_rate is not None

    def build_goal_model_state(self):
        self.objective_model_state = None

        msg = None
        try:
            msg = rospy.wait_for_message(SUB_MODEL_STATES_TOPIC, ModelStates, timeout=2)
        except rospy.exceptions.ROSException:
            print "WARNING: Did not hear model state on {0}.".format(SUB_MODEL_STATES_TOPIC)
            return

        model_name = self.get_model_name()
        for i in range(len(msg.name)):
            if msg.name[i] == model_name:
                self.objective_model_state = ModelState(model_name=model_name, pose=msg.pose[i], twist=msg.twist[i], reference_frame="map")
                break

        if self.objective_model_state:
            cx = self.objective_model_state.pose.position.x; cy = self.objective_model_state.pose.position.y; cz = self.objective_model_state.pose.position.z
            cR, cP, cY = tft.euler_from_quaternion([
                self.objective_model_state.pose.orientation.x,
                self.objective_model_state.pose.orientation.y,
                self.objective_model_state.pose.orientation.z,
                self.objective_model_state.pose.orientation.w
            ])
            ix = None; iy = None; iz = None
            iR = None; iP = None; iY = None
            if self.init_model_pose:
                try:
                    vals = {}
                    for val in self.init_model_pose.split("-"):
                        val = val.strip()
                        if len(val) > 0:
                            space = val.index(" ")
                            vals[val[0:space].strip()] = float(val[space+1:].strip())
                    ix = vals["x"];       iy = vals["y"];       iz = vals["z"]
                    iR = rads(vals["R"]); iP = rads(vals["P"]); iY = rads(vals["Y"])
                except:
                    print "Error parsing init pose \"{0}\", assuming no initial pose provided.".format(self.init_model_pose)

            if self.modelX == "current":
                self.objective_model_state.pose.position.x = cx
            elif self.modelX == "init":
                if ix is None:
                    print "Could not get initial X position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.x = cx
                else:
                    self.objective_model_state.pose.position.x = ix
            else:
                try:
                    self.objective_model_state.pose.position.x = float(self.modelX)
                except ValueError as e:
                    print "Could not set initial X position from {0}, defaulting to current.".format(self.modelX)
                    self.objective_model_state.pose.position.x = cx

            if self.modelY == "current":
                self.objective_model_state.pose.position.y = cy
            elif self.modelY == "init":
                if iy is None:
                    print "Could not get initial Y position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.y = cy
                else:
                    self.objective_model_state.pose.position.y = iy
            else:
                try:
                    self.objective_model_state.pose.position.y = float(self.modelY)
                except ValueError as e:
                    print "Could not set inital Y position from {0}, defaulting to current.".format(self.modelY)
                    self.objective_model_state.pose.position.y = cy

            if self.modelZ == "current":
                self.objective_model_state.pose.position.z = cz
            elif self.modelZ == "init":
                if iz is None:
                    print "Could not get initial Z position from global rosparam server, defaulting to current."
                    self.objective_model_state.pose.position.z = cz
                else:
                    self.objective_model_state.pose.position.z = iz
            else:
                try:
                    self.objective_model_state.pose.position.z = float(self.modelZ)
                except ValueError as e:
                    print "Could not set initial Z position from {0}, defaulting to current.".format(self.modelZ)
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

    def call_cli_set_model_configuration(self):
        model_name = self.get_model_name()
        joint_names = []
        fixed_positions = []
        if self.vehicle_joint_names:
            joint_names.extend(self.vehicle_joint_names)
            fixed_positions.extend(self.vehicle_fixed_positions)
        if self.arm_joint_names:
            joint_names.extend(self.arm_joint_names)
            fixed_positions.extend(self.arm_fixed_positions)
        return self.cli_set_model_configuration(
            model_name=model_name,
            urdf_param_name=model_name,
            joint_names=joint_names,
            joint_positions=fixed_positions
        )

    def call_cli_set_model_state(self):
        return self.cli_set_model_state(model_state=self.objective_model_state)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def get_model_name(self):
        return self.combined_model_name if self.combined_model_name else (
            self.vehicle_model_name if self.vehicle_model_name else self.arm_model_name
        )

    @staticmethod
    def get_rosparam(name):
        value = None
        if rospy.has_param(name):
            value = rospy.get_param(name)
        return value

def main():
    parser = ArgumentParser(description=APPLICATION_HELP)
    parser.add_argument(
        "--hz",
        help="the number of calls to the service to make every second",
        type=int,
        default=-1
    )
    parser.add_argument(
        "--jr",
        help="""Usage: '--jr VehicleJointName JointPosDegrees (the name of the joint on the vehicle and the decimal position
 (in degrees) to set it to, can be repeated as many times as desired / as many joints are available to be set)""",
        action="append",
        nargs="*"
    )
    parser.add_argument(
        "--ja",
        help="""Usage: '--ja ArmJointName JointPosDegrees (the name of the joint on the arm and the decimal position
 (in degrees) to set it to, can be repeated as many times as desired / as many joints are available to be set)""",
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
    vehicle_joint_names     = []
    vehicle_joint_positions = []
    arm_joint_names         = []
    arm_joint_positions     = []
    if args.jr is not None:
        for j in args.jr:
            if len(j) != 2:
                print "Each value for --jr must be a joint name and a position (in degrees) separated by a space. Exiting."
                return
            vehicle_joint_names.append(j[0])
            vehicle_joint_positions.append(rads(float(j[1])))
    if args.ja is not None:
        for j in args.ja:
            if len(j) != 2:
                print "Each value for --ja must be a joint name and a position (in degrees) separated by a space. Exiting."
                return
            arm_joint_names.append(j[0])
            arm_joint_positions.append(rads(float(j[1])))

    wrapper = ROSWrapper(args.hz,
                         vehicle_joint_names,
                         vehicle_joint_positions,
                         arm_joint_names,
                         arm_joint_positions,
                         args.mx,
                         args.my,
                         args.mz,
                         args.maa
    )

    if not wrapper.get_model_name():
        print "Model name is not set in global rosparam server."
        return

    try:
        while not wrapper.is_shutdown():
            if wrapper.objective_model_state:
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
