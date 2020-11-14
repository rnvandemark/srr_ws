#!/usr/bin/env python
import rospy
from argparse import ArgumentParser

from gazebo_msgs.srv import SetModelConfiguration

NODE_NAME = "srr_teleop_force_set_positions"
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

    model_name_is_set           = False
    model_name                  = None
    call_rate                   = None
    joint_names                 = None
    joint_positions             = None
    cli_set_model_configuration = None

    def __init__(self, hz, joint_names, joint_positions):
        rospy.init_node(NODE_NAME)

        self.model_name_is_set, self.model_name = ROSWrapper.get_rosparam("/srr_model_name")
        self.call_rate                          = None if hz < 1 else rospy.Rate(hz)
        self.joint_names                        = joint_names
        self.joint_positions                    = joint_positions

        self.cli_set_model_configuration = rospy.ServiceProxy(CLI_SET_MODEL_CONFIGURATION_TOPIC, SetModelConfiguration)

    def model_is_valid(self):
        return self.model_name_is_set

    def get_call_rate(self):
        return self.call_rate

    def has_call_rate(self):
        return self.get_call_rate() is not None

    def call_cli_set_model_configuration(self):
        return self.cli_set_model_configuration(
            model_name=self.model_name,
            urdf_param_name=self.model_name,
            joint_names=self.joint_names,
            joint_positions=self.joint_positions
        )

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
        "-z", "--hz",
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

    args = parser.parse_args()
    if len(args.joint) == 0:
        print "No joints given, nothing to do."
        return
    joint_names     = []
    joint_positions = []
    for j in args.joint:
        if len(j) != 2:
            print "Each value for --joint must be a joint name and a position (in degrees) separated by a space. Exiting."
            return
        joint_names.append(j[0])
        joint_positions.append(float(j[1]))

    wrapper = ROSWrapper(args.hz, joint_names, joint_positions)

    if not wrapper.model_is_valid():
        print "Model name is not set in global rosparam server."
        return

    try:
        while not wrapper.is_shutdown():
            srv_response = wrapper.call_cli_set_model_configuration()
            if not srv_response.success:
                print "Exiting, as ROS service call to {0} failed: {1}".format(CLI_SET_MODEL_CONFIGURATION_TOPIC, srv_response.status_message)
                break
            elif not wrapper.has_call_rate():
                print "Task finished successfully. Exiting."
                break
            else:
                wrapper.get_call_rate().sleep()
    except rospy.service.ServiceException as e:
        print "Caught a service exception: " + str(e)
    except rospy.exceptions.ROSInterruptException:
        print "Exiting."

if __name__ == "__main__":
    main()
