#!/usr/bin/env python
import rospy
from time import sleep

from srr_msgs.srv import CalculateVehicleVelKin, CalculateArmInvKin, CalculateCombinedKin
from srr_msgs.msg import VehiclePath, VehicleVelKinDebug, VehicleVelKinSolution, ArmCartesianPose, CombinedProgram

class VehicleVelKinCaller(object):
    cli_calc_vehicle_vel_kin       = None
    sub_calc_vehicle_vel_kin_debug = None
    msg_calc_vehicle_vel_kin_debug = None
    pub_vehicle_program            = None

    def __init__(self, node_name, cli_topic, sub_debug_topic=None, pub_program_topic=None):
        if node_name:
            rospy.init_node(node_name, disable_signals=True)

        self.cli_calc_vehicle_vel_kin = rospy.ServiceProxy(cli_topic, CalculateVehicleVelKin)
        self.sub_calc_vehicle_vel_kin_debug = rospy.Subscriber(
            sub_debug_topic,
            VehicleVelKinDebug,
            self.handle_debug_callback,
            queue_size=1
        )
        if pub_program_topic:
            self.pub_vehicle_program = rospy.Publisher(pub_program_topic, VehicleVelKinSolution, queue_size=1)

    def call(self, tf, tb, odot, wp, waiti_s=0, waitf_s=0):
        sleep(waiti_s)
        response = self.cli_calc_vehicle_vel_kin(
            theta_front=tf,
            theta_back=tb,
            omega_dot_max=odot,
            path=VehiclePath(waypoints=wp)
        )
        if self.pub_vehicle_program:
            self.pub_vehicle_program.publish(response.solution)
        if self.sub_calc_vehicle_vel_kin_debug:
            sleep(waitf_s)
            rospy.spin();
        return response, self.msg_calc_vehicle_vel_kin_debug

    def handle_debug_callback(self, msg):
        self.msg_calc_vehicle_vel_kin_debug = msg
        rospy.signal_shutdown("Program finished.")

class CombinedKinCaller(object):
    cli_calc_combined_kin = None
    pub_combined_program  = None

    def __init__(self, node_name, cli_topic, pub_program_topic=None):
        rospy.init_node(node_name, disable_signals=True)
        self.cli_calc_combined_kin = rospy.ServiceProxy(cli_topic, CalculateCombinedKin)
        if pub_program_topic:
            self.pub_combined_program = rospy.Publisher(pub_program_topic, CombinedProgram, queue_size=1)

    def call(self, tf, tb, odot, wp, pp, gamma, waiti_s=0, waitf_s=0):
        sleep(waiti_s)
        response = self.cli_calc_combined_kin(
            theta_front=tf,
            theta_back=tb,
            omega_dot_max=odot,
            path_to_pick=VehiclePath(waypoints=wp),
            pick_point=pp,
            gamma=gamma
        )
        if self.pub_combined_program:
            self.pub_combined_program.publish(response.program)
            sleep(waitf_s)
        return response
