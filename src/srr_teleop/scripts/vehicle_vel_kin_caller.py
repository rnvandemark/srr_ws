#!/usr/bin/env python
import rospy
from time import sleep

from srr_msgs.srv import CalculateVehicleVelKin
from srr_msgs.msg import VehiclePath, VehicleVelKinDebug

class VehicleVelKinCaller(object):
    cli_calc_vehicle_vel_kin       = None
    sub_calc_vehicle_vel_kin_debug = None
    msg_calc_vehicle_vel_kin_debug = None

    def __init__(self, node_name, cli_topic, sub_debug_topic):
        rospy.init_node(node_name, disable_signals=True)

        self.cli_calc_vehicle_vel_kin = rospy.ServiceProxy(cli_topic, CalculateVehicleVelKin)
        self.sub_calc_vehicle_vel_kin_debug = rospy.Subscriber(
            sub_debug_topic,
            VehicleVelKinDebug,
            self.handle_debug_callback,
            queue_size=1
        )

    def call(self, tf, tb, odot, wp, wait_s=0):
        sleep(wait_s)
        response = self.cli_calc_vehicle_vel_kin(
            theta_front=tf,
            theta_back=tb,
            omega_dot_max=odot,
            path=VehiclePath(waypoints=wp)
        )

        rospy.spin();
        return response, self.msg_calc_vehicle_vel_kin_debug

    def handle_debug_callback(self, msg):
        self.msg_calc_vehicle_vel_kin_debug = msg
        rospy.signal_shutdown("Program finished.")
