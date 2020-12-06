#!/usr/bin/env python

import rospy

from srr_msgs.srv import CalculateVehicleVelKin
from srr_msgs.msg import VehiclePath
from geometry_msgs.msg import Pose2D

if __name__ == "__main__":
    rospy.init_node("srr_vehicle_test_vehicle_vel_kin")

    srv_calc_vehicle_vel_kin = rospy.ServiceProxy("/srr_integrated/calc_vehicle_vel_kin", CalculateVehicleVelKin)
    msg_vehicle_path = VehiclePath(waypoints=[Pose2D(x=10.0, y=10.0, theta=0.1), Pose2D(x=-20.0, y=-10.0, theta=2.0)])
    response = srv_calc_vehicle_vel_kin(theta_front=0.0, theta_back=0.0, omega_dot_max=6.0, path=msg_vehicle_path)
    print response
