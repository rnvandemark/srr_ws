#!/usr/bin/env python
from argparse import ArgumentParser
from math import radians as rads

from geometry_msgs.msg import Pose2D

from kin_callers import VehicleVelKinCaller

NODE_NAME = "srr_vehicle_test_vehicle_vel_kin"
CLI_CALC_VEHICLE_VEL_KIN = "/srr_integrated/calc_vehicle_vel_kin"
SUB_VEHICLE_VEL_KIN_DEBUG = "/srr_integrated/vehicle_vel_kin_debug"
PUB_VEHICLE_PROGRAM = "/srr_integrated/program"

def main():
    parser = ArgumentParser()
    parser.add_argument(
        "-p", "--p2d",
        help="Add a 2D pose in meters and degrees as '[x,y,theta]'. Example: '[4.5,-2.0,45]'",
        type=str,
        action="append"
    )
    parser.add_argument(
        "-f", "--tf",
        help="The angle theta of the vehicle's front legs in degrees",
        type=float,
        default=0
    )
    parser.add_argument(
        "-b", "--tb",
        help="The angle theta of the vehicle's back legs in degrees",
        type=float,
        default=0
    )
    parser.add_argument(
        "-o", "--odot",
        help="The peak velocity of the vehicle's wheels in degrees/sec",
        type=float,
        default=180.0
    )

    args = parser.parse_args()
    if args.p2d is None:
        print "Error: nothing to do with no waypoints"
        return
    if len(args.p2d) < 2:
        print "Error: need at least two waypoints to do anything"
        return

    waypoints = []
    for p2d_str in args.p2d:
        p2d_str = p2d_str[p2d_str.index("[")+1:p2d_str.index("]")]
        p2d_parts = [p.strip() for p in p2d_str.split(",")]
        if len(p2d_parts) != 3:
            print "Error: could not get pose components from {0}".format(p2d_str)
            return
        waypoints.append(Pose2D(
            x=float(p2d_parts[0]),
            y=float(p2d_parts[1]),
            theta=rads(float(p2d_parts[2]))
        ))

    caller = VehicleVelKinCaller(NODE_NAME, CLI_CALC_VEHICLE_VEL_KIN, SUB_VEHICLE_VEL_KIN_DEBUG, pub_program_topic=PUB_VEHICLE_PROGRAM)
    response, msg_debug = caller.call(rads(args.tf), rads(args.tb), rads(args.odot), waypoints, waiti_s=0.25, waitf_s=0.25);

    print "CALCULATION RESPONSE:"
    print "  {0}".format(str(response).replace("\n", "\n  "))
    print
    print "DEBUG MESSAGE:"
    print "  {0}".format(str(msg_debug).replace("\n", "\n  "))

if __name__ == "__main__":
    main()
