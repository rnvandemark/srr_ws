#!/usr/bin/env python
from argparse import ArgumentParser
from math import radians as rads

from geometry_msgs.msg import Pose2D

from kin_callers import CombinedKinCaller

NODE_NAME = "srr_vehicle_test_combined_kin"
CLI_CALC_COMBINED_KIN = "/combined_integrated/calc_combined_kin"
PUB_PROGRAM_TOPIC = "/combined_integrated/program"

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
    parser.add_argument(
        "-x", "--ppx",
        help="The x position of the sample / pick point",
        type=float,
        default=0
    )
    parser.add_argument(
        "-y", "--ppy",
        help="The y position of the sample / pick point",
        type=float,
        default=0
    )
    parser.add_argument(
        "-t", "--ppt",
        help="The orientation theta of the sample / pick point",
        type=float,
        default=0
    )
    parser.add_argument(
        "-g", "--gamma",
        help="The orientation for the pick",
        type=float,
        default=180
    )

    args = parser.parse_args()
    if args.p2d is None:
        print "Error: nothing to do with no waypoints"
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

	pick_point = Pose2D(x=args.ppx, y=args.ppx, theta=rads(args.ppt))

    caller = CombinedKinCaller(NODE_NAME, CLI_CALC_COMBINED_KIN, PUB_PROGRAM_TOPIC)
    response = caller.call(rads(args.tf), rads(args.tb), rads(args.odot), waypoints, pick_point, rads(args.gamma), waiti_s=0.25, waitf_s=0.25);

    print "CALCULATION RESPONSE:"
    print "  {0}".format(str(response).replace("\n", "\n  "))

if __name__ == "__main__":
    main()
