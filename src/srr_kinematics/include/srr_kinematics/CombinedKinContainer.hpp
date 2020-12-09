#ifndef __SRR_COMBINED_KIN_CONTAINER_HPP__
#define __SRR_COMBINED_KIN_CONTAINER_HPP__

#include "ros/ros.h"

#include "srr_msgs/CalculateCombinedKin.h"

#include <string>
#include <unordered_map>

namespace SRR {

class CombinedKinContainer {
protected:
    ros::ServiceClient cli_vehicle_vel_kin;
    ros::ServiceClient cli_arm_inv_kin;

    double arm_mount_elevation;
    double arm_base_height_off_vehicle;
    double arm_base_min_height_off_ground;
    double vehicle_front_distance_from_pick;
    double leg_length;
    double conn_length;

public:
    CombinedKinContainer(
        ros::ServiceClient& _cli_vehicle_vel_kin,
        ros::ServiceClient& _cli_arm_inv_kin,
        double _arm_mount_elevation,
        double _arm_base_height_off_vehicle,
        double _arm_base_min_height_off_ground,
        double _vehicle_front_distance_from_pick,
        double _leg_length,
        double _conn_length);

    bool handle_combined_kin_callback(srr_msgs::CalculateCombinedKin::Request&  req,
                                      srr_msgs::CalculateCombinedKin::Response& res);
};  // class CombinedKinContainer

}   // namespace SRR

#endif  // __SRR_COMBINED_KIN_CONTAINER_HPP__
