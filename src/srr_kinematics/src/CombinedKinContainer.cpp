#include "srr_kinematics/CombinedKinContainer.hpp"

#include "srr_msgs/CalculateVehicleVelKin.h"
#include "srr_msgs/CalculateArmInvKin.h"
#include "srr_msgs/VehiclePath.h"
#include "srr_msgs/VehicleVelKinSolution.h"
#include "geometry_msgs/Pose2D.h"

#include <cmath>

SRR::CombinedKinContainer::CombinedKinContainer(
        ros::ServiceClient& _cli_vehicle_vel_kin,
        ros::ServiceClient& _cli_arm_inv_kin,
        double _arm_mount_elevation,
        double _arm_base_height_off_vehicle,
        double _arm_base_min_height_off_ground,
        double _vehicle_front_distance_from_pick,
        double _leg_length,
        double _conn_length):
    cli_vehicle_vel_kin(_cli_vehicle_vel_kin),
    cli_arm_inv_kin(_cli_arm_inv_kin),
    arm_mount_elevation(_arm_mount_elevation),
    arm_base_height_off_vehicle(_arm_base_height_off_vehicle),
    arm_base_min_height_off_ground(_arm_base_min_height_off_ground),
    vehicle_front_distance_from_pick(_vehicle_front_distance_from_pick),
    leg_length(_leg_length),
    conn_length(_conn_length)
{
}

bool SRR::CombinedKinContainer::handle_combined_kin_callback(srr_msgs::CalculateCombinedKin::Request&  req,
                                                             srr_msgs::CalculateCombinedKin::Response& res)
{
    geometry_msgs::Pose2D final_pose;
    final_pose.x = req.pick_point.x + (vehicle_front_distance_from_pick * std::cos(req.pick_point.theta));
    final_pose.y = req.pick_point.y - (vehicle_front_distance_from_pick * std::sin(req.pick_point.theta));
    final_pose.theta = req.pick_point.theta;

    srr_msgs::VehiclePath modified_path(req.path_to_pick);
    modified_path.waypoints.push_back(final_pose);

    srr_msgs::CalculateVehicleVelKin calc_vehicle_vel_kin;
    calc_vehicle_vel_kin.request.theta_front = req.theta_front;
    calc_vehicle_vel_kin.request.theta_back = req.theta_back;
    calc_vehicle_vel_kin.request.omega_dot_max = req.omega_dot_max;
    calc_vehicle_vel_kin.request.path = modified_path;

    if (!cli_vehicle_vel_kin.call(calc_vehicle_vel_kin))
    {
        return false;
    }
    else if (calc_vehicle_vel_kin.response.solution.return_code != srr_msgs::VehicleVelKinSolution::SUCCESS)
    {
        std::cerr << "Impossible path requested: " << calc_vehicle_vel_kin.response.solution.return_code << std::endl;
        return true;
    }

    // For here, assume chassis is flat (i.e., theta_front == theta_back)
    double leg_offset_off_ground = (leg_length * std::cos(req.theta_front)) + (conn_length * std::sin(req.theta_front));

    srr_msgs::CalculateArmInvKin calc_arm_inv_kin;
    calc_arm_inv_kin.request.cart_pose.x = vehicle_front_distance_from_pick;
    calc_arm_inv_kin.request.cart_pose.y = 0;
    calc_arm_inv_kin.request.cart_pose.z = -(leg_offset_off_ground + arm_mount_elevation + arm_base_height_off_vehicle);
    calc_arm_inv_kin.request.cart_pose.gamma = req.gamma;

    if (!cli_arm_inv_kin.call(calc_arm_inv_kin))
    {
        return false;
    }
    else if (calc_arm_inv_kin.response.solution.empty())
    {
        std::cerr << "Impossible arm pose requested: " << calc_arm_inv_kin.request.cart_pose << std::endl;
        return true;
    }

    res.solution_vehicle = calc_vehicle_vel_kin.response.solution;
    res.solution_arm = calc_arm_inv_kin.response.solution[0];
}
