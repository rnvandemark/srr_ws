#include "srr_kinematics/CombinedKinContainer.hpp"

#include "srr_msgs/CalculateVehicleVelKin.h"
#include "srr_msgs/CalculateArmInvKin.h"

#include <iostream>

static const std::string NODE_NAME = "srr_kinematics_combined_kin_calculator";

static const std::string TOPIC_VEHICLE_VEL_KIN = "/srr_integrated/calc_vehicle_vel_kin";
static const std::string TOPIC_ARM_INV_KIN = "/sra_integrated/calc_arm_inv_kin";
static const std::string TOPIC_COMBINED_KIN = "/combined_integrated/calc_combined_kin";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;

    double mount_elev, height_off, min_height, front_dist, leg_length, conn_length;
    std::string pref = "/combined_integrated";
    if (!n.getParam(pref+"/arm_mount_elevation", mount_elev)
        || !n.getParam(pref+"/arm_base_height_off_vehicle", height_off)
        || !n.getParam(pref+"/arm_base_min_height_off_ground", min_height)
        || !n.getParam(pref+"/vehicle_front_distance_from_pick", front_dist)
        || !n.getParam("/srr_integrated/velocity_kinematics/leg_length", leg_length)
        || !n.getParam("/srr_integrated/velocity_kinematics/conn_length", conn_length))
    {
        std::cerr << "Error getting one or more ROS parameters." << std::endl;
        return 1;
    }

    ros::ServiceClient cli_vehicle_vel_kin = n.serviceClient<srr_msgs::CalculateVehicleVelKin>(TOPIC_VEHICLE_VEL_KIN);
    ros::ServiceClient cli_arm_inv_kin = n.serviceClient<srr_msgs::CalculateArmInvKin>(TOPIC_ARM_INV_KIN);

    SRR::CombinedKinContainer container(
        cli_vehicle_vel_kin,
        cli_arm_inv_kin,
        mount_elev,
        height_off,
        min_height,
        front_dist,
        leg_length,
        conn_length
    );

    ros::ServiceServer srv_calc_combined_kin = n.advertiseService(
        TOPIC_COMBINED_KIN,
        &SRR::CombinedKinContainer::handle_combined_kin_callback,
        &container
    );

    ROS_INFO("Combined kinematics calculator is ready.");
    ros::spin();

    return 0;
}
