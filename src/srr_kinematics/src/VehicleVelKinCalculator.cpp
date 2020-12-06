#include "ros/ros.h"

#include "srr_kinematics/VehicleVelKinContainer.hpp"

#include <iostream>
#include <string>

static const std::string NODE_NAME = "srr_kinematics_vehicle_vel_kin_calculator";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;
    std::string srr_model_name;
    if (!n.getParam("/srr_model_name", srr_model_name))
    {
        std::cerr << "Error getting SRR model name. Exiting." << std::endl;
        return 1;
    }

    double theta_i, leg_length, conn_length, d, Rw;
    std::string pref = std::string("/") + srr_model_name;
    if (!n.getParam(pref+"/velocity_kinematics/theta_i", theta_i)
        || !n.getParam(pref+"/velocity_kinematics/leg_length", leg_length)
        || !n.getParam(pref+"/velocity_kinematics/conn_length", conn_length)
        || !n.getParam(pref+"/velocity_kinematics/d", d)
        || !n.getParam(pref+"/velocity_kinematics/Rw", Rw))
    {
        std::cerr << "Error getting SRR velocity kinematics parameters. Exiting." << std::endl;
        return 2;
    }

    SRR::VehicleVelKinContainer container(
        theta_i,
        leg_length,
        conn_length,
        d,
        Rw
    );

    ros::ServiceServer srv_calc_vehicle_vel_kin = n.advertiseService(
        pref+"/calc_vehicle_vel_kin",
        &SRR::VehicleVelKinContainer::handle_callback,
        &container
    );

    ROS_INFO("Vehicle velocity kinematics calculator is ready.");
    ros::spin();

    return 0;
}
