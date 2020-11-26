#include "ros/ros.h"

#include "srr_kinematics/TractionControlContainer.hpp"

static const std::string NODE_NAME = "srr_kinematics_traction_control";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;

    SRR::TractionControlContainer container(0, 0, 0, 0);

    ROS_INFO("Traction Controller is ready.");
    ros::spin();

    return 0;
}
