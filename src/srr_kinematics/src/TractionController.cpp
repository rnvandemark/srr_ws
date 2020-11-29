#include "ros/ros.h"

#include "srr_kinematics/TractionControlContainer.hpp"

#include <iostream>

static const std::string NODE_NAME = "srr_kinematics_traction_control";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;

    SRR::TractionControlContainer container(
        0.0943864,
        0.1,
        0.21844,
        0.04445,
        "FL_Joint",
        "FR_Joint",
        "RL_Joint",
        "RR_Joint",
        0.24511,
        0.24511,
        0.27,
        0.27
    );

    ros::Subscriber sub_vehicle_joint_states = n.subscribe(
        "/srr_integrated/joint_states",
        1,
        &SRR::TractionControlContainer::handle_joint_state_callback,
        &container
    );

    ros::Subscriber sub_vehicle_imu = n.subscribe(
        "/srr_integrated/imu_data",
        1,
        &SRR::TractionControlContainer::handle_imu_callback,
        &container
    );

    ros::Rate spin_rate(5);
    SRR::TractionControlContainer::LegAbstractMap<double> contact_angles;
    ROS_INFO("Traction Controller is ready.");

    while (ros::ok())
    {
        bool passed = container.publish_wheel_rates(contact_angles);
        std::cout
            << ", Results: {"
            << contact_angles[SRR::TractionControlContainer::LegPivotEnum::LEFT_NEAR] << ", "
            << contact_angles[SRR::TractionControlContainer::LegPivotEnum::RIGHT_NEAR] << ", "
            << contact_angles[SRR::TractionControlContainer::LegPivotEnum::LEFT_FAR] << ", "
            << contact_angles[SRR::TractionControlContainer::LegPivotEnum::RIGHT_FAR]
            << "}"
            << (const char*)(passed ? ", PASSED!!!" : "")
            << std::endl;
        spin_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
