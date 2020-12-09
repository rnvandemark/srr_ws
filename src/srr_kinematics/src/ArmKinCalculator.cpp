#include "ros/ros.h"

#include "srr_kinematics/ArmKinContainer.hpp"

#include <iostream>

static const std::string NODE_NAME = "srr_kinematics_arm_kin_calculator";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;

    double d1, d2, a2, d3, d4, d5;
    std::string pref = "/srr_integrated/dh_param";
    if (!n.getParam(pref+"/d1", d1) || !n.getParam(pref+"/d2", d2) || !n.getParam(pref+"/a2", a2)
        || !n.getParam(pref+"/d3", d3) || !n.getParam(pref+"/d4", d4) || !n.getParam(pref+"/d5", d5))
    {
        std::cerr << "Error getting one or more DH parameters." << std::endl;
        return 1;
    }

    SRR::ArmKinContainer container = SRR::ArmKinContainer()
		.add_param("d1", d1)
		.add_param("d2", d2)
		.add_param("a2", a2)
		.add_param("d3", d3)
		.add_param("d4", d4)
		.add_param("d5", d5)
	;

    ros::ServiceServer srv_calc_arm_fwd_kin = n.advertiseService(
        "srr_integrated/calc_arm_fwd_kin",
        &SRR::ArmKinContainer::handle_fwd_kin_callback,
        &container
    );

    ros::ServiceServer srv_calc_arm_inv_kin = n.advertiseService(
        "srr_integrated/calc_arm_inv_kin",
        &SRR::ArmKinContainer::handle_inv_kin_callback,
        &container
    );

    ROS_INFO("Arm kinematics calculator is ready.");
    ros::spin();

    return 0;
}
