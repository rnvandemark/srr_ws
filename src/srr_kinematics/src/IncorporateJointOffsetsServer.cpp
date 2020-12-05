#include "ros/ros.h"

#include "srr_kinematics/JointOffsetsCollection.hpp"

static const std::string NODE_NAME = "srr_kinematics_incorporate_joint_offsets_server";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    if (argc != 2)
    {
        std::cerr << "The desired model's rosparam namespace is a required argument! (Got " << argc << " args.)" << std::endl;
        return 1;
    }
    std::string rosparam_namespace = argv[1];

    ros::NodeHandle n;
    std::string list_joint_name_and_offset;
    std::string list_joint_name_and_direction;
    assert(n.getParam(std::string("/") + rosparam_namespace + "/joint_zero_offsets", list_joint_name_and_offset));
    assert(n.getParam(std::string("/") + rosparam_namespace + "/joint_directions_of_rotation", list_joint_name_and_direction));
    SRR::JointOffsetsCollection collection(list_joint_name_and_offset, list_joint_name_and_direction);

    ros::ServiceServer srv_calc_joint_position = n.advertiseService(
        std::string("/") + rosparam_namespace + "/calculate_position_with_offsets",
        &SRR::JointOffsetsCollection::handle_calculate_position_callback,
        &collection
    );

    ros::ServiceServer srv_get_joint_direction = n.advertiseService(
        std::string("/") + rosparam_namespace + "/get_direction_of_rotation",
        &SRR::JointOffsetsCollection::handle_direction_of_rotation_callback,
        &collection
    );

    ROS_INFO("Joint offsets server is ready.");
    ros::spin();

    return 0;
}
