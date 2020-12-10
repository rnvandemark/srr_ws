#include "ros/ros.h"

#include <srr_program/CombinedProgramStateMachine.hpp>

static const std::string NODE_NAME = "srr_program_psm_combined";
static const std::string PROGRAM_TOPIC = "/combined_integrated/program";
static const double NODE_SLEEP = 0.05;

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    SRR::CombinedProgramStateMachine psm(
        PROGRAM_TOPIC,
        NODE_SLEEP
    );

    ROS_INFO("Combined program state machine is ready.");
	psm.run();

    return 0;
}
