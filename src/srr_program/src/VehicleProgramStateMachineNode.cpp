#include "ros/ros.h"

#include <srr_program/VehicleProgramStateMachine.hpp>

static const std::string NODE_NAME = "srr_program_psm_vehicle";
static const std::string PROGRAM_TOPIC = "/srr_integrated/program";
static const double NODE_SLEEP = 0.05;

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    SRR::VehicleProgramStateMachine psm(
        PROGRAM_TOPIC,
        NODE_SLEEP
    );

    ROS_INFO("Vehicle program state machine is ready.");
	psm.run();

    return 0;
}
