#include "ros/ros.h"

#include "srr_kinematics/TractionControlContainer.hpp"

static const std::string NODE_NAME = "srr_kinematics_traction_control";

static const int SPIN_RATE_HZ = 50;
static const double NUM_SECONDS_IN_WINDOW = 1.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle n;

    SRR::TractionControlContainer container(
        "srr_integrated",
        0.0943864,
        10,
        0.21844,
        0.04445,
        "FL_Joint",
        "FR_Joint",
        "RL_Joint",
        "RR_Joint",
        +0.24511,
        -0.24511,
        +0.27,
        -0.27,
        "FL_Rev_Joint",
        "FR_Rev_Joint",
        static_cast<int>(SPIN_RATE_HZ * NUM_SECONDS_IN_WINDOW)
    );

    ros::Subscriber sub_vehicle_joint_states = n.subscribe(
        "/srr_integrated/joint_states",
        1,
        &SRR::TractionControlContainer::handle_joint_state_callback,
        &container
    );

    ros::Subscriber sub_model_states = n.subscribe(
        "/gazebo/model_states",
        1,
        &SRR::TractionControlContainer::handle_model_states_callback,
        &container
    );

    ros::Subscriber sub_vehicle_imu = n.subscribe(
        "/srr_integrated/imu_data",
        1,
        &SRR::TractionControlContainer::handle_imu_callback,
        &container
    );

    srr_msgs::TractionControlDebug msg_debug;
    ros::Publisher pub_tractl_debug = n.advertise<srr_msgs::TractionControlDebug>(
        "/srr_integrated/tractl_debug",
        10
    );

    ros::Rate spin_rate(SPIN_RATE_HZ);
    SRR::TractionControlContainer::LegAbstractMap<tf::Vector3> linear_velocity_wheel_wrt_wheel;
    SRR::TractionControlContainer::LegAbstractMap<double> contact_angles;
    SRR::TractionControlContainer::LegAbstractMap<double> commanded_wheel_rates;
    ROS_INFO("Traction Controller is ready.");

    while (ros::ok())
    {
        if (container.calculate_wheel_rates(linear_velocity_wheel_wrt_wheel, contact_angles, commanded_wheel_rates, msg_debug))
        {
            pub_tractl_debug.publish(msg_debug);
        }
        spin_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
