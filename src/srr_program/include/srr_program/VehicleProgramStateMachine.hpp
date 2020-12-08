#ifndef __SRR_VEHICLE_PROGRAM_STATE_MACHINE_HPP__
#define __SRR_VEHICLE_PROGRAM_STATE_MACHINE_HPP__

#include <std_msgs/Float64.h>
#include <srr_program/AbstractProgramStateMachine.hpp>
#include <srr_msgs/VehicleVelKinSolution.h>

namespace SRR {

using PSM_V_t = AbstractProgramStateMachine<srr_msgs::VehicleVelKinSolution>;

class VehicleProgramStateMachine : public PSM_V_t {
public:
    enum VehicleMove {NONE = 1, NOT_STARTED, FIRST_TURN, LINEAR_MOVE, SECOND_TURN};

protected:
    // The current segment in the vehicle's kinematic solution
    int curr_segment;

    // The current move being done
    VehicleMove curr_move;

    // Publishers to all of the necessary joint controllers
    ros::Publisher pub_leg_front_left;
    ros::Publisher pub_leg_front_right;
    ros::Publisher pub_leg_back_left;
    ros::Publisher pub_leg_back_right;
    ros::Publisher pub_wheel_steer_left;
    ros::Publisher pub_wheel_steer_right;
    ros::Publisher pub_wheel_rotation_front_left;
    ros::Publisher pub_wheel_rotation_front_right;
    ros::Publisher pub_wheel_rotation_back_left;
    ros::Publisher pub_wheel_rotation_back_right;

    // Verify that the program is valid
    virtual bool verify(const srr_msgs::VehicleVelKinSolution& msg)
    {
        return msg.return_code == srr_msgs::VehicleVelKinSolution::SUCCESS;
    }

    // Init anything for when a new program has started
    virtual void init()
    {
        curr_move = VehicleMove::NOT_STARTED;
        curr_segment = 0;
    }

    // Do a cycle in the main activity loop
    virtual bool cycle(double curr_secs)
    {
        bool rv = false;
        if ((curr_duration < 0) || (curr_secs >= curr_start + curr_duration))
        {
            curr_start = curr_secs;
            switch(curr_move)
            {
                case VehicleMove::NOT_STARTED:
                    curr_move = VehicleMove::FIRST_TURN;
                    curr_duration = program.segments[curr_segment].ft_duration;
                    break;
                case VehicleMove::FIRST_TURN:
                    curr_move = VehicleMove::LINEAR_MOVE;
                    curr_duration = program.segments[curr_segment].m_duration;
                    break;
                case VehicleMove::LINEAR_MOVE:
                    curr_move = VehicleMove::SECOND_TURN;
                    curr_duration = program.segments[curr_segment].st_duration;
                    break;
                case VehicleMove::SECOND_TURN:
                    if (++curr_segment >= program.segments.size())
                    {
                        curr_move = VehicleMove::NONE;
                        rv = true;
                    }
                    else
                    {
                        curr_move = VehicleMove::NOT_STARTED;
                        curr_duration = -1;
                    }
                    break;
                default:
                    rv = true;
                    break;
            }
        }
        return rv;
    }

    // Utility function to create a publisher to the given joint's controller
    ros::Publisher create_pub_joint_controller(std::string joint_name, int queue_depth=1)
    {
        return nh.advertise<std_msgs::Float64>(
            std::string("/srr_integrated/") + joint_name + "_joint_controller/command",
            queue_depth
        );
    }

public:
    VehicleProgramStateMachine(std::string program_topic, double sleep):
        PSM_V_t(program_topic, sleep),
        curr_segment(-1),
        curr_move(VehicleMove::NONE),
        pub_leg_front_left(create_pub_joint_controller("front_left")),
        pub_leg_front_right(create_pub_joint_controller("front_right")),
        pub_leg_back_left(create_pub_joint_controller("rear_left")),
        pub_leg_back_right(create_pub_joint_controller("rear_right")),
        pub_wheel_steer_left(create_pub_joint_controller("front_left_steer")),
        pub_wheel_steer_right(create_pub_joint_controller("front_right_steer")),
        pub_wheel_rotation_front_left(create_pub_joint_controller("front_left_wheel")),
        pub_wheel_rotation_front_right(create_pub_joint_controller("front_right_wheel")),
        pub_wheel_rotation_back_left(create_pub_joint_controller("rear_left_wheel")),
        pub_wheel_rotation_back_right(create_pub_joint_controller("rear_right_wheel"))
    {
    }

};  // class VehicleProgramStateMachine

}   // namespace SRR

#endif  // __SRR_VEHICLE_PROGRAM_STATE_MACHINE_HPP__
