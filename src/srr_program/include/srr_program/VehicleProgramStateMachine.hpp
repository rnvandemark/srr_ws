#ifndef __SRR_VEHICLE_PROGRAM_STATE_MACHINE_HPP__
#define __SRR_VEHICLE_PROGRAM_STATE_MACHINE_HPP__

#include <std_msgs/Float64.h>
#include <srr_program/AbstractProgramStateMachine.hpp>
#include <srr_msgs/VehicleVelKinSolution.h>
#include <srr_msgs/VehicleVelKinSegment.h>

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
        if (state == PSM_V_t::PSMState::ACTIVE)
        {
            if ((curr_duration < 0) || (curr_secs >= curr_start + curr_duration))
            {
                curr_start = curr_secs;
                srr_msgs::VehicleVelKinSegment seg = program.segments[curr_segment];
                switch(curr_move)
                {
                    case VehicleMove::NOT_STARTED:
                        curr_move = VehicleMove::FIRST_TURN;
                        curr_duration = seg.ft_duration;
                        publish_controller_values(
                            seg.ft_lw_direction,
                            seg.ft_rw_direction,
                            seg.ft_lw_angular_velocity,
                            seg.ft_rw_angular_velocity,
                            seg.ft_lw_angular_velocity,
                            seg.ft_rw_angular_velocity
                        );
                        break;
                    case VehicleMove::FIRST_TURN:
                        curr_move = VehicleMove::LINEAR_MOVE;
                        curr_duration = seg.m_duration;
                        publish_controller_values(
                            0,
                            0,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities
                        );
                        break;
                    case VehicleMove::LINEAR_MOVE:
                        curr_move = VehicleMove::SECOND_TURN;
                        curr_duration = seg.st_duration;
                        publish_controller_values(
                            seg.st_lw_direction,
                            seg.st_rw_direction,
                            seg.st_lw_angular_velocity,
                            seg.st_rw_angular_velocity,
                            seg.st_lw_angular_velocity,
                            seg.st_rw_angular_velocity
                        );
                        break;
                    case VehicleMove::SECOND_TURN:
                        if (++curr_segment >= program.segments.size())
                        {
                            curr_move = VehicleMove::NONE;
                            publish_controller_values(0, 0, 0, 0, 0, 0);
                            ROS_INFO("Done program.");
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

    // Utility function to publish values to each steer and wheel joint controller
    void publish_controller_values(double wsl, double wsr, double wrfl, double wrfr, double wrbl, double wrbr)
    {
        std_msgs::Float64 msg;
#define POPULATE_AND_PUBLISH(p, v) msg.data = v; p.publish(msg)
        POPULATE_AND_PUBLISH(pub_wheel_steer_left,           wsl);
        POPULATE_AND_PUBLISH(pub_wheel_steer_right,          wsr);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_front_left,  wrfl);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_front_right, wrfr);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_back_left,   wrbl);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_back_right,  wrbr);
#undef POPULATE_AND_PUBLISH
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
