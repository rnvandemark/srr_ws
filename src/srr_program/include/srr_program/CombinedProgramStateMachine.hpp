#ifndef __SRR_COMBINED_PROGRAM_STATE_MACHINE_HPP__
#define __SRR_COMBINED_PROGRAM_STATE_MACHINE_HPP__

#include <std_msgs/Float64.h>
#include <srr_program/AbstractProgramStateMachine.hpp>
#include <srr_msgs/CombinedProgram.h>
#include <srr_msgs/VehicleVelKinSolution.h>

namespace SRR {

using PSM_C_t = AbstractProgramStateMachine<srr_msgs::CombinedProgram>;

class CombinedProgramStateMachine : public PSM_C_t {
public:
    enum CombinedMove {NONE = 1, NOT_STARTED, FIRST_TURN, LINEAR_MOVE, SECOND_TURN, ARM_STARTED};

protected:
    // The current segment in the vehicle's kinematic solution
    int curr_segment;

    // The current move being done
    CombinedMove curr_move;

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
    ros::Publisher pub_base;
    ros::Publisher pub_shoulder;
    ros::Publisher pub_elbow;
    ros::Publisher pub_wrist;
	

    // Assume inv kin only returned valid solutions
    virtual bool verify(const srr_msgs::CombinedProgram& msg)
    {
        return msg.solution_vehicle.return_code == srr_msgs::VehicleVelKinSolution::SUCCESS;
    }

    // Init anything for when a new program has started
    virtual void init()
    {
        curr_move = CombinedMove::NOT_STARTED;
        curr_segment = 0;
    }

    // Do a cycle in the main activity loop
    virtual bool cycle(double curr_secs)
    {
        bool rv = false;
        if (state == PSM_C_t::PSMState::ACTIVE)
        {
            if ((curr_duration < 0) || (curr_secs >= curr_start + curr_duration))
            {
                curr_start = curr_secs;
                srr_msgs::VehicleVelKinSegment seg = program.solution_vehicle.segments[curr_segment];
                switch(curr_move)
                {
                    case CombinedMove::NOT_STARTED:
                        curr_move = CombinedMove::FIRST_TURN;
                        curr_duration = seg.ft_duration;
                        publish_vehicle_controller_values(
                            seg.ft_lw_direction,
                            seg.ft_rw_direction,
                            seg.ft_lw_angular_velocity,
                            seg.ft_rw_angular_velocity,
                            seg.ft_lw_angular_velocity,
                            seg.ft_rw_angular_velocity
                        );
                        break;
                    case CombinedMove::FIRST_TURN:
                        curr_move = CombinedMove::LINEAR_MOVE;
                        curr_duration = seg.m_duration;
                        publish_vehicle_controller_values(
                            0,
                            0,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities,
                            seg.m_w_angular_velocities
                        );
                        break;
                    case CombinedMove::LINEAR_MOVE:
                        curr_move = CombinedMove::SECOND_TURN;
                        curr_duration = seg.st_duration;
                        publish_vehicle_controller_values(
                            seg.st_lw_direction,
                            seg.st_rw_direction,
                            seg.st_lw_angular_velocity,
                            seg.st_rw_angular_velocity,
                            seg.st_lw_angular_velocity,
                            seg.st_rw_angular_velocity
                        );
                        break;
                    case CombinedMove::SECOND_TURN:
                        if (++curr_segment >= program.solution_vehicle.segments.size())
                        {
                            curr_move = CombinedMove::ARM_STARTED;
							curr_duration = 8;
							publish_arm_controller_values(
								program.solution_arm.q[0],
								program.solution_arm.q[1],
								program.solution_arm.q[2],
								program.solution_arm.q[3]
							);
							break;
                        }
                        else
                        {
                            ROS_INFO("Next segment.");
                            curr_move = CombinedMove::NOT_STARTED;
                            curr_duration = -1;
                        }
                        break;
                    case CombinedMove::ARM_STARTED:
                        curr_move = CombinedMove::NONE;
						publish_arm_controller_values(0, 0, 2.6179939, -2.6179939);
						ROS_INFO("Done program.");
						rv = true;
						curr_duration = -1;
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
            std::string("/sra/") + joint_name + "_joint_controller/command",
            queue_depth
        );
    }

#define POPULATE_AND_PUBLISH(p, v) msg.data = v; p.publish(msg)
    // Utility function to publish values to each steer and wheel joint controller
    void publish_vehicle_controller_values(double wsl, double wsr, double wrfl, double wrfr, double wrbl, double wrbr)
     {
        std_msgs::Float64 msg;
        POPULATE_AND_PUBLISH(pub_wheel_steer_left,           wsl);
        POPULATE_AND_PUBLISH(pub_wheel_steer_right,          wsr);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_front_left,  wrfl);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_front_right, wrfr);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_back_left,   wrbl);
        POPULATE_AND_PUBLISH(pub_wheel_rotation_back_right,  wrbr);
    }

    // Utility function to publish values to each steer and wheel joint controller
    void publish_arm_controller_values(double b, double s, double e, double w)
    {
        std_msgs::Float64 msg;
        POPULATE_AND_PUBLISH(pub_base,     b);
        POPULATE_AND_PUBLISH(pub_shoulder, s);
        POPULATE_AND_PUBLISH(pub_elbow,    e);
        POPULATE_AND_PUBLISH(pub_wrist,    w);
    }
#undef POPULATE_AND_PUBLISH

public:
    CombinedProgramStateMachine(std::string program_topic, double sleep):
        PSM_C_t(program_topic, sleep),
        curr_move(CombinedMove::NONE),
        pub_leg_front_left(create_pub_joint_controller("front_left")),
        pub_leg_front_right(create_pub_joint_controller("front_right")),
        pub_leg_back_left(create_pub_joint_controller("rear_left")),
        pub_leg_back_right(create_pub_joint_controller("rear_right")),
        pub_wheel_steer_left(create_pub_joint_controller("front_left_steer")),
        pub_wheel_steer_right(create_pub_joint_controller("front_right_steer")),
        pub_wheel_rotation_front_left(create_pub_joint_controller("front_left_wheel")),
        pub_wheel_rotation_front_right(create_pub_joint_controller("front_right_wheel")),
        pub_wheel_rotation_back_left(create_pub_joint_controller("rear_left_wheel")),
        pub_wheel_rotation_back_right(create_pub_joint_controller("rear_right_wheel")),
        pub_base(create_pub_joint_controller("base")),
        pub_shoulder(create_pub_joint_controller("shoulder")),
        pub_elbow(create_pub_joint_controller("elbow")),
        pub_wrist(create_pub_joint_controller("wrist"))
    {
		publish_arm_controller_values(0, 0, 2.6179939, -2.6179939);
    }

};  // class CombinedProgramStateMachine

}   // namespace SRR

#endif  // __SRR_COMBINED_PROGRAM_STATE_MACHINE_HPP__
