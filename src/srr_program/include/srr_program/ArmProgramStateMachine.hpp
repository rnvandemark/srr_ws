#ifndef __SRR_ARM_PROGRAM_STATE_MACHINE_HPP__
#define __SRR_ARM_PROGRAM_STATE_MACHINE_HPP__

#include <std_msgs/Float64.h>
#include <srr_program/AbstractProgramStateMachine.hpp>
#include <srr_msgs/ArmJointPose.h>

namespace SRR {

using PSM_A_t = AbstractProgramStateMachine<srr_msgs::ArmJointPose>;

class ArmProgramStateMachine : public PSM_A_t {
public:
    enum ArmMove {NONE = 1, NOT_STARTED, STARTED};

protected:
    // The current move being done
    ArmMove curr_move;

    // Publishers to all of the necessary joint controllers
    ros::Publisher pub_base;
    ros::Publisher pub_shoulder;
    ros::Publisher pub_elbow;
    ros::Publisher pub_wrist;

    // Assume inv kin only returned valid solutions
    virtual bool verify(const srr_msgs::ArmJointPose& msg)
    {
        return true;
    }

    // Init anything for when a new program has started
    virtual void init()
    {
        curr_move = ArmMove::NOT_STARTED;
    }

    // Do a cycle in the main activity loop
    virtual bool cycle(double curr_secs)
    {
        bool rv = false;
        if (state == PSM_A_t::PSMState::ACTIVE)
        {
            if ((curr_duration < 0) || (curr_secs >= curr_start + curr_duration))
            {
                curr_start = curr_secs;
                switch(curr_move)
                {
                    case ArmMove::NOT_STARTED:
                        curr_move = ArmMove::STARTED;
                        curr_duration = 8;
                        publish_controller_values(
                            program.q[0],
                            program.q[1],
                            program.q[2],
                            program.q[3]
                        );
                        break;
                    case ArmMove::STARTED:
						curr_move = ArmMove::NONE;
						publish_controller_values(0, 0, 2.6179939, -2.6179939);
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

    // Utility function to publish values to each steer and wheel joint controller
    void publish_controller_values(double b, double s, double e, double w)
    {
        std_msgs::Float64 msg;
#define POPULATE_AND_PUBLISH(p, v) msg.data = v; p.publish(msg)
        POPULATE_AND_PUBLISH(pub_base,     b);
        POPULATE_AND_PUBLISH(pub_shoulder, s);
        POPULATE_AND_PUBLISH(pub_elbow,    e);
        POPULATE_AND_PUBLISH(pub_wrist,    w);
#undef POPULATE_AND_PUBLISH
    }

public:
    ArmProgramStateMachine(std::string program_topic, double sleep):
        PSM_A_t(program_topic, sleep),
        curr_move(ArmMove::NONE),
        pub_base(create_pub_joint_controller("base")),
        pub_shoulder(create_pub_joint_controller("shoulder")),
        pub_elbow(create_pub_joint_controller("elbow")),
        pub_wrist(create_pub_joint_controller("wrist"))
    {
		publish_controller_values(0, 0, 2.6179939, -2.6179939);
    }

};  // class ArmProgramStateMachine

}   // namespace SRR

#endif  // __SRR_ARM_PROGRAM_STATE_MACHINE_HPP__
