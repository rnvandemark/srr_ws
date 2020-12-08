#ifndef __SRR_ABSTRACT_PROGRAM_STATE_MACHINE_HPP__
#define __SRR_ABSTRACT_PROGRAM_STATE_MACHINE_HPP__

#include "ros/ros.h"

#include <string>

namespace SRR {

template <typename ProgramMsgType>
class AbstractProgramStateMachine {
public:
    enum PSMState {UNKNOWN = 0, INACTIVE, ACTIVE};

protected:
    // The subscriber for program messages
    ros::NodeHandle nh;

    // The subscriber for program messages
    ros::Subscriber sub_program;

    // The amount of time to sleep between cycles of the main activity loop
    ros::Duration sleep_duration;

    // The current state of the program state machine
    PSMState state;

    // The program currently being ran
    ProgramMsgType program;

    // The time that the current movement started
    double curr_start;

    // The amount of time esimated to be needed to do the current movement
    double curr_duration;

    // Verify that the program is valid
    virtual bool verify(const ProgramMsgType& msg) = 0;

    // Init anything for when a new program has started
    virtual void init() = 0;

    // Do a cycle in the main activity loop
    virtual bool cycle(double curr_secs) = 0;

public:
    AbstractProgramStateMachine(std::string program_topic, double sleep):
        nh(),
        sub_program(nh.subscribe(
            program_topic,
            1,
            &SRR::AbstractProgramStateMachine<ProgramMsgType>::handle_program_callback,
            this
        )),
        sleep_duration(sleep),
        state(PSMState::INACTIVE)
    {
    }
    virtual ~AbstractProgramStateMachine()
    {
    }

    // Contains the main activity loop
    void run()
    {
        while (ros::ok())
        {
            if (cycle(ros::Time::now().toSec()))
            {
                state = PSMState::INACTIVE;
            }
            sleep_duration.sleep();
            ros::spinOnce();
        }
    }

    void handle_program_callback(const ProgramMsgType& msg)
    {
        if (verify(msg))
        {
            program = msg;
            curr_start = -1;
            curr_duration = -1;
            state = PSMState::ACTIVE;
            init();
        }
    }

};  // class AbstractProgramStateMachine

}   // namespace SRR

#endif  // __SRR_ABSTRACT_PROGRAM_STATE_MACHINE_HPP__
