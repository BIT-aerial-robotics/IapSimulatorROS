// CommandsReceiver.h
// declarations to receive assembly robot commands
// Du Jianrui   2023.9.23

#include "Common.hpp"
#include "Callback.h"
#include "ros/ros.h"
#include <zmq.hpp>

class CommandsReceiver
{
public:
    CommandsReceiver(const ros::NodeHandle& _nodeHandle, raisim::ArticulatedSystem* _robot);
    CommandsReceiver(const CommandsReceiver& _receiver) = delete;
    CommandsReceiver operator=(const CommandsReceiver& _anotherReceiver) = delete;

    void receiveCommandsAndApply(AvaRobotCommand& _commands, const AvaRobotState& _states);

    ~CommandsReceiver()
    {
        robot_ = nullptr;
    }

private:
    ros::NodeHandle nodeHandle_;
    raisim::ArticulatedSystem* robot_;

private: // lists subscribers here
    // subscriber handles
    ros::Subscriber thrustSubscriberNo1_;
    ros::Subscriber thrustSubscriberNo2_;
    ros::Subscriber momentSubscriberNo1_;
    ros::Subscriber momentSubscriberNo2_;
    // subscriber callback instances
    Thrust thrustCallbackNo1_;
    Thrust thrustCallbackNo2_;
    Moment momentCallbackNo1_;
    Moment momentCallbackNo2_;

private:
    void initializeSubscriber();
};

