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

public:
    void setOriginalRotation(const Eigen::Vector3d& _originalEulerAngle1,
                             const Eigen::Vector3d& _originalEulerAngle2,
                             const Eigen::Vector3d& _originalEulerAngle3)
    {
        originalEulerAngle1_ = _originalEulerAngle1;
        originalEulerAngle2_ = _originalEulerAngle2;
        originalEulerAngle3_ = _originalEulerAngle3;
    }
    void setRelativePosition(const Eigen::Vector3d& _relativePosition1,
                             const Eigen::Vector3d& _relativePosition2,
                             const Eigen::Vector3d& _relativePosition3)
    {
        relativePosition1_ = _relativePosition1;
        relativePosition2_ = _relativePosition2;
        relativePosition3_ = _relativePosition3;
    }

private:
    ros::NodeHandle nodeHandle_;
    raisim::ArticulatedSystem* robot_;

private: // lists subscribers here
    // subscriber handles
    ros::Subscriber thrustSubscriberNo1_;
    ros::Subscriber thrustSubscriberNo2_;
    ros::Subscriber thrustSubscriberNo3_;
    ros::Subscriber momentSubscriberNo1_;
    ros::Subscriber momentSubscriberNo2_;
    ros::Subscriber momentSubscriberNo3_;
    // subscriber callback instances
    Thrust thrustCallbackNo1_;
    Thrust thrustCallbackNo2_;
    Thrust thrustCallbackNo3_;
    Moment momentCallbackNo1_;
    Moment momentCallbackNo2_;
    Moment momentCallbackNo3_;

private:
    Eigen::Vector3d originalEulerAngle1_; // original Euler angle from joint1 to base frame
    Eigen::Vector3d originalEulerAngle2_; // original Euler angle from joint2 to base frame
    Eigen::Vector3d originalEulerAngle3_; // original Euler angle from joint3 to base frame
    Eigen::Vector3d relativePosition1_; // child drone1 relative position in base frame
    Eigen::Vector3d relativePosition2_; // child drone2 relative position in base frame
    Eigen::Vector3d relativePosition3_; // child drone3 relative position in base frame

private:
    void initializeSubscriber();
};

