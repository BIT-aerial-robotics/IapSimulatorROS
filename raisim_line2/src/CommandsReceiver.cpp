// CommandsReceiver.cpp
// declarations to receive assembly robot commands
// Du Jianrui   2023.2.6

#include "../include/CommandsReceiver.h"

#include "../include/DataPacket.hpp"
#include "../include/StatesSender.h"

CommandsReceiver::CommandsReceiver(const ros::NodeHandle& _nodeHandle, raisim::ArticulatedSystem* _robot)
  : nodeHandle_(_nodeHandle), robot_(_robot)
{
    initializeSubscriber();
}

void CommandsReceiver::receiveCommandsAndApply(AvaRobotCommand& _commands, const AvaRobotState& _states)
{
    _commands.thrust1 = thrustCallbackNo1_.get().data;
    _commands.thrust2 = thrustCallbackNo2_.get().data;
    _commands.moment1.x() = momentCallbackNo1_.get().x;
    _commands.moment1.y() = momentCallbackNo1_.get().y;
    _commands.moment1.z() = momentCallbackNo1_.get().z;
    _commands.moment2.x() = momentCallbackNo2_.get().x;
    _commands.moment2.y() = momentCallbackNo2_.get().y;
    _commands.moment2.z() = momentCallbackNo2_.get().z;

    Eigen::Vector3d force0, force1, force2; // active force in their own frame
    Eigen::Vector3d moment0; // active composite moment on the base in base frame
    Eigen::Vector3d moment_reaction; // reaction moment caused by joint moments
    Eigen::Vector3d relative_position1( 1.0, 0.0, 0.0); // child drone1 relative position in base frame
    Eigen::Vector3d relative_position2(-1.0, 0.0, 0.0); // child drone2 relative position in base frame
    Eigen::Matrix3d matrix1_origin = getRotationMatrix(Eigen::Vector3d(0, 0, 0)); // original rotation from joint1 to base frame
    Eigen::Matrix3d matrix2_origin = getRotationMatrix(Eigen::Vector3d(0, 0, 0)); // original rotation from joint2 to base frame

    Eigen::Vector3d e3(0, 0, 1);
    force1 = -_commands.thrust1 * e3;
    force2 = -_commands.thrust2 * e3;
    force0 = matrix1_origin * _states.rotation1_0 * force1 +
             matrix2_origin * _states.rotation2_0 * force2;
    moment0 = relative_position1.cross(matrix1_origin * _states.rotation1_0 * force1) +
              relative_position2.cross(matrix2_origin * _states.rotation2_0 * force2);
    moment_reaction = -(matrix1_origin * _states.rotation1_0 * _commands.moment1 +
                        matrix2_origin * _states.rotation2_0 * _commands.moment2);
    Eigen::Vector3d moment_total = moment0 - moment_reaction;
    moment_total.x() -= _states.bodyRate0.x() * 0.02;

    Eigen::Vector3d force0_w = convertFromFluToFrd(_states.rotation0_w * force0);
    Eigen::Vector3d moment_total_w = convertFromFluToFrd(_states.rotation0_w * moment_total);
    Eigen::Vector3d moment1_0 = convertFromFluToFrd(_states.rotation1_0 * _commands.moment1);
    Eigen::Vector3d moment2_0 = convertFromFluToFrd(_states.rotation2_0 * _commands.moment2);
    robot_->setGeneralizedForce({force0_w.x(),       force0_w.y(),       force0_w.z(),
                                 moment_total_w.x(), moment_total_w.y(), moment_total_w.z(),
                                 moment1_0.x(),      moment1_0.y(),      moment1_0.z(),
                                 moment2_0.x(),      moment2_0.y(),      moment2_0.z()});
}

void CommandsReceiver::initializeSubscriber()
{
    thrustSubscriberNo1_ = nodeHandle_.subscribe<std_msgs::Float64>
                           (robot_->getName() + "/thrust_1", 1, &Thrust::callback, &thrustCallbackNo1_);
    thrustSubscriberNo2_ = nodeHandle_.subscribe<std_msgs::Float64>
                           (robot_->getName() + "/thrust_2", 1, &Thrust::callback, &thrustCallbackNo2_);
    momentSubscriberNo1_ = nodeHandle_.subscribe<geometry_msgs::Point>
                           (robot_->getName() + "/moment_1", 1, &Moment::callback, &momentCallbackNo1_);
    momentSubscriberNo2_ = nodeHandle_.subscribe<geometry_msgs::Point>
                           (robot_->getName() + "/moment_2", 1, &Moment::callback, &momentCallbackNo2_);
}
