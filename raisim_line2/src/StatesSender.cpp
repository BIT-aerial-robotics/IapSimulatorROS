// StatesSender.cpp
// definitions to send assembly robot states
// Du Jianrui   2023.9.23

#include "../include/StatesSender.h"

#include "../include/DataPacket.hpp"

StatesSender::StatesSender(const ros::NodeHandle& _nodeHandle,
                           raisim::ArticulatedSystem* _robot,
                           raisim::World* _world,
                           double _timeStep,
                           const std::string& _localHostIp,
                           const std::string& _dataPort)
  : nodeHandle_(_nodeHandle),
    robot_(_robot),
    timeStep_(_timeStep), ip_(_localHostIp), statesPort_(_dataPort), world_(_world),
    imuNo0_(_timeStep), imuNo1_(_timeStep), imuNo2_(_timeStep),
    imuNoNoise0_(_timeStep), imuNoNoise1_(_timeStep), imuNoNoise2_(_timeStep)
{
    initializeRosPublishers();
    initializeRosSubscribers();
    initializeServer();
}

constexpr double g = 9.805;

void StatesSender::getAndSendStates(AvaRobotState& _states)
{
    auto jointStates = robot_->getGeneralizedCoordinate();
    auto jointVelocities = robot_->getGeneralizedVelocity();
    auto currentTime = world_->getWorldTime();

    // base quaternion to world
    Eigen::Quaterniond quaternion0_w;
    // child drone 1 quaternion to base
    Eigen::Quaterniond quaternion1_0;
    // child drone 1 quaternion to base
    Eigen::Quaterniond quaternion2_0;
    // base Euler Angle to world
    Eigen::Vector3d eulerAngle0;
    // child drone 1 Euler angle to base
    Eigen::Vector3d eulerAngle1_0;
    // child drone 2 Euler angle to base
    Eigen::Vector3d eulerAngle2_0;
    // child drone 1 Euler angle to world
    Eigen::Vector3d eulerAngle1;
    // child drone 2 Euler angle to world
    Eigen::Vector3d eulerAngle2;
    // base frame rotation to world frame
    Eigen::Matrix3d matrix0;
    // original rotation from drone 1 to base frame
    Eigen::Matrix3d matrix1_origin = getRotationMatrix(Eigen::Vector3d(0, 0, 0));
    // original rotation from drone 2 to base frame
    Eigen::Matrix3d matrix2_origin = getRotationMatrix(Eigen::Vector3d(0, 0, 0));
    // child drone 1 frame rotation to joint1
    Eigen::Matrix3d matrix1_0;
    // child drone 2 frame rotation to joint2
    Eigen::Matrix3d matrix2_0;
    // child drone 1 frame rotation to world frame
    Eigen::Matrix3d matrix1;
    // child drone 2 frame rotation to world frame
    Eigen::Matrix3d matrix2;

    // base position in world frame
    Eigen::Vector3d position0;
    // child drone 1 position in world frame
    Eigen::Vector3d position1;
    // child drone 2 position in world frame
    Eigen::Vector3d position2;

    // base angular velocity to world in world frame
    Eigen::Vector3d bodyRate0_w;
    // child drone 1 angular velocity to joint1 in joint1 frame
    Eigen::Vector3d bodyRate1_0;
    // child drone 2 angular velocity to joint2 in joint2 frame
    Eigen::Vector3d bodyRate2_0;
    // base body rate
    Eigen::Vector3d bodyRate0;
    raisim::Vec<3> bodyRate0_r;
    // child drone 1 body rate
    Eigen::Vector3d bodyRate1;
    raisim::Vec<3> bodyRate1_r;
    // child drone 2 body rate
    Eigen::Vector3d bodyRate2;
    raisim::Vec<3> bodyRate2_r;

    // base velocity to world in world frame
    Eigen::Vector3d velocity0_w;
    raisim::Vec<3> velocity0_r;
    // base velocity in base frame
    Eigen::Vector3d velocity0;
    // child drone 1 velocity in world frame
    Eigen::Vector3d velocity1_w;
    raisim::Vec<3> velocity1_r;
    // child drone 1 velocity in child drone 1 frame
    Eigen::Vector3d velocity1;
    // child drone 2 velocity in world frame
    Eigen::Vector3d velocity2_w;
    raisim::Vec<3> velocity2_r;
    // child drone 2 velocity in child drone 2 frame
    Eigen::Vector3d velocity2;

    // linear accelerations
    Eigen::Vector3d linearAcceleration0;
    Eigen::Vector3d linearAcceleration1;
    Eigen::Vector3d linearAcceleration2;

    position0.x() = jointStates[0];
    position0.y() = jointStates[1];
    position0.z() = jointStates[2];
    quaternion0_w.w() = jointStates[3];
    quaternion0_w.x() = jointStates[4];
    quaternion0_w.y() = jointStates[5];
    quaternion0_w.z() = jointStates[6];
    quaternion1_0.w() = jointStates[7];
    quaternion1_0.x() = jointStates[8];
    quaternion1_0.y() = jointStates[9];
    quaternion1_0.z() = jointStates[10];
    quaternion2_0.w() = jointStates[11];
    quaternion2_0.x() = jointStates[12];
    quaternion2_0.y() = jointStates[13];
    quaternion2_0.z() = jointStates[14];
    velocity0_w.x() = jointVelocities[0];
    velocity0_w.y() = jointVelocities[1];
    velocity0_w.z() = jointVelocities[2];
    bodyRate0_w.x() = jointVelocities[3];
    bodyRate0_w.y() = jointVelocities[4];
    bodyRate0_w.z() = jointVelocities[5];
    bodyRate1_0.x() = jointVelocities[6];
    bodyRate1_0.y() = jointVelocities[7];
    bodyRate1_0.z() = jointVelocities[8];
    bodyRate2_0.x() = jointVelocities[9];
    bodyRate2_0.y() = jointVelocities[10];
    bodyRate2_0.z() = jointVelocities[11];

    robot_->getFrameVelocity(robot_->getBodyIdx("base_link"), velocity0_r);
    robot_->getFrameVelocity(std::string("Uav1_joint"), velocity1_r);
    robot_->getFrameVelocity(std::string("Uav2_joint"), velocity2_r);
    robot_->getFrameAngularVelocity(robot_->getBodyIdx("base_link"), bodyRate0_r);
    robot_->getFrameAngularVelocity(std::string("Uav1_joint"), bodyRate1_r);
    robot_->getFrameAngularVelocity(std::string("Uav2_joint"), bodyRate2_r);
    velocity0_w = velocity0_r.e();
    velocity1_w = velocity1_r.e();
    velocity2_w = velocity2_r.e();
    bodyRate0 = bodyRate0_r.e();
    bodyRate1 = bodyRate1_r.e();
    bodyRate2 = bodyRate2_r.e();

    // converts position from NWU to NED
    position0 = convertFromFluToFrd(position0);

    // converts quaternions to Euler angles
    eulerAngle0 = getEulerAngle(quaternion0_w);
    eulerAngle1_0 = getEulerAngle(quaternion1_0);
    eulerAngle2_0 = getEulerAngle(quaternion2_0);
    // converts Euler angles from FLU to FRD
    eulerAngle0 = convertFromFluToFrd(eulerAngle0);
    eulerAngle1_0 = convertFromFluToFrd(eulerAngle1_0);
    eulerAngle2_0 = convertFromFluToFrd(eulerAngle2_0);
    // calculates rotation matrices
    matrix0 = getRotationMatrix(eulerAngle0);
    matrix1_0 = getRotationMatrix(eulerAngle1_0);
    matrix2_0 = getRotationMatrix(eulerAngle2_0);
    matrix1 = matrix0 * matrix1_origin * matrix1_0;
    matrix2 = matrix0 * matrix2_origin * matrix2_0;
    // gets child drones Euler angle from rotation matrices
    eulerAngle1 = getEulerAngle(matrix1);
    eulerAngle2 = getEulerAngle(matrix2);

    // calculates base velocity
    velocity0_w = convertFromFluToFrd(velocity0_w);
    velocity0 = matrix0.transpose() * velocity0_w;

    // calculates base body rate
    bodyRate0_w = convertFromFluToFrd(bodyRate0_w);
    bodyRate0 = matrix0.transpose() * bodyRate0_w;
    // calculates child drones body rates
    bodyRate1 = convertFromFluToFrd(bodyRate1);
    bodyRate1 = matrix1.transpose() * bodyRate1;
    bodyRate2 = convertFromFluToFrd(bodyRate2);
    bodyRate2 = matrix2.transpose() * bodyRate2;

    // calculates child drones velocity
    velocity1_w = convertFromFluToFrd(velocity1_w);
    velocity2_w = convertFromFluToFrd(velocity2_w);
    velocity1 = matrix1.transpose() * velocity1_w;
    velocity2 = matrix2.transpose() * velocity2_w;

    Eigen::Vector3d e3;
    e3 << 0, 0, 1;
    // calculates linear accelerations
    if (velocityLast0_ == Eigen::Vector3d::Zero())
    {
      linearAcceleration0 = Eigen::Vector3d::Zero();
      linearAcceleration1 = Eigen::Vector3d::Zero();
      linearAcceleration2 = Eigen::Vector3d::Zero();
      
      velocityLast0_ = velocity0_w;
      velocityLast1_ = velocity1_w;
      velocityLast2_ = velocity2_w;
      // velocityLast0_ = velocity0;
      // velocityLast1_ = velocity1;
      // velocityLast2_ = velocity2;
    }
    else
    {
      // linearAcceleration0 = (velocity0 - velocityLast0_) / timeStep_ + bodyRate0.cross(velocity0) - g * matrix0.transpose() * e3;
      // linearAcceleration1 = (velocity1 - velocityLast1_) / timeStep_ + bodyRate1.cross(velocity1) - g * matrix1.transpose() * e3;
      // linearAcceleration2 = (velocity2 - velocityLast2_) / timeStep_ + bodyRate2.cross(velocity2) - g * matrix2.transpose() * e3;
      linearAcceleration0 = matrix0.transpose() * ((velocity0_w - velocityLast0_) / timeStep_ - g * e3);
      linearAcceleration1 = matrix1.transpose() * ((velocity1_w - velocityLast1_) / timeStep_ - g * e3);
      linearAcceleration2 = matrix2.transpose() * ((velocity2_w - velocityLast2_) / timeStep_ - g * e3);

      velocityLast0_ = velocity0_w;
      velocityLast1_ = velocity1_w;
      velocityLast2_ = velocity2_w;
      // velocityLast0_ = velocity0;
      // velocityLast1_ = velocity1;
      // velocityLast2_ = velocity2;
    }

    // calculates child drones' positions
    Eigen::Vector3d relative_position1( 1.0, 0.0, 0.0); // child drone1 relative position in base frame
    Eigen::Vector3d relative_position2(-1.0, 0.0, 0.0); // child drone2 relative position in base frame
    position1 = matrix0 * relative_position1 + position0;
    position2 = matrix0 * relative_position2 + position0;

    // packs Euler angles
    DataProtocol::Data eulerAngleData0x(eulerAngle0.x(), 0);
    DataProtocol::Data eulerAngleData0y(eulerAngle0.y(), 1);
    DataProtocol::Data eulerAngleData0z(eulerAngle0.z(), 2);
    DataProtocol::DataPacket eulerAnglePack0;
    eulerAnglePack0.addData(eulerAngleData0x);
    eulerAnglePack0.addData(eulerAngleData0y);
    eulerAnglePack0.addData(eulerAngleData0z);
    DataProtocol::Data eulerAngleData1x(eulerAngle1.x(), 0);
    DataProtocol::Data eulerAngleData1y(eulerAngle1.y(), 1);
    DataProtocol::Data eulerAngleData1z(eulerAngle1.z(), 2);
    DataProtocol::DataPacket eulerAnglePack1;
    eulerAnglePack1.addData(eulerAngleData1x);
    eulerAnglePack1.addData(eulerAngleData1y);
    eulerAnglePack1.addData(eulerAngleData1z);
    DataProtocol::Data eulerAngleData2x(eulerAngle2.x(), 0);
    DataProtocol::Data eulerAngleData2y(eulerAngle2.y(), 1);
    DataProtocol::Data eulerAngleData2z(eulerAngle2.z(), 2);
    DataProtocol::DataPacket eulerAnglePack2;
    eulerAnglePack2.addData(eulerAngleData2x);
    eulerAnglePack2.addData(eulerAngleData2y);
    eulerAnglePack2.addData(eulerAngleData2z);

    // packs body rates
    DataProtocol::Data bodyRateData0x(bodyRate0.x(), 0);
    DataProtocol::Data bodyRateData0y(bodyRate0.y(), 1);
    DataProtocol::Data bodyRateData0z(bodyRate0.z(), 2);
    DataProtocol::DataPacket bodyRatePack0;
    bodyRatePack0.addData(bodyRateData0x);
    bodyRatePack0.addData(bodyRateData0y);
    bodyRatePack0.addData(bodyRateData0z);
    DataProtocol::Data bodyRateData1x(bodyRate1.x(), 0);
    DataProtocol::Data bodyRateData1y(bodyRate1.y(), 1);
    DataProtocol::Data bodyRateData1z(bodyRate1.z(), 2);
    DataProtocol::DataPacket bodyRatePack1;
    bodyRatePack1.addData(bodyRateData1x);
    bodyRatePack1.addData(bodyRateData1y);
    bodyRatePack1.addData(bodyRateData1z);
    DataProtocol::Data bodyRateData2x(bodyRate2.x(), 0);
    DataProtocol::Data bodyRateData2y(bodyRate2.y(), 1);
    DataProtocol::Data bodyRateData2z(bodyRate2.z(), 2);
    DataProtocol::DataPacket bodyRatePack2;
    bodyRatePack2.addData(bodyRateData2x);
    bodyRatePack2.addData(bodyRateData2y);
    bodyRatePack2.addData(bodyRateData2z);

    // packs center position
    DataProtocol::Data positionData0x(position0.x(), 0);
    DataProtocol::Data positionData0y(position0.y(), 1);
    DataProtocol::Data positionData0z(position0.z(), 2);
    DataProtocol::DataPacket positionPack0;
    positionPack0.addData(positionData0x);
    positionPack0.addData(positionData0y);
    positionPack0.addData(positionData0z);

    // packs velocities
    DataProtocol::Data velocityData0x(velocity0.x(), 0);
    DataProtocol::Data velocityData0y(velocity0.y(), 1);
    DataProtocol::Data velocityData0z(velocity0.z(), 2);
    DataProtocol::DataPacket velocityPack0;
    velocityPack0.addData(velocityData0x);
    velocityPack0.addData(velocityData0y);
    velocityPack0.addData(velocityData0z);
    DataProtocol::Data velocityData1x(velocity1.x(), 0);
    DataProtocol::Data velocityData1y(velocity1.y(), 1);
    DataProtocol::Data velocityData1z(velocity1.z(), 2);
    DataProtocol::DataPacket velocityPack1;
    velocityPack1.addData(velocityData1x);
    velocityPack1.addData(velocityData1y);
    velocityPack1.addData(velocityData1z);
    DataProtocol::Data velocityData2x(velocity2.x(), 0);
    DataProtocol::Data velocityData2y(velocity2.y(), 1);
    DataProtocol::Data velocityData2z(velocity2.z(), 2);
    DataProtocol::DataPacket velocityPack2;
    velocityPack2.addData(velocityData2x);
    velocityPack2.addData(velocityData2y);
    velocityPack2.addData(velocityData2z);

    // pakcs linear accelerations
    DataProtocol::Data linearAccelerationData0x(linearAcceleration0.x(), 0);
    DataProtocol::Data linearAccelerationData0y(linearAcceleration0.y(), 1);
    DataProtocol::Data linearAccelerationData0z(linearAcceleration0.z(), 2);
    DataProtocol::DataPacket linearAccelerationPack0;
    linearAccelerationPack0.addData(linearAccelerationData0x);
    linearAccelerationPack0.addData(linearAccelerationData0y);
    linearAccelerationPack0.addData(linearAccelerationData0z);
    DataProtocol::Data linearAccelerationData1x(linearAcceleration1.x(), 0);
    DataProtocol::Data linearAccelerationData1y(linearAcceleration1.y(), 1);
    DataProtocol::Data linearAccelerationData1z(linearAcceleration1.z(), 2);
    DataProtocol::DataPacket linearAccelerationPack1;
    linearAccelerationPack1.addData(linearAccelerationData1x);
    linearAccelerationPack1.addData(linearAccelerationData1y);
    linearAccelerationPack1.addData(linearAccelerationData1z);
    DataProtocol::Data linearAccelerationData2x(linearAcceleration2.x(), 0);
    DataProtocol::Data linearAccelerationData2y(linearAcceleration2.y(), 1);
    DataProtocol::Data linearAccelerationData2z(linearAcceleration2.z(), 2);
    DataProtocol::DataPacket linearAccelerationPack2;
    linearAccelerationPack2.addData(linearAccelerationData2x);
    linearAccelerationPack2.addData(linearAccelerationData2y);
    linearAccelerationPack2.addData(linearAccelerationData2z);

    // packs all data
    DataProtocol::DataPacket statesPack;
    statesPack.addData(eulerAnglePack0, 0);
    statesPack.addData(eulerAnglePack1, 1);
    statesPack.addData(eulerAnglePack2, 2);
    statesPack.addData(bodyRatePack0, 4);
    statesPack.addData(bodyRatePack1, 5);
    statesPack.addData(bodyRatePack2, 6);
    statesPack.addData(positionPack0, 8);
    statesPack.addData(velocityPack0, 9);
    statesPack.addData(velocityPack1, 10);
    statesPack.addData(velocityPack2, 11);
    statesPack.addData(linearAccelerationPack0, 13);
    statesPack.addData(linearAccelerationPack1, 14);
    statesPack.addData(linearAccelerationPack2, 15);

    // Eigen::Vector3d contactForces;
    // Eigen::Vector3d contactMoments;
    // for (auto& contact: robot_->getContacts())
    // {
    //   if (contact.skip()) continue;
    //   if (_states.toolIndex == contact.getlocalBodyIndex())
    //   {
    //     Eigen::Vector3d impulseOnA;
    //     auto impulse = contact.getImpulse().e();
    //     if (contact.isObjectA())
    //     {
    //       impulseOnA = impulse;
    //     }
    //     else // !contact.isObjectA()
    //     {
    //       impulseOnA = -impulse;
    //     }
    //     if (impulseOnA.x() != 0 || impulseOnA.y() != 0 || impulseOnA.z() != 0)
    //     {
    //       Eigen::Vector3d contactForce = convertFromFluToFrd(contact.getContactFrame().e().transpose() * impulseOnA / timeStep_);
    //       Eigen::Vector3d contactMoment = (matrix0 * Eigen::Vector3d(-0.8, 0, 0)).cross(contactForce);

    //       contactForces += contactForce;
    //       contactMoments += contactMoment;
    //     }
    //   }
    // }

    // DataProtocol::Data contactForceData0x(contactForces.x(), 0);
    // DataProtocol::Data contactForceData0y(contactForces.y(), 1);
    // DataProtocol::Data contactForceData0z(contactForces.z(), 2);
    // DataProtocol::DataPacket contactForcePack;
    // contactForcePack.addData(contactForceData0x);
    // contactForcePack.addData(contactForceData0y);
    // contactForcePack.addData(contactForceData0z);
    // DataProtocol::Data contactMomentData0x(contactMoments.x(), 0);
    // DataProtocol::Data contactMomentData0y(contactMoments.y(), 1);
    // DataProtocol::Data contactMomentData0z(contactMoments.z(), 2);
    // DataProtocol::DataPacket contactMomentPack;
    // contactMomentPack.addData(contactMomentData0x);
    // contactMomentPack.addData(contactMomentData0y);
    // contactMomentPack.addData(contactMomentData0z);

    // // packs contact forces and moments
    // statesPack.addData(contactForcePack, 17);
    // statesPack.addData(contactMomentPack, 18);
    
    // packs time
    if (initialTime_ == -1.0)
    {
      initialTime_ = currentTime;
      initialTimeMessage_.point.x = initialTime_;
      initialTimeMessage_.point.y = initialTime_;
      initialTimeMessage_.point.z = initialTime_;
      initialTimeMessage_.header.stamp = ros::Time::now();
    }
    DataProtocol::Data timeData0x(currentTime, 0);
    DataProtocol::Data timeData0y(currentTime, 1);
    DataProtocol::Data timeData0z(initialTime_, 2);
    DataProtocol::DataPacket timePack;
    timePack.addData(timeData0x);
    timePack.addData(timeData0y);
    timePack.addData(timeData0z);
    statesPack.addData(timePack, 19);

    // sends all states
    zmq::message_t stateMessage(statesPack.getEncodedDataString());
    auto result = statesPublisher_.send(stateMessage, zmq::send_flags::none);

    // publishes all ROS messages
    publishRosVector(eulerAnglePublisherNo0_, eulerAngle0, eulerAngleMessageNo0_);
    if (!ifUseVinsCallback_.get().data)
    {
      publishRosVector(positionPublisherNo0_, position0, positionMessageNo0_);
      publishRosVector(velocityPublisherNo0_, velocity0, velocityMessageNo0_);
    }
    publishRosVector(eulerAnglePublisherNo1_, eulerAngle1, eulerAngleMessageNo1_);
    publishRosVector(eulerAnglePublisherNo2_, eulerAngle2, eulerAngleMessageNo2_);
    publishRosVector(bodyRatePublisherNo0_, bodyRate0, bodyRateMessageNo0_);
    publishRosVector(bodyRatePublisherNo1_, bodyRate1, bodyRateMessageNo1_);
    publishRosVector(bodyRatePublisherNo2_, bodyRate2, bodyRateMessageNo2_);
    publishRosVector(positionPublisherNo1_, position1, positionMessageNo1_);
    publishRosVector(positionPublisherNo2_, position2, positionMessageNo2_);
    publishRosVector(velocityPublisherNo1_, velocity1, velocityMessageNo1_);
    publishRosVector(velocityPublisherNo2_, velocity2, velocityMessageNo2_);
    publishRosVector(linearAccelerationPublisherNo0_, linearAcceleration0, linearAccelerationMessageNo0_);
    publishRosVector(linearAccelerationPublisherNo1_, linearAcceleration1, linearAccelerationMessageNo1_);
    publishRosVector(linearAccelerationPublisherNo2_, linearAcceleration2, linearAccelerationMessageNo2_);
    // publishRosVector(contactForcePublisherNo0_, contactForces, contactForceMessage_);
    // publishRosVector(contactMomentPublisherNo0_, contactMoments, contactMomentMessage_);
    publishRosVector(timePublisher_, Eigen::Vector3d(currentTime, currentTime, initialTime_), timeMessage_);
    initialTimePublisher_.publish(initialTimeMessage_);
    auto imuTimeNow = ros::Time().fromSec(initialTimeMessage_.header.stamp.toSec() + currentTime - initialTime_);
    poseMessageNo0_.header.stamp = imuTimeNow;
    poseMessageNo1_.header.stamp = imuTimeNow;
    poseMessageNo2_.header.stamp = imuTimeNow;
    publishRosPose(posePublisherNo0_, position0, Eigen::Quaterniond(matrix0), poseMessageNo0_);
    publishRosPose(posePublisherNo1_, position1, Eigen::Quaterniond(matrix1), poseMessageNo1_);
    publishRosPose(posePublisherNo2_, position2, Eigen::Quaterniond(matrix2), poseMessageNo2_);

    // converts from FRD to FLU for SLAM
    bodyRate0 = convertFromFluToFrd(bodyRate0);
    bodyRate1 = convertFromFluToFrd(bodyRate1);
    bodyRate2 = convertFromFluToFrd(bodyRate2);
    linearAcceleration0 = convertFromFluToFrd(linearAcceleration0);
    linearAcceleration1 = convertFromFluToFrd(linearAcceleration1);
    linearAcceleration2 = convertFromFluToFrd(linearAcceleration2);

    // updates IMUs
    imuNo0_.updateDataWithNoise(bodyRate0, linearAcceleration0);
    imuNo1_.updateDataWithNoise(bodyRate1, linearAcceleration1);
    imuNo2_.updateDataWithNoise(bodyRate2, linearAcceleration2);

    imuMessageNo0_.header.stamp = imuTimeNow;
    imuMessageNo0_.orientation = poseMessageNo0_.pose.pose.orientation;
    imuMessageNo1_.header.stamp = imuTimeNow;
    imuMessageNo1_.orientation = poseMessageNo1_.pose.pose.orientation;
    imuMessageNo2_.header.stamp = imuTimeNow;
    imuMessageNo2_.orientation = poseMessageNo2_.pose.pose.orientation;
    publishRosImu(imuPublisherNo0_, imuNo0_, imuMessageNo0_, currentTime);
    publishRosImu(imuPublisherNo1_, imuNo1_, imuMessageNo1_, currentTime);
    publishRosImu(imuPublisherNo2_, imuNo2_, imuMessageNo2_, currentTime);

    // // debugs
    imuNoNoiseMessageNo0_.header.stamp = imuTimeNow;
    imuNoNoiseMessageNo0_.orientation = poseMessageNo0_.pose.pose.orientation;
    imuNoNoiseMessageNo1_.header.stamp = imuTimeNow;
    imuNoNoiseMessageNo1_.orientation = poseMessageNo1_.pose.pose.orientation;
    imuNoNoiseMessageNo2_.header.stamp = imuTimeNow;
    imuNoNoiseMessageNo2_.orientation = poseMessageNo2_.pose.pose.orientation;
    // imuTestMessageNo0_.header.stamp = imuTimeNow;
    // imuTestNoNoiseMessageNo0_.header.stamp = imuTimeNow;
    imuNoNoise0_.dataOmega = bodyRate0;
    imuNoNoise0_.dataAcc = linearAcceleration0;
    imuNoNoise1_.dataOmega = bodyRate1;
    imuNoNoise1_.dataAcc = linearAcceleration1;
    imuNoNoise2_.dataOmega = bodyRate2;
    imuNoNoise2_.dataAcc = linearAcceleration2;
    // imuTestNoNoise0_.dataOmega = bodyRate0;
    // imuTestNoNoise0_.dataAcc = velocityDerivative0;
    publishRosImu(imuNoNoisePublisherNo0_, imuNoNoise0_, imuNoNoiseMessageNo0_, currentTime);
    publishRosImu(imuNoNoisePublisherNo1_, imuNoNoise1_, imuNoNoiseMessageNo1_, currentTime);
    publishRosImu(imuNoNoisePublisherNo2_, imuNoNoise2_, imuNoNoiseMessageNo2_, currentTime);
    // publishRosImu(imuTestPublisherNo0_, imuTest0_, imuTestMessageNo0_, currentTime);
    // publishRosImu(imuTestNoNoisePublisherNo0_, imuTestNoNoise0_, imuTestNoNoiseMessageNo0_, currentTime);
    bag_.write("/imu_no_noise_0", imuTimeNow, imuNoNoiseMessageNo0_);
    bag_.write("/imu_no_noise_1", imuTimeNow, imuNoNoiseMessageNo1_);
    bag_.write("/imu_no_noise_2", imuTimeNow, imuNoNoiseMessageNo2_);
    // bag_.write("/imu_test_0", imuTimeNow, imuTestMessageNo0_);
    // bag_.write("/imu_test_no_noise_0", imuTimeNow, imuTestNoNoiseMessageNo0_);

    // writes messages in a bag
    bag_.write("/euler_angle_0", imuTimeNow, eulerAngleMessageNo0_);
    if (!ifUseVinsCallback_.get().data)
    {
      bag_.write("/position_0", imuTimeNow, positionMessageNo0_);
    }
    bag_.write("/euler_angle_1", imuTimeNow, eulerAngleMessageNo1_);
    bag_.write("/euler_angle_2", imuTimeNow, eulerAngleMessageNo2_);
    bag_.write("/initial_time", imuTimeNow, initialTimeMessage_);
    bag_.write("/simulation_time", imuTimeNow, timeMessage_);
    bag_.write("/imu_0", imuTimeNow, imuMessageNo0_);
    bag_.write("/imu_1", imuTimeNow, imuMessageNo1_);
    bag_.write("/imu_2", imuTimeNow, imuMessageNo2_);
    bag_.write("/pose_0", imuTimeNow, poseMessageNo0_);
    bag_.write("/pose_1", imuTimeNow, poseMessageNo1_);
    bag_.write("/pose_2", imuTimeNow, poseMessageNo2_);
    // bag_.write("/linear_acceleration_0", imuTimeNow, linearAccelerationMessageNo0_);
    // bag_.write("/linear_acceleration_1", imuTimeNow, linearAccelerationMessageNo1_);
    // bag_.write("/linear_acceleration_2", imuTimeNow, linearAccelerationMessageNo2_);

    // writes states to structure
    _states.rotation0_w = matrix0;
    _states.rotation1_0 = matrix1_0;
    _states.rotation2_0 = matrix2_0;
    _states.eulerAngle0 = eulerAngle0;
    _states.eulerAngle1 = eulerAngle1;
    _states.eulerAngle2 = eulerAngle2;
    _states.position0 = position0;
    _states.position1 = position1;
    _states.position2 = position2;
    _states.bodyRate0 = convertFromFluToFrd(bodyRate0);
    _states.bodyRate1 = bodyRate1;
    _states.bodyRate2 = bodyRate2;
    _states.velocity0 = velocity0;
    _states.velocity1 = velocity1;
    _states.velocity2 = velocity2;
    _states.linearAcceleration0 = linearAcceleration0;
    _states.linearAcceleration1 = linearAcceleration1;
    _states.linearAcceleration2 = linearAcceleration2;
}

void StatesSender::openRosbag(const std::string& _bagPath)
{
    bag_.open(_bagPath, rosbag::bagmode::Write);
}

void StatesSender::initializeRosPublishers()
{
    eulerAnglePublisherNo0_ = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/euler_angle_0", 10);
    eulerAnglePublisherNo1_ = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/euler_angle_1", 10);
    eulerAnglePublisherNo2_ = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/euler_angle_2", 10);
    bodyRatePublisherNo0_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/body_rate_0", 10);
    bodyRatePublisherNo1_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/body_rate_1", 10);
    bodyRatePublisherNo2_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/body_rate_2", 10);
    positionPublisherNo0_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/position_0", 10);
    positionPublisherNo1_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/position_1", 10);
    positionPublisherNo2_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/position_2", 10);
    velocityPublisherNo0_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/velocity_0", 10);
    velocityPublisherNo1_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/velocity_1", 10);
    velocityPublisherNo2_   = nodeHandle_.advertise<geometry_msgs::Point>
                              (robot_->getName() + "/velocity_2", 10);
    posePublisherNo0_       = nodeHandle_.advertise<nav_msgs::Odometry>
                              (robot_->getName() + "/pose_0", 10);
    posePublisherNo1_       = nodeHandle_.advertise<nav_msgs::Odometry>
                              (robot_->getName() + "/pose_1", 10);
    posePublisherNo2_       = nodeHandle_.advertise<nav_msgs::Odometry>
                              (robot_->getName() + "/pose_2", 10);
    linearAccelerationPublisherNo0_ = nodeHandle_.advertise<geometry_msgs::Point>
                                      (robot_->getName() + "/linear_acceleration_0", 10);
    linearAccelerationPublisherNo1_ = nodeHandle_.advertise<geometry_msgs::Point>
                                      (robot_->getName() + "/linear_acceleration_1", 10);
    linearAccelerationPublisherNo2_ = nodeHandle_.advertise<geometry_msgs::Point>
                                      (robot_->getName() + "/linear_acceleration_2", 10);
    imuPublisherNo0_           = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_0", 10);
    imuPublisherNo1_           = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_1", 10);
    imuPublisherNo2_           = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_2", 10);
    imuNoNoisePublisherNo0_    = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_no_noise_0", 10);
    imuNoNoisePublisherNo1_    = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_no_noise_1", 10);
    imuNoNoisePublisherNo2_    = nodeHandle_.advertise<sensor_msgs::Imu>
                                 (robot_->getName() + "/imu_no_noise_2", 10);
    // imuTestPublisherNo0_       = nodeHandle_.advertise<sensor_msgs::Imu>
    //                              (robot_->getName() + "/imu_test_0", 10);
    // imuTestNoNoisePublisherNo0_ = nodeHandle_.advertise<sensor_msgs::Imu>
    //                              (robot_->getName() + "/imu_test_no_noise_0", 10);
    contactForcePublisherNo0_  = nodeHandle_.advertise<geometry_msgs::Point>
                                 (robot_->getName() + "/external_force_world", 10);
    contactMomentPublisherNo0_ = nodeHandle_.advertise<geometry_msgs::Point>
                                 (robot_->getName() + "/external_moment_world", 10);
    timePublisher_        = nodeHandle_.advertise<geometry_msgs::Point>
                            (robot_->getName() + "/simulation_time", 10);
    initialTimePublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>
                            (robot_->getName() + "/initial_time", 10);
    // avaSlamDataPublisher_ = nodeHandle_.advertise<star3single::AvaSlamData>
    //                         (robot_->getName() + "/ava_slam_data", 10);
}

void StatesSender::initializeRosSubscribers()
{
    ifUseVinsSubscriber_ = nodeHandle_.subscribe<std_msgs::Bool>
                           ("use_vins", 1, &IfUseVins::callback, &ifUseVinsCallback_);
}

void StatesSender::initializeServer()
{
    statesPublisher_ = zmq::socket_t(statesContext_, zmq::socket_type::pub);
    statesPublisher_.bind(std::string("tcp://") + ip_ + std::string(":") + statesPort_);
}

void StatesSender::publishRosVector(const ros::Publisher& _publisher, const Eigen::Vector3d& _vector, geometry_msgs::Point& _message)
{
    _message.x = _vector.x();
    _message.y = _vector.y();
    _message.z = _vector.z();

    _publisher.publish(_message);
}

void StatesSender::publishRosPose(const ros::Publisher& _publisher, const Eigen::Vector3d& _position, const Eigen::Quaterniond& _quaternion, nav_msgs::Odometry& _message)
{
    _message.pose.pose.position.x =  _position.x();
    _message.pose.pose.position.y = -_position.y();
    _message.pose.pose.position.z = -_position.z();
    // Eigen::Quaterniond quaternionRoll = Eigen::Quaterniond(0, 1, 0, 0);
    auto quaternionFlu = _quaternion;
    quaternionFlu.y() = -quaternionFlu.y();
    quaternionFlu.z() = -quaternionFlu.z();
    // auto quaternion = quaternionFlu * quaternionRoll;
    _message.pose.pose.orientation.w = quaternionFlu.w();
    _message.pose.pose.orientation.x = quaternionFlu.x();
    _message.pose.pose.orientation.y = quaternionFlu.y();
    _message.pose.pose.orientation.z = quaternionFlu.z();

    _publisher.publish(_message);
}

void StatesSender::publishRosImu(const ros::Publisher& _publisher, const Imu& _imu, sensor_msgs::Imu& _message, double _simulationTime)
{
    _message.angular_velocity.x = _imu.dataOmega.x();
    _message.angular_velocity.y = _imu.dataOmega.y();
    _message.angular_velocity.z = _imu.dataOmega.z();
    _message.linear_acceleration.x = _imu.dataAcc.x();
    _message.linear_acceleration.y = _imu.dataAcc.y();
    _message.linear_acceleration.z = _imu.dataAcc.z();
    _message.orientation_covariance[0] = _simulationTime;

    _publisher.publish(_message);
}

Eigen::Vector3d convertFromFluToFrd(const Eigen::Vector3d& _vectorFlu)
{
    Eigen::Vector3d vector_in_frd;

    vector_in_frd.x() =  _vectorFlu.x();
    vector_in_frd.y() = -_vectorFlu.y();
    vector_in_frd.z() = -_vectorFlu.z();

    return vector_in_frd;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& _eulerAngle)
{
	  const double& Roll  = _eulerAngle(0);
	  const double& Pitch = _eulerAngle(1);
	  const double& Yaw   = _eulerAngle(2);

	  Eigen::Matrix3d RotationMatrix;
	  RotationMatrix(0, 0) = cos(Pitch) * cos(Yaw);
	  RotationMatrix(0, 1) = sin(Roll) * sin(Pitch) * cos(Yaw) - cos(Roll) * sin(Yaw);
	  RotationMatrix(0, 2) = cos(Roll) * sin(Pitch) * cos(Yaw) + sin(Roll) * sin(Yaw);
	  RotationMatrix(1, 0) = cos(Pitch) * sin(Yaw);
	  RotationMatrix(1, 1) = sin(Roll) * sin(Pitch) * sin(Yaw) + cos(Roll) * cos(Yaw);
	  RotationMatrix(1, 2) = cos(Roll) * sin(Pitch) * sin(Yaw) - sin(Roll) * cos(Yaw);
	  RotationMatrix(2, 0) = -sin(Pitch);
	  RotationMatrix(2, 1) = sin(Roll) * cos(Pitch);
	  RotationMatrix(2, 2) = cos(Roll) * cos(Pitch);

	  return RotationMatrix;
}

Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& _rotation)
{
    Eigen::Vector3d eulerAngle;
    double& Roll  = eulerAngle(0);
    double& Pitch = eulerAngle(1);
    double& Yaw   = eulerAngle(2);

    Pitch = asin(-_rotation(2, 0));
    Roll = asin(_rotation(2, 1) / cos(Pitch));
    
    double sinYaw = _rotation(1, 0) / cos(Pitch);
    double cosYaw = _rotation(0, 0) / cos(Pitch);
    Yaw = atan2(sinYaw, cosYaw);
    
    return eulerAngle;
}

Eigen::Vector3d getEulerAngle(const Eigen::Quaterniond& _quaternion)
{
    Eigen::Vector3d eulerAngle;
    eulerAngle.x() = atan2(2 * (_quaternion.w() * _quaternion.x() + _quaternion.y() * _quaternion.z()), 1 - 2 * (_quaternion.x() * _quaternion.x() + _quaternion.y() * _quaternion.y()));
    eulerAngle.y() = asin(2 * (_quaternion.w() * _quaternion.y() - _quaternion.x() * _quaternion.z()));
    eulerAngle.z() = atan2(2 * (_quaternion.w() * _quaternion.z() + _quaternion.x() * _quaternion.y()), 1 - 2 * (_quaternion.y() * _quaternion.y() + _quaternion.z() * _quaternion.z()));

    return eulerAngle;
}

Eigen::Matrix3d getSkewMatrix(const Eigen::Vector3d& _vector)
{
	  Eigen::Matrix3d SkewMatrix;
	  SkewMatrix <<              0, -_vector(2, 0),  _vector(1, 0),
	    	           _vector(2, 0),              0, -_vector(0, 0),
		              -_vector(1, 0),  _vector(0, 0),              0;

	return SkewMatrix;
}
