// StatesSender.h
// declarations to send assembly robot states
// Du Jianrui   2023.9.23

#include "Common.hpp"
#include <zmq.hpp>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include "rosbag/bag.h"

class IfUseVins
{
public:
    void callback(const std_msgs::Bool::ConstPtr& _msg)
    {
        bUseVins_ = *_msg;
    }
    std_msgs::Bool get() const
    {
        return bUseVins_;
    }
private:
    std_msgs::Bool bUseVins_;
};

class StatesSender
{
public:
    StatesSender(const ros::NodeHandle& _nodeHandle,
                 raisim::ArticulatedSystem* _robot,
                 raisim::World* _world,
                 double _timeStep,
                 const std::string& _localHostIp,
                 const std::string& _dataPort);
    StatesSender(const StatesSender& _sender) = delete;
    StatesSender operator=(const StatesSender& _anotherSender) = delete;

    void getAndSendStates(AvaRobotState& _states);

    ~StatesSender()
    {
        robot_ = nullptr;
        world_ = nullptr;

        bag_.close();
    }

public:
    void openRosbag(const std::string& _bagPath);

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
    raisim::World* world_;
    double timeStep_;

private: // lists zmq ports
    std::string ip_;
    std::string statesPort_;

private: // lists zmq sockets and messages
    zmq::context_t statesContext_;
    zmq::socket_t statesPublisher_;

private: // lists publishers here
    // publisher handles
    ros::Publisher eulerAnglePublisherNo0_;
    ros::Publisher eulerAnglePublisherNo1_;
    ros::Publisher eulerAnglePublisherNo2_;
    ros::Publisher eulerAnglePublisherNo3_;
    ros::Publisher bodyRatePublisherNo0_;
    ros::Publisher bodyRatePublisherNo1_;
    ros::Publisher bodyRatePublisherNo2_;
    ros::Publisher bodyRatePublisherNo3_;
    ros::Publisher positionPublisherNo0_;
    ros::Publisher positionPublisherNo1_;
    ros::Publisher positionPublisherNo2_;
    ros::Publisher positionPublisherNo3_;
    ros::Publisher velocityPublisherNo0_;
    ros::Publisher velocityPublisherNo1_;
    ros::Publisher velocityPublisherNo2_;
    ros::Publisher velocityPublisherNo3_;
    ros::Publisher posePublisherNo0_;
    ros::Publisher posePublisherNo1_;
    ros::Publisher posePublisherNo2_;
    ros::Publisher posePublisherNo3_;
    ros::Publisher linearAccelerationPublisherNo0_;
    ros::Publisher linearAccelerationPublisherNo1_;
    ros::Publisher linearAccelerationPublisherNo2_;
    ros::Publisher linearAccelerationPublisherNo3_;
    ros::Publisher imuPublisherNo0_;
    ros::Publisher imuPublisherNo1_;
    ros::Publisher imuPublisherNo2_;
    ros::Publisher imuPublisherNo3_;
    ros::Publisher imuNoNoisePublisherNo0_;
    ros::Publisher imuNoNoisePublisherNo1_;
    ros::Publisher imuNoNoisePublisherNo2_;
    ros::Publisher imuNoNoisePublisherNo3_;
    ros::Publisher contactForcePublisherNo0_;
    ros::Publisher contactMomentPublisherNo0_;
    ros::Publisher contactForceWorldPublisherNo0_;
    ros::Publisher contactMomentWorldPublisherNo0_;
    ros::Publisher timePublisher_;
    // messages to publish
    geometry_msgs::Point eulerAngleMessageNo0_;
    geometry_msgs::Point eulerAngleMessageNo1_;
    geometry_msgs::Point eulerAngleMessageNo2_;
    geometry_msgs::Point eulerAngleMessageNo3_;
    geometry_msgs::Point bodyRateMessageNo0_;
    geometry_msgs::Point bodyRateMessageNo1_;
    geometry_msgs::Point bodyRateMessageNo2_;
    geometry_msgs::Point bodyRateMessageNo3_;
    geometry_msgs::Point positionMessageNo0_;
    geometry_msgs::Point positionMessageNo1_;
    geometry_msgs::Point positionMessageNo2_;
    geometry_msgs::Point positionMessageNo3_;
    geometry_msgs::Point velocityMessageNo0_;
    geometry_msgs::Point velocityMessageNo1_;
    geometry_msgs::Point velocityMessageNo2_;
    geometry_msgs::Point velocityMessageNo3_;
    nav_msgs::Odometry poseMessageNo0_;
    nav_msgs::Odometry poseMessageNo1_;
    nav_msgs::Odometry poseMessageNo2_;
    nav_msgs::Odometry poseMessageNo3_;
    geometry_msgs::Point linearAccelerationMessageNo0_;
    geometry_msgs::Point linearAccelerationMessageNo1_;
    geometry_msgs::Point linearAccelerationMessageNo2_;
    geometry_msgs::Point linearAccelerationMessageNo3_;
    sensor_msgs::Imu imuMessageNo0_;
    sensor_msgs::Imu imuMessageNo1_;
    sensor_msgs::Imu imuMessageNo2_;
    sensor_msgs::Imu imuMessageNo3_;
    sensor_msgs::Imu imuNoNoiseMessageNo0_;
    sensor_msgs::Imu imuNoNoiseMessageNo1_;
    sensor_msgs::Imu imuNoNoiseMessageNo2_;
    sensor_msgs::Imu imuNoNoiseMessageNo3_;
    geometry_msgs::Point contactForceMessage_;
    geometry_msgs::Point contactMomentMessage_;
    geometry_msgs::Point contactForceWorldMessage_;
    geometry_msgs::Point contactMomentWorldMessage_;
    geometry_msgs::Point timeMessage_;

private: // whether to use vins result to provide locationing data
    IfUseVins ifUseVinsCallback_;
    ros::Subscriber ifUseVinsSubscriber_;
    std_msgs::Bool ifUseVinsMessage_;

private: // imu calculation-related
    Eigen::Vector3d velocityLast0_;
    Eigen::Vector3d velocityLast1_;
    Eigen::Vector3d velocityLast2_;
    Eigen::Vector3d velocityLast3_;
    Imu imuNo0_;
    Imu imuNo1_;
    Imu imuNo2_;
    Imu imuNo3_;
    Imu imuNoNoise0_;
    Imu imuNoNoise1_;
    Imu imuNoNoise2_;
    Imu imuNoNoise3_;

private:
    Eigen::Vector3d originalEulerAngle1_; // original Euler angle from joint1 to base frame
    Eigen::Vector3d originalEulerAngle2_; // original Euler angle from joint2 to base frame
    Eigen::Vector3d originalEulerAngle3_; // original Euler angle from joint3 to base frame
    Eigen::Vector3d relativePosition1_; // child drone1 relative position in base frame
    Eigen::Vector3d relativePosition2_; // child drone2 relative position in base frame
    Eigen::Vector3d relativePosition3_; // child drone3 relative position in base frame

private: // time sync
    ros::Publisher initialTimePublisher_;
    geometry_msgs::PointStamped initialTimeMessage_;
    double initialTime_ = -1.0;

private:
    rosbag::Bag bag_;

private:
    void initializeRosPublishers();
    void initializeRosSubscribers();
    void initializeServer();

    void publishRosVector(const ros::Publisher& _publisher, const Eigen::Vector3d& _vector, geometry_msgs::Point& _message);
    void publishRosPose(const ros::Publisher& _publisher, const Eigen::Vector3d& _position, const Eigen::Quaterniond& _quaternion, nav_msgs::Odometry& _message);
    void publishRosImu(const ros::Publisher& _publisher, const Imu& _imu, sensor_msgs::Imu& _message, double _simulationTime);
};


Eigen::Vector3d convertFromFluToFrd(const Eigen::Vector3d& _vectorFlu);
Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& _eulerAngle);
Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& _rotation);
Eigen::Vector3d getEulerAngle(const Eigen::Quaterniond& _quaternion);
Eigen::Matrix3d getSkewMatrix(const Eigen::Vector3d& _vector);
