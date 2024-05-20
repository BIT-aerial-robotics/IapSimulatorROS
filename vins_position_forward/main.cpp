// main.cpp
// ROS node
// Du Jianrui   2023.12.12
// forwards vins positioning result

#include "ros/ros.h"
#include "rosbag/bag.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

#include <cmath>
#include <Eigen/Dense>

class VinsEstimation
{
public:
    void callback(const nav_msgs::Odometry::ConstPtr& _msg)
    {
        estimation_ = *_msg;
    }
    nav_msgs::Odometry get() const
    {
        return estimation_;
    }

private:
    nav_msgs::Odometry estimation_;
};

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

class TruePosition
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        position_ = *_msg;
        if (bUseTruePosition_)
        {
            lastPosition_.x() = position_.x;
            lastPosition_.y() = position_.y;
            lastPosition_.z() = position_.z;
            
            bLastUseTruePosition_ = bUseTruePosition_;
        }

        if (!bUseTruePosition_ && bLastUseTruePosition_)
        {
            bSwitchToVins_ = true;
        }
        else
        {
            bSwitchToVins_ = false;
        }
        if (bSwitchToVins_)
        {
            positionBias_.x() = lastPosition_.x() - position_.x;
            positionBias_.y() = lastPosition_.y() - position_.y;
            positionBias_.z() = lastPosition_.z() - position_.z;
        }
    }
    void setIfUseVins(bool _bUseVins)
    {
        bUseTruePosition_ = !_bUseVins;
    }
    geometry_msgs::Point get() const
    {
        return position_;
    }
    Eigen::Vector3d getLastTruePosition() const
    {
        return lastPosition_;
    }
    Eigen::Vector3d getPositionBias() const
    {
        return positionBias_;
    }
private:
    geometry_msgs::Point position_;
    Eigen::Vector3d lastPosition_;
    Eigen::Vector3d positionBias_;
    bool bUseTruePosition_ = true;
    bool bLastUseTruePosition_ = true;
    bool bSwitchToVins_ = false;
};


Eigen::Vector3d convertFromFluToFrd(const Eigen::Vector3d& _vectorFlu);
Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& _rotation);
void publishRosVector(const ros::Publisher& _publisher, const Eigen::Vector3d& _vector, geometry_msgs::Point& _message, bool bPublishRos);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_position_forward");
    ros::NodeHandle nh;

    double rate = 200.0;
    ros::Rate loopRate(rate);

    // operates the rosbag
    rosbag::Bag estimationBag;
    std::string rosbagPath;
    ros::param::get("~rosbagPath", rosbagPath);
    estimationBag.open(rosbagPath, rosbag::bagmode::Write);
    
    // defines ROS message variables
    geometry_msgs::Point eulerAngleMessageNo0;
    geometry_msgs::Point positionMessageNo0;
    geometry_msgs::Point velocityMessageNo0;
    std_msgs::Bool ifUseVinsMessage;
    VinsEstimation estimationCallback;
    IfUseVins ifUseVinsCallback;
    TruePosition truePositionCallback;

    // defines the publishers
    ros::Publisher eulerAnglePublisherNo0 = nh.advertise<geometry_msgs::Point>
                                            ("euler_angle_0", 10);
    ros::Publisher positionPublisherNo0   = nh.advertise<geometry_msgs::Point>
                                            ("position_0", 10);
    ros::Publisher velocityPublisherNo0   = nh.advertise<geometry_msgs::Point>
                                            ("velocity_0", 10);

    // defines the subscribers
    ros::Subscriber estimationSubscriberNo0 = nh.subscribe<nav_msgs::Odometry>
                                              ("/vins_estimator/imu_propagate", 1, &VinsEstimation::callback, &estimationCallback);
    ros::Subscriber ifUseVinsSubscriber = nh.subscribe<std_msgs::Bool>
                                          ("use_vins", 1, &IfUseVins::callback, &ifUseVinsCallback);
    ros::Subscriber positionSubScriber = nh.subscribe<geometry_msgs::Point>
                                         ("position_0", 1, &TruePosition::callback, &truePositionCallback);

    Eigen::Vector3d eulerAngle0, position0, velocity0;
    Eigen::Quaterniond quaternion0;
    Eigen::Matrix3d matrix0;

    while (ros::ok())
    {   
        truePositionCallback.setIfUseVins(ifUseVinsCallback.get().data);
        
        // gets original estimation position, linear velocity, quaternion in world NWU
        position0.x() = estimationCallback.get().pose.pose.position.x;
        position0.y() = estimationCallback.get().pose.pose.position.y;
        position0.z() = estimationCallback.get().pose.pose.position.z;
        velocity0.x() = estimationCallback.get().twist.twist.linear.x;
        velocity0.y() = estimationCallback.get().twist.twist.linear.y;
        velocity0.z() = estimationCallback.get().twist.twist.linear.z;
        quaternion0.w() = estimationCallback.get().pose.pose.orientation.w;
        quaternion0.x() = estimationCallback.get().pose.pose.orientation.x;
        quaternion0.y() = estimationCallback.get().pose.pose.orientation.y;
        quaternion0.z() = estimationCallback.get().pose.pose.orientation.z;
        // converts the position, linear velocity, and quaternion to world NED
        position0 = convertFromFluToFrd(position0);
        velocity0 = convertFromFluToFrd(velocity0);
        quaternion0 = quaternion0 * Eigen::Quaterniond(0, 1, 0, 0);
        // gets the Euler angle / the RPY angle
        matrix0 = Eigen::Matrix3d(quaternion0);
        eulerAngle0 = getEulerAngle(matrix0);
        // gets the velocity in body-fixed FRD
        velocity0 = matrix0.transpose() * velocity0;

        // if (ifUseVinsCallback.get().data)
        // {
        //     position0 += truePositionCallback.getPositionBias();
        // }

        // publishes
        // publishRosVector(eulerAnglePublisherNo0, eulerAngle0, eulerAngleMessageNo0, ifUseVinsCallback.get().data);
        publishRosVector(positionPublisherNo0, position0, positionMessageNo0, ifUseVinsCallback.get().data);
        publishRosVector(velocityPublisherNo0, velocity0, velocityMessageNo0, ifUseVinsCallback.get().data);

        // records
        auto currentTime = ros::Time::now();
        // estimationBag.write("/euler_angle_0", currentTime, eulerAngleMessageNo0);
        estimationBag.write("/position_0", currentTime, positionMessageNo0);
        estimationBag.write("/velocity_0", currentTime, velocityMessageNo0);

        ros::spinOnce();
        loopRate.sleep();
    }
}

Eigen::Vector3d convertFromFluToFrd(const Eigen::Vector3d& _vectorFlu)
{
    Eigen::Vector3d vector_in_frd;

    vector_in_frd.x() =  _vectorFlu.x();
    vector_in_frd.y() = -_vectorFlu.y();
    vector_in_frd.z() = -_vectorFlu.z();

    return vector_in_frd;
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

void publishRosVector(const ros::Publisher& _publisher, const Eigen::Vector3d& _vector, geometry_msgs::Point& _message, bool _bPublishRos)
{
    _message.x = _vector.x();
    _message.y = _vector.y();
    _message.z = _vector.z();

    if (_bPublishRos)
    {
        _publisher.publish(_message);
    }
}
