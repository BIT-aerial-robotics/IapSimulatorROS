// main.cpp
// ROS node
// Du Jianrui   2023.12.07
// uses a PlayStation 5 DualSense Controller to generate a trajectory

#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Joy.h"

#include <cmath>
#include "Eigen/Dense"

#include "rosbag/bag.h"

class Joystick
{
public:
    void callback(const sensor_msgs::Joy::ConstPtr& _msg)
    {
        joy_ = *_msg;

        vx = xvMax_ * joy_.axes[1];
        vy = -yvMax_ * joy_.axes[0];
        vz = -zvMax_ * joy_.axes[5];
        vp = rollRateMax_ * (joy_.buttons[5] - joy_.buttons[4]);
        vq = pitchRateMax_ * (joy_.buttons[7] - joy_.buttons[6]);
        vr = -yawRateMax_ * joy_.axes[2];

        if (joy_.buttons[3] == 1)
        {
            bUseVins = !bUseVins;
        }
    }

    void setMaxVelocity(double _xvMax, double _yvMax, double _zvMax)
    {
        xvMax_ = _xvMax;
        yvMax_ = _yvMax;
        zvMax_ = _zvMax;
    }
    void setMaxBodyRate(double _rollRateMax, double _pitchRateMax, double _yawRateMax)
    {
        rollRateMax_ = _rollRateMax;
        pitchRateMax_ = _pitchRateMax;
        yawRateMax_ = _yawRateMax;
    }

    sensor_msgs::Joy get() const
    {
        return joy_;
    }
    double getVelocityX() const { return vx; }
    double getVelocityY() const { return vy; }
    double getVelocityZ() const { return vz; }
    double getBodyRateX() const { return vp; }
    double getBodyRateY() const { return vq; }
    double getBodyRateZ() const { return vr; }
    bool getIfUseVins() const { return bUseVins; }

private:
    sensor_msgs::Joy joy_;

    double xvMax_ = 1.5;
    double yvMax_ = 1.5;
    double zvMax_ = 1.5;
    double rollRateMax_ = M_PI / 9.0;
    double pitchRateMax_ = M_PI / 9.0;
    double yawRateMax_ = M_PI / 9.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double vp = 0.0;
    double vq = 0.0;
    double vr = 0.0;

    bool bUseVins = false;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_input");
    ros::NodeHandle nh;

    double rate = 100.0;
    ros::Rate loopRate(rate);

    // gets parameters from the launch file
    std::string rosbagPath;
    double xvMax, yvMax, zvMax, rollRateMax, pitchRateMax, yawRateMax;
    ros::param::get("~rosbagPath", rosbagPath);
    ros::param::get("~xvMax", xvMax);
    ros::param::get("~yvMax", yvMax);
    ros::param::get("~zvMax", zvMax);
    ros::param::get("~rollRateMax", rollRateMax);
    ros::param::get("~pitchRateMax", pitchRateMax);
    ros::param::get("~yawRateMax", yawRateMax);

    Joystick joyCallback;
    joyCallback.setMaxVelocity(xvMax, yvMax, zvMax);
    joyCallback.setMaxBodyRate(rollRateMax, pitchRateMax, yawRateMax);
    ros::Subscriber joySubscriber_ = nh.subscribe<sensor_msgs::Joy>
                                     ("joy", 1, &Joystick::callback, &joyCallback);

    // defines publishers
    ros::Publisher desiredPositionPublisher = nh.advertise<geometry_msgs::Point>
                                              ("nominal_position_0", 10);
    ros::Publisher desiredEulerAnglePublisher = nh.advertise<geometry_msgs::Point>
                                                ("nominal_euler_angle_0", 10);
    ros::Publisher desiredVelocityPublisher = nh.advertise<geometry_msgs::Point>
                                              ("nominal_velocity_0", 10);
    ros::Publisher desiredBodyRatePublisher = nh.advertise<geometry_msgs::Point>
                                                ("nominal_body_rate_0", 10);
    ros::Publisher initialEulerAnglePublisher1 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_1", 10);
    ros::Publisher initialEulerAnglePublisher2 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_2", 10);
    ros::Publisher initialEulerAnglePublisher3 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_3", 10);
    ros::Publisher ifUseVinsPublisher = nh.advertise<std_msgs::Bool>
                                        ("use_vins", 10);
    
    // defines ROS message variables
    geometry_msgs::Point desiredPositionMessage;
    geometry_msgs::Point desiredEulerAngleMessage;
    geometry_msgs::Point desiredVelocityMessage;
    geometry_msgs::Point desiredBodyRateMessage;
    geometry_msgs::Point initialEulerAngleMessage1;
    geometry_msgs::Point initialEulerAngleMessage2;
    geometry_msgs::Point initialEulerAngleMessage3;
    std_msgs::Bool ifUseVinsMessage;

    // defines variables to manipulate
    double& positionX = desiredPositionMessage.x;
    double& positionY = desiredPositionMessage.y;
    double& positionZ = desiredPositionMessage.z;
    double& eulerAngleX = desiredEulerAngleMessage.x;
    double& eulerAngleY = desiredEulerAngleMessage.y;
    double& eulerAngleZ = desiredEulerAngleMessage.z;
    double& velocityX = desiredVelocityMessage.x;
    double& velocityY = desiredVelocityMessage.y;
    double& velocityZ = desiredVelocityMessage.z;
    double& bodyRateX = desiredBodyRateMessage.x;
    double& bodyRateY = desiredBodyRateMessage.y;
    double& bodyRateZ = desiredBodyRateMessage.z;
    double& initialEulerAngle1X = initialEulerAngleMessage1.x;
    double& initialEulerAngle1Y = initialEulerAngleMessage1.y;
    double& initialEulerAngle1Z = initialEulerAngleMessage1.z;
    double& initialEulerAngle2X = initialEulerAngleMessage2.x;
    double& initialEulerAngle2Y = initialEulerAngleMessage2.y;
    double& initialEulerAngle2Z = initialEulerAngleMessage2.z;
    double& initialEulerAngle3X = initialEulerAngleMessage3.x;
    double& initialEulerAngle3Y = initialEulerAngleMessage3.y;
    double& initialEulerAngle3Z = initialEulerAngleMessage3.z;
    
    positionX = 0.0;
    positionY = 0.0;
    positionZ = 0.0;
    eulerAngleX = 0.0;
    eulerAngleY = 0.0;
    eulerAngleZ = 0.0;
    velocityX = 0.0;
    velocityY = 0.0;
    velocityZ = 0.0;
    bodyRateX = 0.0;
    bodyRateY = 0.0;
    bodyRateZ = 0.0;
    initialEulerAngle1X = 0.0;
    initialEulerAngle1Y = 0.0;
    initialEulerAngle1Z = 0.0;
    initialEulerAngle2X = 0.0;
    initialEulerAngle2Y = 0.0;
    initialEulerAngle2Z = 2 * M_PI / 3;
    initialEulerAngle3X = 0.0;
    initialEulerAngle3Y = 0.0;
    initialEulerAngle3Z = -2 * M_PI / 3;

    initialEulerAnglePublisher1.publish(initialEulerAngleMessage1);
    initialEulerAnglePublisher2.publish(initialEulerAngleMessage2);
    initialEulerAnglePublisher3.publish(initialEulerAngleMessage3);

    // opens a rosbag
    rosbag::Bag trajectoryBag;
    trajectoryBag.open(rosbagPath, rosbag::bagmode::Write);

    // positionZ = -0.1;
    while (ros::ok())
    {
        positionX += (joyCallback.getVelocityX() * std::cos(eulerAngleZ)
                    - joyCallback.getVelocityY() * std::sin(eulerAngleZ)) / rate;
        positionY += (joyCallback.getVelocityX() * std::sin(eulerAngleZ)
                    + joyCallback.getVelocityY() * std::cos(eulerAngleZ)) / rate;
        positionZ += joyCallback.getVelocityZ() / rate;
        eulerAngleX += joyCallback.getBodyRateX() / rate;
        eulerAngleY += joyCallback.getBodyRateY() / rate;
        eulerAngleZ += joyCallback.getBodyRateZ() / rate;
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        velocityX = joyCallback.getVelocityX();
        velocityY = joyCallback.getVelocityY();
        velocityZ = joyCallback.getVelocityZ();
        bodyRateX = joyCallback.getBodyRateX();
        bodyRateY = joyCallback.getBodyRateY();
        bodyRateZ = joyCallback.getBodyRateZ();
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);

        ifUseVinsMessage.data = joyCallback.getIfUseVins();
        ifUseVinsPublisher.publish(ifUseVinsMessage);

        auto currentTime = ros::Time::now();
        trajectoryBag.write("/nominal_position_0", currentTime, desiredPositionMessage);
        trajectoryBag.write("/nominal_euler_angle_0", currentTime, desiredEulerAngleMessage);
        trajectoryBag.write("/nominal_velocity_0", currentTime, desiredVelocityMessage);
        trajectoryBag.write("/nominal_body_rate_0", currentTime, desiredBodyRateMessage);
        
        ros::spinOnce();
        loopRate.sleep();
    }
}
