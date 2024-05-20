// main.cpp
// ROS node
// Du Jianrui   2022.2.8
// a test for UE4 model validation

#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_square");
    ros::NodeHandle nh;

    double rate = 100.0;
    ros::Rate loopRate(rate);

    // defines publishers
    ros::Publisher desiredPositionPublisher = nh.advertise<geometry_msgs::Point>
                                              ("nominal_position_0", 10);
    ros::Publisher desiredEulerAnglePublisher = nh.advertise<geometry_msgs::Point>
                                                ("nominal_euler_angle_0", 10);
    ros::Publisher initialEulerAnglePublisher1 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_1", 10);
    ros::Publisher initialEulerAnglePublisher2 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_2", 10);
    ros::Publisher initialEulerAnglePublisher3 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_3", 10);
    
    // defines ROS message variables
    geometry_msgs::Point desiredPositionMessage;
    geometry_msgs::Point desiredEulerAngleMessage;
    geometry_msgs::Point initialEulerAngleMessage1;
    geometry_msgs::Point initialEulerAngleMessage2;
    geometry_msgs::Point initialEulerAngleMessage3;

    // defines variables to manipulate
    double& positionX = desiredPositionMessage.x;
    double& positionY = desiredPositionMessage.y;
    double& positionZ = desiredPositionMessage.z;
    double& eulerAngleX = desiredEulerAngleMessage.x;
    double& eulerAngleY = desiredEulerAngleMessage.y;
    double& eulerAngleZ = desiredEulerAngleMessage.z;
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
    initialEulerAngle1X = 0.0;
    initialEulerAngle1Y = 0.0;
    initialEulerAngle1Z = 0.0;
    initialEulerAngle2X = 0.0;
    initialEulerAngle2Y = 0.0;
    initialEulerAngle2Z = 0.0;
    initialEulerAngle3X = 0.0;
    initialEulerAngle3Y = 0.0;
    initialEulerAngle3Z = 0.0;

    double i = 0;
    positionZ = -0.1;
    while (ros::ok() && i <= 5)
    {
        initialEulerAnglePublisher1.publish(initialEulerAngleMessage1);
        initialEulerAnglePublisher2.publish(initialEulerAngleMessage2);
        initialEulerAnglePublisher3.publish(initialEulerAngleMessage3);

        i += 1 / rate;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // initialization

    i = 0;
    while (ros::ok() && i <=10)
    {
        positionZ -= 0.2 / rate;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // take off

    i = 0;
    while (ros::ok() && i <= 5)
    {
        eulerAngleX -= 0.05 / rate;
        eulerAngleY += 0.05 / rate;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // change orientation

    i = 0;
    while (ros::ok() && i <= 5)
    {
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // hold orientation

    i = 0;
    while (ros::ok() && i <= 5)
    {
        eulerAngleX += 0.05 / rate;
        eulerAngleY -= 0.05 / rate;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // change orientation

    i = 0;
    while (ros::ok() && i <= 5)
    {
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // hold orientation

    i = 0;
    while (ros::ok() && i <= 32)
    {
        positionX = 1 * std::sin(i * 2 * M_PI / 16);
        positionY = 1 * (std::cos(i * 2 * M_PI / 16) - 1.0);
        positionZ = 0.5 * std::sin(i * 2 * M_PI / 16) - 2.1;
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // change position

    i = 0;
    while (ros::ok() && i <= 16)
    {
        eulerAngleX += 0.05 / rate * 5 / 16;
        eulerAngleY -= 0.05 / rate * 5 / 16;
        positionX = 1 * std::sin(i * 2 * M_PI / 16);
        positionY = 1 * (std::cos(i * 2 * M_PI / 16) - 1.0);
        positionZ = 0.5 * std::sin(i * 2 * M_PI / 16) - 2.1;
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // change position and orientation simutaneously

    i = 0;
    while (ros::ok() && i <= 16)
    {
        positionX = 1 * std::sin(i * 2 * M_PI / 16);
        positionY = 1 * (std::cos(i * 2 * M_PI / 16) - 1.0);
        positionZ += 0.1 / rate;
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // change position and descend

    i = 0;
    while (ros::ok() && i <= 16)
    {
        eulerAngleX -= 0.05 / rate * 5 / 16;
        eulerAngleY += 0.05 / rate * 5 / 16;
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // change position and orientation simutaneously

    i = 0;
    while (ros::ok())
    {
        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // hold position
}
