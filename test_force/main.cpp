// main.cpp
// ROS node
// Du Jianrui   2023.11.5
// a test for UE4 model validation

#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_force");
    ros::NodeHandle nh;

    double rate = 50.0;
    ros::Rate loopRate(rate);

    // defines publishers
    ros::Publisher desiredPositionPublisher    = nh.advertise<geometry_msgs::Point>
                                                 ("nominal_position_0", 10);
    ros::Publisher desiredEulerAnglePublisher  = nh.advertise<geometry_msgs::Point>
                                                 ("nominal_euler_angle_0", 10);
    ros::Publisher desiredVelocityPublisher    = nh.advertise<geometry_msgs::Point>
                                                 ("nominal_velocity", 10);
    ros::Publisher desiredBodyRatePublisher    = nh.advertise<geometry_msgs::Point>
                                                 ("nominal_body_rate", 10);
    ros::Publisher desiredForcePublisher       = nh.advertise<geometry_msgs::Point>
                                                 ("desired_force", 10);
    ros::Publisher desiredTorquePublisher      = nh.advertise<geometry_msgs::Point>
                                                 ("desired_moment", 10);
    
    ros::Publisher psiBiasPublisherNo1         = nh.advertise<std_msgs::Float64>
                                                 ("psi_bias_1", 10);
    ros::Publisher psiBiasPublisherNo2         = nh.advertise<std_msgs::Float64>
                                                 ("psi_bias_2", 10);
    ros::Publisher psiBiasPublisherNo3         = nh.advertise<std_msgs::Float64>
                                                 ("psi_bias_3", 10);

    ros::Publisher initialEulerAnglePublisher1 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_1", 10);
    ros::Publisher initialEulerAnglePublisher2 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_2", 10);
    ros::Publisher initialEulerAnglePublisher3 = nh.advertise<geometry_msgs::Point>
                                                 ("initial_euler_angle_3", 10);
    ros::Publisher controllerStartPublisher    = nh.advertise<std_msgs::Bool>
                                                 ("start_pub_att", 10);
    ros::Publisher modeSwitchPublisher         = nh.advertise<std_msgs::Bool>
                                                 ("mode_switch", 10);
    
    // defines ROS message variables
    geometry_msgs::Point desiredPositionMessage;
    geometry_msgs::Point desiredEulerAngleMessage;
    geometry_msgs::Point desiredVelocityMessage;
    geometry_msgs::Point desiredBodyRateMessage;
    geometry_msgs::Point desiredForceMessage;
    geometry_msgs::Point desiredTorqueMessage;
    std_msgs::Float64 psiBiasMessage1;
    std_msgs::Float64 psiBiasMessage2;
    std_msgs::Float64 psiBiasMessage3;
    geometry_msgs::Point initialEulerAngleMessage1;
    geometry_msgs::Point initialEulerAngleMessage2;
    geometry_msgs::Point initialEulerAngleMessage3;
    std_msgs::Bool controllerStartMessage;
    std_msgs::Bool modeSwitchMessage;

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
    double& forceX = desiredForceMessage.x;
    double& forceY = desiredForceMessage.y;
    double& forceZ = desiredForceMessage.z;
    double& torqueX = desiredTorqueMessage.x;
    double& torqueY = desiredTorqueMessage.y;
    double& torqueZ = desiredTorqueMessage.z;
    double& psiBias1 = psiBiasMessage1.data;
    double& psiBias2 = psiBiasMessage2.data;
    double& psiBias3 = psiBiasMessage3.data;
    double& initialEulerAngle1X = initialEulerAngleMessage1.x;
    double& initialEulerAngle1Y = initialEulerAngleMessage1.y;
    double& initialEulerAngle1Z = initialEulerAngleMessage1.z;
    double& initialEulerAngle2X = initialEulerAngleMessage2.x;
    double& initialEulerAngle2Y = initialEulerAngleMessage2.y;
    double& initialEulerAngle2Z = initialEulerAngleMessage2.z;
    double& initialEulerAngle3X = initialEulerAngleMessage3.x;
    double& initialEulerAngle3Y = initialEulerAngleMessage3.y;
    double& initialEulerAngle3Z = initialEulerAngleMessage3.z;
    auto& controllerStart = controllerStartMessage.data;
    auto& modeSwitch = modeSwitchMessage.data;
    
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
    psiBias1 = 0.0;
    psiBias2 = 2 * M_PI / 3;
    psiBias3 = -2 * M_PI / 3;
    controllerStart = true;

    double i = 0;
    positionZ = -0.1;
    while (ros::ok() && i <= 3)
    {
        initialEulerAnglePublisher1.publish(initialEulerAngleMessage1);
        initialEulerAnglePublisher2.publish(initialEulerAngleMessage2);
        initialEulerAnglePublisher3.publish(initialEulerAngleMessage3);

        psiBiasPublisherNo1.publish(psiBiasMessage1);
        psiBiasPublisherNo2.publish(psiBiasMessage2);
        psiBiasPublisherNo3.publish(psiBiasMessage3);

        controllerStartPublisher.publish(controllerStartMessage);

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
        positionZ -= 2.15 / rate / 10;
        velocityZ = -2.15 / 10;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // takes off

    i = 0;
    while (ros::ok() && i <= 5)
    {
        velocityZ = 0.0;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);
        
        i += 1 / rate;

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);
        
        ros::spinOnce();
        loopRate.sleep();
    } // holds

    i = 0;
    while (ros::ok() && i <= 10)
    {
        positionX -= 1.4 / rate / 10;
        velocityX = -1.4 / 10;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // approaches

    i = 0;
    while (ros::ok() && i <= 10)
    {
        positionX -= 0.3 / rate / 10;
        velocityX = -0.3 / 10;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // approaches

    i = 0;
    while (ros::ok() && i <= 5)
    {
        velocityX = 0.0;
        positionY -= 0.04 / rate / 10;
        velocityY = -0.04 / 10;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // halts

    i = 0;
    while (ros::ok() && i <= 3)
    {
        forceX = 1.5;
        forceY = 4.0;
        torqueZ = -4.0;
        positionX -= 0.2 / rate / 3;
        velocityX = -0.2 / 3;
        positionY += 0.04 / rate / 3;
        velocityY = 0.04 / 3;
        modeSwitch = true;
        
        ROS_INFO("Desired force: %f, %f, %f", forceX, forceY, forceZ);
        ROS_INFO("Desired torque: %f, %f, %f", torqueX, torqueY, torqueZ);

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);
        desiredBodyRatePublisher.publish(desiredBodyRateMessage);
        modeSwitchPublisher.publish(modeSwitchMessage);
        desiredForcePublisher.publish(desiredForceMessage);
        desiredTorquePublisher.publish(desiredTorqueMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // contacts

    i = 0;
    while (ros::ok())
    {
        forceX = 1.5;
        forceY = 4.0;
        torqueZ = -4.0;

        desiredPositionPublisher.publish(desiredPositionMessage);
        desiredEulerAnglePublisher.publish(desiredEulerAngleMessage);
        modeSwitchPublisher.publish(modeSwitchMessage);
        desiredForcePublisher.publish(desiredForceMessage);
        desiredTorquePublisher.publish(desiredTorqueMessage);

        // ROS_INFO("Desired position: %f, %f, %f", positionX, positionY, positionZ);
        // ROS_INFO("Desired Euler angle: %f, %f, %f", eulerAngleX, eulerAngleY, eulerAngleZ);

        i += 1 / rate;

        ros::spinOnce();
        loopRate.sleep();
    } // hold position
}
