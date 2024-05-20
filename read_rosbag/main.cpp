// main.cpp
// ROS node
// Du Jianrui   2023.12.01

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <vector>

int main(int argc, char** argv)
{
    //------------- ROS -------------
    ros::init(argc, argv, "read_rosbag");
    ros::NodeHandle nodeHandle;
    
    // information of bags and topics
    std::string sourceMotionBagPath = "/home/siris/catkin_ws/rosbags/slam_motion-2023121401.bag";
    // std::string sourceCaptureBagPath1 = "/home/siris/catkin_ws/rosbags/slam_capture01-2023120802.bag";
    // std::string sourceCaptureBagPath2 = "/home/siris/catkin_ws/rosbags/slam_capture02-2023120802.bag";
    // std::string sourceCaptureBagPath3 = "/home/siris/catkin_ws/rosbags/slam_capture03-2023120802.bag";
    // std::string sourceCaptureBagPath4 = "/home/siris/catkin_ws/rosbags/slam_capture04-2023120802.bag";
    // std::string sourceCaptureBagPath5 = "/home/siris/catkin_ws/rosbags/slam_capture05-2023120802.bag";
    // std::string sourceCaptureBagPath6 = "/home/siris/catkin_ws/rosbags/slam_capture06-2023120802.bag";
    std::string sourceCaptureBagPath7 = "/home/siris/catkin_ws/rosbags/slam_capture07-2023121401.bag";
    std::string sourceCaptureBagPath8 = "/home/siris/catkin_ws/rosbags/slam_capture08-2023121401.bag";
    std::string targetBagPath = "/home/siris/catkin_ws/rosbags/slam_complete-2023121401.bag";
    std::string imuTopic0 = "/imu_0";
    // std::string imuTopic1 = "/imu_1";
    // std::string imuTopic2 = "/imu_2";
    // std::string imuTopic3 = "/imu_3";
    std::string imuTopicRaw0 = "/imu_no_noise_0";
    // std::string imuTopicRaw1 = "/imu_no_noise_1";
    // std::string imuTopicRaw2 = "/imu_no_noise_2";
    // std::string imuTopicRaw3 = "/imu_no_noise_3";
    std::string poseTopic0 = "/pose_0";
    // std::string poseTopic1 = "/pose_1";
    // std::string poseTopic2 = "/pose_2";
    // std::string poseTopic3 = "/pose_3";
    // std::string imageTopic1 = "/camera1/image";
    // std::string imageTopic2 = "/camera2/image";
    // std::string imageTopic3 = "/camera3/image";
    // std::string imageTopic4 = "/camera4/image";
    // std::string imageTopic5 = "/camera5/image";
    // std::string imageTopic6 = "/camera6/image";
    std::string imageTopic7 = "/camera7/image";
    std::string imageTopic8 = "/camera8/image";

    // rosbags read/write initialization
    rosbag::Bag sourceMotionBag, targetBag,
                // sourceCaptureBag1, sourceCaptureBag2,
                // sourceCaptureBag3, sourceCaptureBag4,
                // sourceCaptureBag5, sourceCaptureBag6;
                sourceCaptureBag7, sourceCaptureBag8;
    sourceMotionBag.open(sourceMotionBagPath, rosbag::bagmode::Read);
    // sourceCaptureBag1.open(sourceCaptureBagPath1, rosbag::bagmode::Read);
    // sourceCaptureBag2.open(sourceCaptureBagPath2, rosbag::bagmode::Read);
    // sourceCaptureBag3.open(sourceCaptureBagPath3, rosbag::bagmode::Read);
    // sourceCaptureBag4.open(sourceCaptureBagPath4, rosbag::bagmode::Read);
    // sourceCaptureBag5.open(sourceCaptureBagPath5, rosbag::bagmode::Read);
    // sourceCaptureBag6.open(sourceCaptureBagPath6, rosbag::bagmode::Read);
    sourceCaptureBag7.open(sourceCaptureBagPath7, rosbag::bagmode::Read);
    sourceCaptureBag8.open(sourceCaptureBagPath8, rosbag::bagmode::Read);
    targetBag.open(targetBagPath, rosbag::bagmode::Write);
    std::vector<std::string> topicsMotion,
                            //  topicsCapture1, topicsCapture2,
                            //  topicsCapture3, topicsCapture4,
                            //  topicsCapture5, topicsCapture6;
                             topicsCapture7, topicsCapture8;
    topicsMotion.push_back(imuTopic0);
    // topicsMotion.push_back(imuTopic1);
    // topicsMotion.push_back(imuTopic2);
    // topicsMotion.push_back(imuTopic3);
    topicsMotion.push_back(imuTopicRaw0);
    // topicsMotion.push_back(imuTopicRaw1);
    // topicsMotion.push_back(imuTopicRaw2);
    // topicsMotion.push_back(imuTopicRaw3);
    topicsMotion.push_back(poseTopic0);
    // topicsMotion.push_back(poseTopic1);
    // topicsMotion.push_back(poseTopic2);
    // topicsMotion.push_back(poseTopic3);
    // topicsCapture1.push_back(imageTopic1);
    // topicsCapture2.push_back(imageTopic2);
    // topicsCapture3.push_back(imageTopic3);
    // topicsCapture4.push_back(imageTopic4);
    // topicsCapture5.push_back(imageTopic5);
    // topicsCapture6.push_back(imageTopic6);
    topicsCapture7.push_back(imageTopic7);
    topicsCapture8.push_back(imageTopic8);
    rosbag::View viewMotion(sourceMotionBag, rosbag::TopicQuery(topicsMotion));
    // rosbag::View viewCapture1(sourceCaptureBag1, rosbag::TopicQuery(topicsCapture1));
    // rosbag::View viewCapture2(sourceCaptureBag2, rosbag::TopicQuery(topicsCapture2));
    // rosbag::View viewCapture3(sourceCaptureBag3, rosbag::TopicQuery(topicsCapture3));
    // rosbag::View viewCapture4(sourceCaptureBag4, rosbag::TopicQuery(topicsCapture4));
    // rosbag::View viewCapture5(sourceCaptureBag5, rosbag::TopicQuery(topicsCapture5));
    // rosbag::View viewCapture6(sourceCaptureBag6, rosbag::TopicQuery(topicsCapture6));
    rosbag::View viewCapture7(sourceCaptureBag7, rosbag::TopicQuery(topicsCapture7));
    rosbag::View viewCapture8(sourceCaptureBag8, rosbag::TopicQuery(topicsCapture8));

    for (auto m : viewMotion)
    {
        sensor_msgs::Imu::ConstPtr imuPtr = m.instantiate<sensor_msgs::Imu>();
        if (imuPtr != nullptr)
        {
            if (m.getTopic() == imuTopic0)
            {
                targetBag.write(imuTopic0, imuPtr->header.stamp, imuPtr);
            }
            // else if (m.getTopic() == imuTopic1)
            // {
            //     targetBag.write(imuTopic1, imuPtr->header.stamp, imuPtr);
            // }
            // else if (m.getTopic() == imuTopic2)
            // {
            //     targetBag.write(imuTopic2, imuPtr->header.stamp, imuPtr);
            // }
            // else if (m.getTopic() == imuTopic3)
            // {
            //     targetBag.write(imuTopic3, imuPtr->header.stamp, imuPtr);
            // }
            else if (m.getTopic() == imuTopicRaw0)
            {
                targetBag.write(imuTopicRaw0, imuPtr->header.stamp, imuPtr);
            }
            // else if (m.getTopic() == imuTopicRaw1)
            // {
            //     targetBag.write(imuTopicRaw1, imuPtr->header.stamp, imuPtr);
            // }
            // else if (m.getTopic() == imuTopicRaw2)
            // {
            //     targetBag.write(imuTopicRaw2, imuPtr->header.stamp, imuPtr);
            // }
            // else if (m.getTopic() == imuTopicRaw3)
            // {
            //     targetBag.write(imuTopicRaw3, imuPtr->header.stamp, imuPtr);
            // }
        }
        
        nav_msgs::Odometry::ConstPtr posePtr = m.instantiate<nav_msgs::Odometry>();
        if (posePtr != nullptr)
        {
            if (m.getTopic() == poseTopic0)
            {
                targetBag.write(poseTopic0, posePtr->header.stamp, posePtr);
            }
            // else if (m.getTopic() == poseTopic1)
            // {
            //     targetBag.write(poseTopic1, posePtr->header.stamp, posePtr);
            // }
            // else if (m.getTopic() == poseTopic2)
            // {
            //     targetBag.write(poseTopic2, posePtr->header.stamp, posePtr);
            // }
            // else if (m.getTopic() == poseTopic3)
            // {
            //     targetBag.write(poseTopic3, posePtr->header.stamp, posePtr);
            // }
        }
    }

    // for (auto m : viewCapture1)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic1, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    // for (auto m : viewCapture2)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic2, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    // for (auto m : viewCapture3)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic3, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    // for (auto m : viewCapture4)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic4, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    // for (auto m : viewCapture5)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic5, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    // for (auto m : viewCapture6)
    // {
    //     sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
    //     if (imagePtr != nullptr)
    //     {
    //         targetBag.write(imageTopic6, imagePtr->header.stamp, imagePtr);
    //     }
    // }
    for (auto m : viewCapture7)
    {
        sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
        if (imagePtr != nullptr)
        {
            targetBag.write(imageTopic7, imagePtr->header.stamp, imagePtr);
        }
    }
    for (auto m : viewCapture8)
    {
        sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
        if (imagePtr != nullptr)
        {
            targetBag.write(imageTopic8, imagePtr->header.stamp, imagePtr);
        }
    }

    ROS_INFO("Done!");
}
