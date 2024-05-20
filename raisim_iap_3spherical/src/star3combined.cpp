// star3single.cpp
// AVA 3star raisim node
// Du Jianrui 2023.9.20

#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"

#include "../include/StatesSender.h"
#include "../include/CommandsReceiver.h"
#include "../include/Timer.hpp"

int main(int argc, char* argv[])
{
  // initializes ROS
  ros::init(argc, argv, "star3combined");
  ros::NodeHandle nh;

  double rate = 200.0;
  ros::Rate loopRate(rate);

  // creates a raisim world
  raisim::World world;
  double timeStep = 1.0 / rate;
  world.setTimeStep(timeStep);
  auto ground = world.addGround(0, "steel");
  ground->setAppearance("white");

  // sets RaiSim collision pairs
  world.setMaterialPairProp("steel", "copper", 0.8, 0.65, 0.001);
  world.setMaterialPairProp("steel", "rubber", 0.8, 0.15, 0.001);
  world.setMaterialPairProp("wood", "rubber", 0.45, 0.15, 0.001);
  world.setMaterialPairProp("rubber", "rubber", 0.4, 0.05, 0.001);

  // auto box1 = world.addBox(0.2, 0.2, 1, 100.0, "steel");
  // box1->setName("wall1");
  // box1->setPosition(-3, 0.25, 2.1);
  // box1->setBodyType(raisim::BodyType::STATIC);

  // auto box2 = world.addBox(0.2, 0.2, 1, 100.0, "steel");
  // box2->setName("wall2");
  // box2->setPosition(-3, -0.25, 2.1);
  // box2->setBodyType(raisim::BodyType::STATIC);

  // auto box3 = world.addBox(0.2, 0.2, 1, 100.0, "steel");
  // box3->setName("wall3");
  // box3->setPosition(-3.7, 0, 2.1);
  // box3->setBodyType(raisim::BodyType::STATIC);

  // auto box4 = world.addBox(0.2, 0.2, 1, 100.0, "steel");
  // box4->setName("wall4");
  // box4->setPosition(-3.3, 0, 1.67);
  // box4->setOrientation(0, -0.793, 0, 0.609);
  // box4->setBodyType(raisim::BodyType::STATIC);

  // auto board = world.addArticulatedSystem("\\home\\siris\\catkin_ws\\src\\raisim_star3_admittance\\rsc\\ava\\urdf\\board.urdf");
  // board->setName("board");
  // Eigen::VectorXd gc2(board->getGeneralizedCoordinateDim()), gv2(board->getDOF());
  // gc2.setZero(); gv2.setZero();
  // gc2 << -3, 0, 2.084, 0, -0.793, 0, 0.609;
  // board->setGeneralizedCoordinate(gc2);
  // board->setGeneralizedVelocity(gv2);

  // auto box1 = world.addBox(0.5, 0.5, 2, 100.0, "steel");
  // box1->setName("wall1");
  // box1->setPosition(-3.125, 0, 1);
  // box1->setBodyType(raisim::BodyType::STATIC);

  // auto box2 = world.addBox(0.125, 0.5, 0.1, 100.0, "steel");
  // box2->setName("wall2");
  // box2->setPosition(-2.9375, 0, 2.05);
  // box2->setBodyType(raisim::BodyType::STATIC);

  // auto box3 = world.addBox(0.125, 0.5, 0.1, 100.0, "steel");
  // box3->setName("wall3");
  // box3->setPosition(-3.3125, 0, 2.05);
  // box3->setBodyType(raisim::BodyType::STATIC);

  auto board = world.addArticulatedSystem("\\home\\siris\\catkin_ws\\src\\raisim_iap_3spherical\\rsc\\ava\\urdf\\board.urdf");
  board->setName("board");
  Eigen::VectorXd gc2(board->getGeneralizedCoordinateDim()), gv2(board->getDOF());
  gc2.setZero(); gv2.setZero();
  gc2 << -3, 0, 2.25, 0.707, 0, -0.707, 0;
  board->setGeneralizedCoordinate(gc2);
  board->setGeneralizedVelocity(gv2);

  // adds and initializes RaiSim robots
  std::string urdfPath;
  ros::param::get("~urdfPath", urdfPath);
  auto robot = world.addArticulatedSystem(urdfPath);
  std::string robotName;
  ros::param::get("~robotName", robotName);
  robot->setName(robotName);
  Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF());
  gc.setZero(); gv.setZero();
  gc << 0, 0, 1.0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0;
  robot->setGeneralizedCoordinate(gc);
  robot->setGeneralizedVelocity(gv);
  AvaRobotState states;
  states.toolIndex = robot->getBodyIdx("base_link");
  AvaRobotCommand commands;

  // launches the raisim server
  int raisimServerPort;
  ros::param::get("~raisimServerPort", raisimServerPort);
  raisim::RaisimServer server(&world);
  server.launchServer(raisimServerPort);
  RSINFO("RaiSim server started.")

  std::string rosbagPath;
  ros::param::get("~rosbagPath", rosbagPath);

  // launches the exoROS server
  std::string localHostIp;
  ros::param::get("~localHostIp", localHostIp);
  std::string dataPort;
  ros::param::get("~dataPort", dataPort);
  StatesSender publisher(nh, robot, &world, timeStep, localHostIp, dataPort);
  CommandsReceiver subscriber(nh, robot);
  Eigen::Vector3d originalEulerAngle1, originalEulerAngle2, originalEulerAngle3;
  Eigen::Vector3d relativePosition1, relativePosition2, relativePosition3;
  originalEulerAngle1 << 0.0, 0.0,        0.0;
  originalEulerAngle2 << 0.0, 0.0,  2.0943951;
  originalEulerAngle3 << 0.0, 0.0, -2.0943951;
  relativePosition1 <<   0.5525,            0.0, -0.132;
  relativePosition2 << -0.27625,  0.47847903559, -0.132;
  relativePosition3 << -0.27625, -0.47847903559, -0.132;
  publisher.setOriginalRotation(originalEulerAngle1, originalEulerAngle2, originalEulerAngle3);
  publisher.setRelativePosition(relativePosition1, relativePosition2, relativePosition3);
  subscriber.setOriginalRotation(originalEulerAngle1, originalEulerAngle2, originalEulerAngle3);
  subscriber.setRelativePosition(relativePosition1, relativePosition2, relativePosition3);
  
  publisher.openRosbag(rosbagPath);

  while(ros::ok())
  {
    server.integrateWorldThreadSafe();

    publisher.getAndSendStates(states);
    subscriber.receiveCommandsAndApply(commands, states);

    ros::spinOnce();
    loopRate.sleep();
  }

  server.killServer();
}
