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
  ros::init(argc, argv, "star3single");
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
