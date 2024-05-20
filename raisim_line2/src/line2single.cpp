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
  ros::init(argc, argv, "line2single");
  ros::NodeHandle nh;

  double rate = 200.0;
  ros::Rate loopRate(rate);
  
  // initializes RaiSim
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\..\\rsc\\activation.raisim");
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  // creates a raisim world
  raisim::World world;
  double timeStep = 1.0 / rate;
  world.setTimeStep(timeStep);
  auto ground = world.addGround(0, "steel");
  ground->setAppearance("white");

  // sets RaiSim collision pairs
  world.setMaterialPairProp("steel", "copper", 0.8, 0.65, 0.001);

  // adds and initializes RaiSim robots
  auto robot = world.addArticulatedSystem("\\home\\siris\\catkin_ws\\src\\raisim_line2\\rsc\\ava\\urdf\\AVA3.urdf");
  std::string robotName;
  ros::param::get("~robotName", robotName);
  robot->setName(robotName);
  Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF());
  gc.setZero(); gv.setZero();
  gc << 0, 0, 1.0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0;
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
