# IapSimulatorROS
 ROS nodes of the IAP (Integrated Aerial Platform) simulator

## Simulation Nodes

- **raisim_iap_3spherical**: This node is for the three-vehicle IAP simulation using the RaiSim physics engine.
- **raisim_line2**: This node is for the two-vehicle IAP simulation using the RaiSim physics engine.

## Test Input Nodes

- **test_***: These nodes input test trajectories for various configurations.
- **test_*_replay**: These nodes are for replaying trajectories using the ROS bags.
- **joystick_input**: This node allows trajectory input using a game controller.

## Position Forwarding Node

- **vins_position_forward**: This node forwards the online VINS position estimates.


## Dependencies

- **ZeroMQ**: An open-source communication framework. Please follow the instructions on its official website for installation.
- **RaiSim**: A robotics simulation engine. It requires filling out an application on its official website and using it with a license.
