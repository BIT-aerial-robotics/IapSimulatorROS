# IapSimulatorROS
 ROS nodes of the IAP (Integrated Aerial Platform) simulator

## Simulation Nodes

- **raisim_iap_3spherical**: This node is for the three-vehicle IAP simulation using the RaiSim physics engine.
- **raisim_line2**: This node is for the two-vehicle IAP simulation using the RaiSim physics engine.

## Test Input Nodes

- **test_***: These nodes are used for inputting test trajectories for various configurations.
- **test_*_replay**: This node is for replaying trajectories using the ROS bags.
- **joystick_input**: This node allows trajectory input using a game controller.

## Position Forwarding Node

- **vins_position_forward**: This node forwards the online VINS position estimates.
