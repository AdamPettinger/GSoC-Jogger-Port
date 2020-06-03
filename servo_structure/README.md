# servo_structure
This package serves to prototype the structure of the moveit_servo package (formerly moveit_jog_arm) for development in ROS2. It will be used to set up a package with the structure of the coming moveit_servo package, but with much simpler nodes. The goal is to nail down some of the communication layout before needing to deal with control math etc:

1) Composable node support that allows launching as part of a larger composable node, its own composable node, or a group of standalone nodes
2) Zero-copy message passing between nodes
3) Maintain the correct number of publishers/subscribers and their connections

# Executables
## ROS1 Executables and Libraries
Library `jog_arm_cpp_api` containing:

1) `moveit_jog_arm::CollisionCheck`
2) `moveit_jog_arm::JogCalcs`
3) `moveit_jog_arm::JogArm`
4) `moveit_jog_arm::LowPassFilter`

Executable `jog_server` (excluding example executables)

Launch files (examples):

1) `cpp_interface_example.launch` Using zero-message copying via the JogArm class and C++ interface
2) `spacenav_cpp.launch` Using another C++ node to jog using a SpaceNavigator mouse and communicating via slow ROS methods
2) `spacenav_teleop_tools.launch` Using a python node to jog with a SpaceNavigator mouse and communicating via slow ROS methods

## Proposed ROS2 Executables and Libraries
Libraries (for use with composable nodes):

1) `moveit_servo::CollisionCheck`
2) `moveit_servo::JogCalcs`
3) `moveit_servo::LowPassFilter` (maybe)
4) `moveit_servo::JogArm` (probably) (likely simple and just talks to collision and calcs)

Executables:

1) standalone `moveit_servo::CollisionCheck`
2) standalone `moveit_servo::JogCalcs`
2) `moveit_servo::JogArm` that combines the above 2, and represents *manual composition* of a composable node

Launch Files:
1) standalone `moveit_servo::CollisionCheck` Just launches the cooresponding executable
2) standalone `moveit_servo::JogCalcs` Just launches the cooresponding executable
3) `moveit_servo::JogArm` Launches the manually composed node executable
4) composed_launch that uses the launch file compisition scheme