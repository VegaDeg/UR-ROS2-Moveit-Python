# UR-ROS2-Moveit-Python

This is an example repository on how to control a Universal Robot with ROS2 and Moveit, using Python. 
Note that ROS2 are still under heavy development with new features releasing, th example in this repository may change to different frameworks or apis overtime.

## Introduction
Current workflow is composed of the following parts:

* [**Universal Robots ROS2 Driver**](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver): this driver manages the basic communication between ROS2 and UR robot
* [**Moveit2**](https://github.com/ros-planning/moveit2): takes care of trajectories planning.
* [**UR-ROS2-Moveit-Python**](https://github.com/VegaDeg/UR-ROS2-Moveit-Python): this package contains a node to receive target TCP pose over ROS2 topic and control robot through Move Group C++ Interface. 


## Preliminaries
This package is tested on the following platform:
* Ubuntu 22.04 Jammy Jellyfish
* ROS2 Rolling
* Moveit2
* Universal Robots ROS2 Driver

## Before using

First start UR ROS2 driver with:

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.102 launch_rviz:=false 
```

Then, start the external control urcap on the UR to allow external control.

Next, start moveit with:
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```
Finally, start the 
Start the hello_moveit_ur file with a launchfile 

```
ros2 launch hello_moveit_ur hello_moveit_ur_launch.py.
```

## Note
Currently we used Move Group C++ Interface because Moveit2 Python Api is still under development. To avoid confusion, this example may be switched to Moveit2 Python Api in the future.

## References
Code, examples and debugging information:
* [C++ with subscription](https://github.com/AndrejOrsula/ign_moveit2_examples/blob/master/examples/cpp/ex_follow_target.cpp)
* [simple_moveit2_universal_robots_movement](https://github.com/LucaBross/simple_moveit2_universal_robots_movement/tree/main)