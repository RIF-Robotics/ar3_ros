# AR3 ROS2 Packages

Contains the ROS2 packages for the AR3 robot arm

## Build

    $ mkdir -p ~/ros2/ar3_ws/src
    $ cd ~/ros2/ar3_ws/src
    $ git clone https://github.com/RIF-Robotics/ar3_ros.git
    $ cd ..
    $ source /opt/ros/foxy/setup.bash
    $ colcon build --symlink-install

## Visualize AR3 in rviz

    $ cd ~/ros2/ar3_ws
    $ source ./install/setup.bash
    $ ros2 launch ar3_bringup rviz.launch.py

## Simple simulation of AR3

The `use_fake_hardware` flag uses a simple software loop back, which won't
communicate with the real AR3 robot arm.

    $ ros2 launch ar3_bringup ar3_base.launch.py use_fake_hardware:=True

## Connect ROS2 to Physical AR3

    $ ros2 launch ar3_bringup ar3_base.launch.py

## MoveIt2 Control

See the
[ar3_moveit2_config](https://github.com/RIF-Robotics/ar3_moveit2_config)
package for how to use MoveIt2 to control the AR3.
