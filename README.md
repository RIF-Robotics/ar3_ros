# AR3 ROS2 Packages

Contains the ROS2 packages for the AR3 robot arm

## Build

    $ mkdir -p ~/ros2/ar3_ws/src
    $ cd ~/ros2/ar3_ws/src
    $ git clone https://github.com/RIF-Robotics/ar3_ros.git
    $ cd ..
    $ source /opt/ros/foxy/setup.bash
    $ colcon build --symlink-install

## Run Simple Demo

    $ cd ~/ros2/ar3_ws
    $ source ./install/setup.bash
    $ ros2 launch ar3_moveit_config sim_demo.launch.py
