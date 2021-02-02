import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command

def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # TODO Get use_sim_time working with everything
    #use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Find and parse URDF xacro
    urdf_xacro_path = os.path.join(get_package_share_directory('ar3_description'),
                                   'urdf', 'ar3.urdf.xacro')
    robot_description = {'robot_description' : Command(['xacro', ' ', urdf_xacro_path])}

    ## RViz
    rviz_config_file = os.path.join(get_package_share_directory('ar3_moveit_config'), 'rviz', 'direct_control.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'ar3_link0'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    # Run the joint_trajectory_controller and joint_state_controller in the
    # controller manager node.
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,
                    os.path.join(get_package_share_directory("ar3_moveit_config"), "config", "controller_manager.yaml")],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Use a ROS service to start the joint_trajectory_controller in the controller manager
    #   joint_trajectory_controller:
    #     Subscribes to: /joint_trajectory_controller/joint_trajectory
    #     Publishes to: /state
    start_trajectory_controller = ExecuteProcess(
        cmd=['ros2 service call controller_manager/load_and_start_controller controller_manager_msgs/srv/LoadStartController \'{name: "joint_trajectory_controller"}\''],
        name='start_controller',
        shell=True
    )

    # Use a ROS service to start the joint_trajectory_controller in the controller manager
    #   joint_state_controller:
    #     Publishes to: /dynamic_joint_states
    #     Publishes to: /joint_states
    start_state_controller = ExecuteProcess(
        cmd=['ros2 service call controller_manager/load_and_start_controller controller_manager_msgs/srv/LoadStartController \'{name: "joint_state_controller"}\''],
        name='start_controller',
        shell=True
    )

    return LaunchDescription([
        controller_manager_node, start_trajectory_controller, start_state_controller,
        rviz_node, static_tf, robot_state_publisher])
