import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_path = os.path.join(get_package_share_directory('ar3_description'),
                              'urdf', 'ar3.urdf.xacro')
    robot_description = {'robot_description' : Command(['xacro', ' ', xacro_path])}

    #robot_description_semantic_config = load_file('ar3_moveit_config', 'config/ar3.srdf')
    #robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    #kinematics_yaml = load_yaml('ar3_moveit_config', 'config/kinematics.yaml')
    #robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    ## Planning Functionality
    #ompl_planning_pipeline_config = { 'move_group' : {
    #    'planning_plugin' : 'ompl_interface/OMPLPlanner',
    #    'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    #    'start_state_max_bounds_error' : 0.1 } }
    #ompl_planning_yaml = load_yaml('ar3_moveit_config', 'config/ompl_planning.yaml')
    #ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    #controllers_yaml = load_yaml(os.path.join('ar3_moveit_config', 'config',
    #                                          'fake_controllers.yaml'))

    #moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
    #                       'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    #trajectory_execution = {'moveit_manage_controllers': True,
    #                        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
    #                        'trajectory_execution.allowed_goal_duration_margin': 0.5,
    #                        'trajectory_execution.allowed_start_tolerance': 0.01}
    #
    #planning_scene_monitor_parameters = {"publish_planning_scene": True,
    #             "publish_geometry_updates": True,
    #             "publish_state_updates": True,
    #             "publish_transforms_updates": True}
    #
    ## Start the actual move_group node/action server
    #run_move_group_node = Node(package='moveit_ros_move_group',
    #                           executable='move_group',
    #                           output='screen',
    #                           emulate_tty=True,
    #                           parameters=[robot_description,
    #                                       robot_description_semantic,
    #                                       kinematics_yaml,
    #                                       ompl_planning_pipeline_config,
    #                                       trajectory_execution,
    #                                       moveit_controllers,
    #                                       planning_scene_monitor_parameters])

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory('ar3_moveit_config'), 'rviz', 'demo.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])
                                 #robot_description_semantic,
                                 #ompl_planning_pipeline_config,
                                 #kinematics_yaml])

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

    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_ros2',
                                  executable='fake_joint_driver_node',
                                  output='screen',
                                  emulate_tty=True,
                                  parameters=[{'controller_name': 'ar3_arm_controller'},
                                              os.path.join(get_package_share_directory("ar3_moveit_config"), "config", "sim_start_positions.yaml"),
                                              os.path.join(get_package_share_directory("ar3_moveit_config"), "config", "ar3_controllers.yaml"),
                                              robot_description])

    return LaunchDescription([ rviz_node, static_tf, robot_state_publisher, fake_joint_driver_node ])
