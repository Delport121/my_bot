from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    my_bot_namespace = 'my_bot'
    lidarbot_namespace = 'lidarbot'
    
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_lidarbot"), "rviz", "rviz.rviz"]
    )
    
    
    declared_arguments = []
    

            # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_bot"), "description", "robot_and_lidar.urdf.xacro"]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    
    
    combined_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # namespace=lidarbot_namespace,
        parameters=[robot_description],
    )
    
    # Set parameters for twist_mux
    package_name = 'my_bot'
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml') 
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )
    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                FindPackageShare('gazebo_ros').find('gazebo_ros') + '/launch/gazebo.launch.py'
            )
    )
    
    # Spawn Both Robots
    combined_gazebo_spawner_node = Node(
        package = "gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_my_bot",
        output="screen",
        arguments=["-topic", "/robot_description", 
                   "-entity", "husky_lidar"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
        
    # Define controller spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],

    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    nodes = [
       # lidarbot_state_publisher_node,
       # my_bot_state_publisher_node,
        combined_state_publisher_node,
        twist_mux,
        gazebo,
       # lidarbot_gazebo_spawner_node,
       # my_bot_gazebo_spawner_node,
        combined_gazebo_spawner_node,
        diff_drive_spawner,
        joint_broad_spawner,
        position_controller_spawner,
        velocity_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments+nodes)