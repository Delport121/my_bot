import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
import yaml
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'my_bot'  # <--- CHANGE ME
    
    # Include robot state publisher
    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Set parameters for twist_mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml') 
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Include joystick configuration
    joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'mco.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'Test_square.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'Test_square_with_base.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'Smooth_curve.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'Corridor_with_room.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'warehouse_obstacles')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', '2D_Test_square.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', '3D_Test_square.world')
    custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', '3D_Test_square_complex.world')
    # custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
    gazebo_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'params_file': gazebo_params_path, 'world': custom_world_path}.items()
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    
    # Define controller spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    # RViz node - start immediately
    rviz_config_path = os.path.join(get_package_share_directory('my_bot'), 'config', 'lidar_3d.rviz')
    print(rviz_config_path)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Map server node with delayed start to allow RViz to initialize first
    config_path = os.path.join(get_package_share_directory('my_bot'), 'config', 'sim.yaml')
    with open(config_path, 'r') as file:
        config_dict = yaml.safe_load(file)
        
    map_server_node = TimerAction(
        period=3.0,  # Delay to ensure RViz is fully initialized
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': True},
                    {'topic_latching': True}  # Enable latching
                ]
            )
        ]
    )

    # Lifecycle manager node with delayed start to match map_server
    nav_lifecycle_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}
                ]
            )
        ]
    )

    # Static transform publisher from map to base_link or odom - no delay needed
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], # <--- CHANGE ME <base_link>
        # arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'], # <--- CHANGE ME <base_link>
        output='screen'
    )
    
    # Launch all components
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz_node,
        # map_server_node,
        # nav_lifecycle_node,
        static_tf_node
    ])
