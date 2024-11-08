import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Load the YAML config path
    config_path = os.path.join(get_package_share_directory('my_bot'), 'config', 'sim.yaml')
    # Load the YAML configuration file
    with open(config_path, 'r') as file:
        config_dict = yaml.safe_load(file)

    # RViz node - start immediately
    rviz_config_path = os.path.join(get_package_share_directory('my_bot'), 'config', 'main.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Map server node with delayed start to allow RViz to initialize first
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
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        static_tf_node,
        map_server_node,
        nav_lifecycle_node
    ])
