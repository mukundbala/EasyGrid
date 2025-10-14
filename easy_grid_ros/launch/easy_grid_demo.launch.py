import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the YAML file
    config = os.path.join(
        get_package_share_directory('easy_grid_ros'),
        'config',
        'demo.yaml'
    )

    rviz_config = os.path.join(get_package_share_directory('easy_grid_ros'), 'rviz', 'demo.rviz')

    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_pub",
        arguments=['2.0', '2.0', '0', '0', '0', '0', 'world', 'map'],
        output='both'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([static_tf_publisher,rviz_node])
