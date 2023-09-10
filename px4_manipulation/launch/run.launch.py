from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    manipulation_node = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='talker',
        name='sim'
    )
    package_dir = get_package_share_directory('px4_offboard')
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    )
    targetpose_marker = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='rviz_targetpose_marker.py',
        name='targetpose_marker'
    )
    px4_visualizer = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='visualizer',
        name='visualizer'
    )
    return LaunchDescription([
        manipulation_node,
        px4_visualizer,
        targetpose_marker,
        rviz
])
