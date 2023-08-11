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
    px4_visualizer = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='visualizer',
        name='visualizer'
    )
    octomap_params = {'resolution': 1.0,
              'frame_id': 'map',
              'base_frame_id': 'map',
            #   'height_map': True,
            #   'colored_map': False,
            #   'color_factor': 0.8,
            #   'filter_ground': True,
            #   'filter_speckles': False,
            #   'compress_map': True,
            #   'incremental_2D_projection': False,
            #   'sensor_model/max_range': 9.0,
            #   'sensor_model/hit': 0.9,
            #   'sensor_model/miss': 0.45,
            #   'sensor_model/min': 0.01,
            #   'sensor_model/max': 0.99,
            #   'pointcloud_max_x': 100.0,
            #   'pointcloud_max_y': 100.0,
            #   'pointcloud_max_z': 100.0,
            #   'pointcloud_min_x': -100.0,
            #   'pointcloud_min_y': -100.0,
            #   'pointcloud_min_z': -100.0,
            #   'occupancy_min_z': 0.0,
            #   'color/r': 0.0,
            #   'color/g': 0.0,
            #   'color/b': 1.0,
            #   'color/a': 1.0,
            #   'color_free/r': 0.0,
            #   'color_free/g': 0.0,
            #   'color_free/b': 1.0,
            #   'color_free/a': 1.0,
            #   'publish_free_space': False,
    }
    return LaunchDescription([
        manipulation_node,
        px4_visualizer,
        rviz
])
