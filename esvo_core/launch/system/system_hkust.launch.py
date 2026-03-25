#!/usr/bin/env python3
"""
ROS2 launch file for ESVO system with HKUST dataset configuration.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    esvo_core_share = get_package_share_directory('esvo_core')

    # Launch arguments
    calib_dir_arg = DeclareLaunchArgument(
        'calib_dir',
        default_value=os.path.join(esvo_core_share, 'calib', 'hkust'),
        description='Calibration directory path'
    )

    sync_rate_arg = DeclareLaunchArgument(
        'sync_rate_hz',
        default_value='50.0',
        description='Sync timer rate in Hz'
    )

    # Time surface parameters
    ts_params_file = os.path.join(esvo_core_share, 'cfg', 'time_surface', 'ts_parameters.yaml')

    # Time Surface - Left
    time_surface_left = Node(
        package='esvo_time_surface',
        executable='esvo_time_surface',
        name='TimeSurface_left',
        remappings=[
            ('events', '/davis/left/events'),
            ('image', '/davis/left/image_raw'),
            ('camera_info', '/davis/left/camera_info'),
            ('time_surface', '/TS_left'),
        ],
        parameters=[ts_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # Time Surface - Right
    time_surface_right = Node(
        package='esvo_time_surface',
        executable='esvo_time_surface',
        name='TimeSurface_right',
        remappings=[
            ('events', '/davis/right/events'),
            ('image', '/davis/right/image_raw'),
            ('camera_info', '/davis/right/camera_info'),
            ('time_surface', '/TS_right'),
        ],
        parameters=[ts_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # Sync timer (replaces rostopic pub)
    sync_timer = Node(
        package='esvo_core',
        executable='sync_timer_node.py',
        name='sync_timer',
        parameters=[
            {'use_sim_time': True},
            {'rate_hz': LaunchConfiguration('sync_rate_hz')},
        ],
        output='screen'
    )

    # Mapping node
    mapping_params_file = os.path.join(esvo_core_share, 'cfg', 'mapping', 'mapping_hkust.yaml')
    esvo_mapping = Node(
        package='esvo_core',
        executable='esvo_Mapping',
        name='esvo_Mapping',
        remappings=[
            ('time_surface_left', '/TS_left'),
            ('time_surface_right', '/TS_right'),
            ('stamped_pose', '/esvo_tracking/pose_pub'),
            ('events_left', '/davis/left/events'),
            ('events_right', '/davis/right/events'),
        ],
        parameters=[
            mapping_params_file,
            {
                'use_sim_time': True,
                'dvs_frame_id': 'dvs',
                'world_frame_id': 'map',
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # Tracking node
    tracking_params_file = os.path.join(esvo_core_share, 'cfg', 'tracking', 'tracking_hkust.yaml')
    esvo_tracking = Node(
        package='esvo_core',
        executable='esvo_Tracking',
        name='esvo_Tracking',
        remappings=[
            ('time_surface_left', '/TS_left'),
            ('time_surface_right', '/TS_right'),
            ('stamped_pose', '/esvo_tracking/pose_pub'),
            ('gt_pose', '/optitrack/davis_stereo'),
            ('events_left', '/davis/left/events'),
            ('pointcloud', '/esvo_mapping/pointcloud_local'),
        ],
        parameters=[
            tracking_params_file,
            {
                'use_sim_time': True,
                'dvs_frame_id': 'dvs',
                'world_frame_id': 'map',
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # RViz
    rviz_config = os.path.join(esvo_core_share, 'esvo_system.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        calib_dir_arg,
        sync_rate_arg,
        # Nodes
        time_surface_left,
        time_surface_right,
        sync_timer,
        esvo_mapping,
        esvo_tracking,
        rviz_node,
    ])
