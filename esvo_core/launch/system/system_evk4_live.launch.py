#!/usr/bin/env python3

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
        default_value=os.path.join(esvo_core_share, 'calib', 'evk4'),
        description='Calibration directory path')

    sync_rate_arg = DeclareLaunchArgument('sync_rate_hz',
                                          default_value='50.0',
                                          description='Sync timer rate in Hz')

    # Time surface parameters
    ts_params_file = os.path.join(esvo_core_share, 'cfg', 'time_surface',
                                  'ts_parameters.yaml')

    # Camera info publisher - reads from calibration YAML files
    camera_info_publisher = Node(package='esvo_time_surface',
                                 executable='camera_info_publisher.py',
                                 name='camera_info_publisher',
                                 parameters=[
                                     {
                                         'use_sim_time': False
                                     },
                                     {
                                         'calib_dir':
                                         LaunchConfiguration('calib_dir')
                                     },
                                     {
                                         'frame_id': 'dvs'
                                     },
                                     {
                                         'publish_rate': 100.0
                                     },
                                     {
                                         'left_topic': '/evk/left/camera_info'
                                     },
                                     {
                                         'right_topic':
                                         '/evk/right/camera_info'
                                     },
                                 ],
                                 output='screen')

    # Time Surface - Left
    time_surface_left = Node(
        package='esvo_time_surface',
        executable='esvo_time_surface',
        name='TimeSurface_left',
        remappings=[
            ('events', '/evk/left/events'),
            ('image', '/evk/left/image_raw'),
            ('camera_info', '/evk/left/camera_info'),
            ('time_surface', '/TS_left'),
        ],
        parameters=[ts_params_file, {
            'use_sim_time': False
        }],
        output='screen')

    # Time Surface - Right
    time_surface_right = Node(
        package='esvo_time_surface',
        executable='esvo_time_surface',
        name='TimeSurface_right',
        remappings=[
            ('events', '/evk/right/events'),
            ('image', '/evk/right/image_raw'),
            ('camera_info', '/evk/right/camera_info'),
            ('time_surface', '/TS_right'),
        ],
        parameters=[ts_params_file, {
            'use_sim_time': False
        }],
        output='screen')

    # Sync timer (replaces rostopic pub)
    sync_timer = Node(package='esvo_core',
                      executable='sync_timer_node.py',
                      name='sync_timer',
                      parameters=[
                          {
                              'use_sim_time': False
                          },
                          {
                              'rate_hz': LaunchConfiguration('sync_rate_hz')
                          },
                      ],
                      output='screen')

    # Mapping node
    mapping_params_file = os.path.join(esvo_core_share, 'cfg', 'mapping',
                                       'mapping_evk4.yaml')
    esvo_mapping = Node(package='esvo_core',
                        executable='esvo_Mapping',
                        name='esvo_Mapping',
                        remappings=[
                            ('time_surface_left', '/TS_left'),
                            ('time_surface_right', '/TS_right'),
                            ('stamped_pose', '/esvo_tracking/pose_pub'),
                            ('events_left', '/evk/left/events'),
                            ('events_right', '/evk/right/events'),
                        ],
                        parameters=[
                            mapping_params_file, {
                                'use_sim_time': False,
                                'dvs_frame_id': 'dvs',
                                'world_frame_id': 'map',
                                'calibInfoDir':
                                LaunchConfiguration('calib_dir'),
                            }
                        ],
                        output='screen')

    # Tracking node
    tracking_params_file = os.path.join(esvo_core_share, 'cfg', 'tracking',
                                        'tracking_evk4.yaml')
    esvo_tracking = Node(package='esvo_core',
                         executable='esvo_Tracking',
                         name='esvo_Tracking',
                         remappings=[
                             ('time_surface_left', '/TS_left'),
                             ('time_surface_right', '/TS_right'),
                             ('stamped_pose', '/esvo_tracking/pose_pub'),
                             ('gt_pose', '/optitrack/davis_stereo'),
                             ('events_left', '/evk/left/events'),
                             ('pointcloud', '/esvo_mapping/pointcloud_local'),
                         ],
                         parameters=[
                             tracking_params_file, {
                                 'use_sim_time': False,
                                 'dvs_frame_id': 'dvs',
                                 'world_frame_id': 'map',
                                 'calibInfoDir':
                                 LaunchConfiguration('calib_dir'),
                             }
                         ],
                         output='screen')

    # RViz
    rviz_config = os.path.join(esvo_core_share, 'esvo_system.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     arguments=['-d', rviz_config],
                     parameters=[{
                         'use_sim_time': False
                     }],
                     output='screen')

    return LaunchDescription([
        # Arguments
        calib_dir_arg,
        sync_rate_arg,
        # Nodes
        camera_info_publisher,
        time_surface_left,
        time_surface_right,
        sync_timer,
        esvo_mapping,
        esvo_tracking,
        rviz_node,
    ])
