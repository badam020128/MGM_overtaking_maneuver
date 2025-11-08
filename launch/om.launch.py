#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, HyunGyu Kim

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import OpaqueFunction


def spawn_robot_cmd(context, *args, **kwargs):
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'tmp'
    )
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='test')
    ns_name = namespace.perform(context)
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'frame_prefix': namespace}.items()
    )

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for odom_frame_tag in root.iter('odometry_frame'):
        odom_frame_tag.text = f'{ns_name}/odom'
    for base_frame_tag in root.iter('robot_base_frame'):
        base_frame_tag.text = f'{ns_name}/base_footprint'
    for scan_frame_tag in root.iter('frame_name'):
        scan_frame_tag.text = f'{ns_name}/base_scan'
    urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
    urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
    with open(f'{save_path}_{ns_name}.sdf', 'w') as file:
        file.write(urdf_modified)

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'multi_spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'robot_name': f'{TURTLEBOT3_MODEL}_{ns_name}',
                'namespace': f'{ns_name}',
                'sdf_path': f'{save_path}_{ns_name}.sdf'
        }.items()
    )

    return [
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event,
                context: [os.remove(f'{save_path}_{ns_name}.sdf')]
            )
        ),

        GroupAction([PushRosNamespace(namespace),
                                    robot_state_publisher_cmd,
                                    spawn_turtlebot_cmd])

    ]
        
def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function = spawn_robot_cmd))

    return ld
