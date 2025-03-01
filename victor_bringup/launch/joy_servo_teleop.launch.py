# Copyright 2022 ICube Laboratory, University of Strasbourg
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch the main launcher with servo

    victor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/victor.launch.py']),
        launch_arguments={
            'command_interface': 'position',
            'robot_controller': 'victor_arm_controller',
            'use_servoing': 'true',
        }.items(),
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name='moveit_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='victor_moveit2',
                plugin='victor_servo::JoyToServoPub',
                name='controller_to_servo_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container, victor_launch, ])
