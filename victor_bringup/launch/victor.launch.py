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

"""
This launch file uses python because the order of launch matters, and some nodes need to be delayed.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='victor_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='victor_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='victor_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='victor.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='position_trajectory_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Robot command interface [position|velocity|effort].',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    command_interface = LaunchConfiguration('command_interface')
    namespace = LaunchConfiguration('namespace')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'command_interface:=',
            command_interface,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Running with Moveit2 planning
    victor_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('victor_bringup'),
            '/launch',
            '/victor_planning.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'start_rviz': start_rviz,
            'namespace': namespace,
            'use_sim': use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )

    # Running with Moveit2 servoing
    victor_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('victor_bringup'),
            '/launch',
            '/victor_servoing.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'victor.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
        ],
        condition=UnlessCondition(use_planning),
    )
    victor_simulation_world = PathJoinSubstitution(
        [FindPackageShare(description_package),
            'gazebo/worlds', 'empty.world']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'false', 'world': victor_simulation_world}.items(),
        condition=IfCondition(use_sim),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', [namespace, 'robot_description'], '-entity', [namespace, 'victor14']],
        output='screen',
        condition=IfCondition(use_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        control_node,
        victor_planning_launch,
        victor_servoing_launch,
        spawn_entity,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
