"""Test połączenia z trzema silnikami moteus przez ros2_control.

Użycie:
  ros2 launch proxima_description moteus_test.launch.py
  ros2 launch proxima_description moteus_test.launch.py use_mock_hardware:=true
  ros2 launch proxima_description moteus_test.launch.py can_id_1:=11 can_id_2:=12 can_id_3:=13
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'proxima_description'

    declared_args = [
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='true = mock_components/GenericSystem zamiast prawdziwych silników'),
        DeclareLaunchArgument('can_id_1', default_value='1', description='CAN ID silnika joint1'),
        DeclareLaunchArgument('can_id_2', default_value='2', description='CAN ID silnika joint2'),
        DeclareLaunchArgument('can_id_3', default_value='3', description='CAN ID silnika joint3'),
        DeclareLaunchArgument(
            'effort_max', default_value='15', description='Maksymalny moment [Nm] (wszystkie przeguby)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Uruchom RViz'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare(pkg), 'robots', 'moteus_test.urdf.xacro']),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' can_id_1:=', LaunchConfiguration('can_id_1'),
        ' can_id_2:=', LaunchConfiguration('can_id_2'),
        ' can_id_3:=', LaunchConfiguration('can_id_3'),
        ' effort_max:=', LaunchConfiguration('effort_max'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controllers_file = PathJoinSubstitution([FindPackageShare(pkg), 'config', 'moteus_test_controllers.yaml'])

    # robot_state_publisher publikuje /robot_description, z którego korzysta controller_manager
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='both',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # RViz startuje dopiero, gdy broadcaster działa (czyli sprzęt przeszedł configure/activate)
    rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz],
        )
    )

    return LaunchDescription(declared_args + [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        rviz_after_jsb,
    ])
