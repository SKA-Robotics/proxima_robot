import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # flaga decydujaca o wlaczeniu tego mocka
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware', 
        default_value='false', 
        description='Użyj wbudowanego mocka sprzętowego (bez fizycznego podłączenia)'
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('proxima_description'), 'robots', 'dynamixel_test.urdf.xacro']), ' ',
        'use_mock_hardware:=', use_mock_hardware
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('proxima_description'), 'config', 'dynamixel_test.yaml'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ])