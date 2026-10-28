import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'proxima_description'

    xacro_file_name = 'proxima.urdf.xacro'

    pkg_path = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_path, 'robots', xacro_file_name)

    manipulator_version_arg = DeclareLaunchArgument(
        'manipulator', default_value='none', description='Manipulator version'
    )

    manipulator_only_arg = DeclareLaunchArgument(
        'manipulator_only', default_value='false', description='Only include manipulator, not base'
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_path,
                ' manipulator:=',
                LaunchConfiguration('manipulator'),
                ' manipulator_only:=',
                LaunchConfiguration('manipulator_only'),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )

    return LaunchDescription(
        [
            manipulator_version_arg,
            manipulator_only_arg,
            robot_state_publisher_node,
            # joint_state_publisher_node
        ]
    )
