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
    default_rviz_config = os.path.join(pkg_path, 'config', 'manip_visualization.rviz')

    manipulator_version_arg = DeclareLaunchArgument(
        'manipulator', default_value='none', description='Manipulator version'
    )

    manipulator_only_arg = DeclareLaunchArgument(
        'manipulator_only', default_value='false', description='Only include manipulator, not base'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config, description='Path to RViz config file'
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
        parameters=[{'robot_description': robot_description}],
    )

    # Slider GUI for moving joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(
        [
            manipulator_version_arg,
            manipulator_only_arg,
            rviz_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
