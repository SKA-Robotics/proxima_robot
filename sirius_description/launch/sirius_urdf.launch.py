"""Launch file for the Sirius robot URDF and visualization."""

import os

import launch
import launch_ros
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='sirius_description').find(
        'sirius_description'
    )
    urdfModelPath = os.path.join(pkgPath, 'robots/sirius_final.urdf')
    rvizConfigPath = os.path.join(pkgPath, 'config/config.rviz')

    with open(urdfModelPath) as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[params]
    # )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='gui',
                default_value='True',
                description='This is a flag for joint_state_publisher_gui',
            ),
            # launch.actions.DeclareLaunchArgument(name='model', default_value=urdfModelPath,
            #                                     description='Path to the urdf model file'),
            robot_state_publisher_node,
            # joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
