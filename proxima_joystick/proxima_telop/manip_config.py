import numpy as np
from rclpy.node import Node
from typing import List


class ManipConfig:
    # Control params
    control_frequency: float = 30.0
    command_timeout: float = 0.3
    servo_gain: np.ndarray = np.array([20.0, 20.0, 20.0, 10.0, 10.0, 10.0])
    max_setpoint_position_distance: float = 0.05
    max_setpoint_orientation_distance: float = 0.12
    trajectory_duration: float = 6.0
    max_ev: np.ndarray = np.array(
        [0.2, 0.2, 0.2, 0.4, 0.4,
         0.4])  # Maximum cartesian velocity of the end effector
    max_qd: np.ndarray = np.array(
        [0.3, 0.3, 0.3, 0.6, 0.6,
         1.0])  # Maximum velocity of each of the arm joints

    # Robot params
    robot_joint_names: List[str] = [
        "joint0", "joint1", "joint2", "joint3", "joint4", "joint5"
    ]
    base_frame_id: str = "base_link"
    ee_frame_id: str = "link6"
    twist_cmd_frame_id: str = "manip_camera_optical"

    # ROS params
    joy_topic: str = "/joy"
    cartesian_command_in_topic: str = "/servo_node/delta_twist_cmds"
    joint_command_in_topic: str = "/servo_node/delta_joint_cmds"
    pose_topic: str = "/pose_cmd"
    joint_topic: str = "/joint_states"
    preset_request_topic: str = "/preset_request"
    gripper_cmd_topic: str = "/grip_action_controller/gripper_cmd"



DEFAULT_CONFIG = ManipConfig()


def _param(node: Node, name: str, default):
    """Declare a parameter on the node and return its value.

    A value passed at startup (launch file or ``--ros-args -p``) replaces
    *default*. Declaring twice is avoided so this can be called after the
    node already owns the parameter.
    """
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return node.declare_parameter(name, default).value


def load_ros_params(node: Node) -> ManipConfig:
    """Load manipulator configuration from this node's ROS 2 parameters.

    Missing parameters keep the defaults from ``ManipConfig``. Pass the
    joystick node; Jazzy has no global parameter server.
    """
    config = ManipConfig()

    config.control_frequency = float(
        _param(node, "control_frequency", config.control_frequency))
    config.command_timeout = float(
        _param(node, "command_timeout", config.command_timeout))
    config.servo_gain = np.array(
        _param(node, "servo_gain", config.servo_gain.tolist()), dtype=float)
    config.max_setpoint_position_distance = float(
        _param(node, "max_setpoint_position_distance",
               config.max_setpoint_position_distance))
    config.max_setpoint_orientation_distance = float(
        _param(node, "max_setpoint_orientation_distance",
               config.max_setpoint_orientation_distance))
    config.trajectory_duration = float(
        _param(node, "trajectory_duration", config.trajectory_duration))
    config.max_ev = np.array(
        _param(node, "max_ev", config.max_ev.tolist()), dtype=float)
    config.max_qd = np.array(
        _param(node, "max_qd", config.max_qd.tolist()), dtype=float)

    config.robot_joint_names = list(
        _param(node, "robot_joint_names", config.robot_joint_names))
    config.base_frame_id = _param(node, "base_frame_id", config.base_frame_id)
    config.ee_frame_id = _param(node, "ee_frame_id", config.ee_frame_id)
    config.twist_cmd_frame_id = _param(
        node, "twist_cmd_frame_id", config.twist_cmd_frame_id)

    config.joy_topic = _param(node, "joy_topic", config.joy_topic)
    config.cartesian_command_in_topic = _param(node, "cartesian_command_in_topic", config.cartesian_command_in_topic)
    config.pose_topic = _param(node, "pose_topic", config.pose_topic)
    config.joint_command_in_topic = _param(node, "joint_command_in_topic", config.joint_command_in_topic)
    config.gripper_cmd_topic = _param(
        node, "gripper_cmd_topic", config.gripper_cmd_topic)
    config.preset_request_topic = _param(
        node, "preset_request_topic", config.preset_request_topic)

    return config
