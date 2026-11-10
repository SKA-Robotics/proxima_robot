from typing import List
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


class RosCommandSender(Node):
    def __init__(self, cartesian_command_in_topic: str, joint_command_in_topic: str, gripper_cmd_topic: str, preset_publisher_topic: str, use_sim_time: bool = False):

        super().__init__('ros_command_sender', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time))])

        self.twist_publisher = self.create_publisher(
            TwistStamped, cartesian_command_in_topic, 10)
        self.joint_publisher = self.create_publisher(
            JointJog, joint_command_in_topic, 10)
        self.gripper_client = ActionClient(self, GripperCommand, gripper_cmd_topic)
        self._last_gripper_position = None
        self.preset_publisher = self.create_publisher(
            JointState, preset_publisher_topic, 10)

    def send_twist_command(self, twist_data: List[float], frame_id: str):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = frame_id
        command.twist.linear.x = twist_data[0]
        command.twist.linear.y = twist_data[1]
        command.twist.linear.z = twist_data[2]
        command.twist.angular.x = twist_data[3]
        command.twist.angular.y = twist_data[4]
        command.twist.angular.z = twist_data[5]
        self.twist_publisher.publish(command)

    def send_joint_command(self, joint_data: List[float], joint_names: List[str]):
        command = JointJog()
        command.header.stamp = self.get_clock().now().to_msg()
        command.velocities = joint_data
        command.joint_names = joint_names
        self.joint_publisher.publish(command)

    def send_gripper_command(self, gripper_cmd: float):

        stroke = -0.0517  # joint_gripper_left, lower limit
        if gripper_cmd < 0.0:
            position = 0.0
        else:
            position = stroke * min(gripper_cmd, 1.0)

        if position == self._last_gripper_position:
            return
        if not self.gripper_client.server_is_ready():
            return
        self._last_gripper_position = position

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0
        self.gripper_client.send_goal_async(goal)

    def send_preset_command(self, target_q: List[float]):
        command = JointState()
        command.position = target_q
        self.preset_publisher.publish(command)
