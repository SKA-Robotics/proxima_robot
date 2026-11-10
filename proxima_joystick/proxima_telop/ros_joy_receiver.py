import rclpy
from rclpy.node import Node
from typing import Callable, List
from sensor_msgs.msg import Joy


class RosJoyReceiver(Node):

    def __init__(self, topic: str):
        super().__init__('ros_joy_receiver')
        self.subscription = self.create_subscription(
            Joy, topic, self.joy_callback, 10)
        self.subscription

        self.callback = None

    def joy_callback(self, msg: Joy):
        if self.callback is not None:
            self.callback(msg.axes, msg.buttons)

    def register_callback(self, callback: Callable[[List[float], List[int]],
                                                   None]):
        self.callback = callback

def main(args=None):
    rclpy.init(args=args)

    ros_joy_receiver = RosJoyReceiver()

    rclpy.spin(ros_joy_receiver)

    ros_joy_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
