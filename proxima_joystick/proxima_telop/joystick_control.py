from enum import Enum
from typing import List, Dict
from proxima_telop.ros_joy_receiver import RosJoyReceiver
from proxima_telop.ros_command_sender import RosCommandSender
from proxima_telop.gripper_controller import GripperController
from proxima_telop.utils import max_abs, trig_to_axis, JoystickTranslator, Debouncing
from proxima_telop.manip_config import ManipConfig, DEFAULT_CONFIG, load_ros_params
import time
import rclpy
from rclpy.node import Node

# MANIP_PRESET_DATABASE = {
#     "ik_ready": [0.0, -0.5, 1.85, -1.53, 0.96, -0.06],
#     "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#     "ground": [0.0, 1.0, 1.462, -1.618, 0.6327, 0.0],
#     "side_box": [-2.3279, 0.3627, 1.4217, -1.2624, 1.0883, 0.0651]
# }


class JoystickControl(Node):

    class Axis(Enum):
        LINEAR_X = 0
        LINEAR_Y = 1
        LINEAR_Z = 2
        ANGULAR_X = 3
        ANGULAR_Y = 4
        ANGULAR_Z = 5
        GRIPPER = 6
        JOINT_1 = 7
        JOINT_2 = 8
        JOINT_3 = 9
        JOINT_4 = 10
        JOINT_5 = 11
        JOINT_6 = 12
        GRIPPER_OPEN = 13
        GRIPPER_CLOSE = 14
        DRIVE_VX = 15
        DRIVE_VY = 16
        DRIVE_WZ = 17

    class Button(Enum):
        SET_FRAME_TOOL = 1
        SET_FRAME_BASE = 2
        SET_JOINT_MODE = 3
        CHANGE_MOVEMENT_MODE = 4
        GOTO_IK_READY = 5
        GOTO_ZERO = 6
        GOTO_GROUND = 7
        GOTO_SIDE_BOX = 8
        SET_DRIVE_MODE = 9
        SET_ARM_MODE = 10
        GEAR_UP = 11
        GEAR_DOWN = 12

    class SpaceMode(Enum):
        CARTESIAN = 0
        JOINT = 1

    class MovementMode(Enum):
        LINEAR = 0
        ANGULAR = 1

    class SubsystemMode(Enum):
        ARM = 0
        DRIVE = 1

    def __init__(self, config: ManipConfig = DEFAULT_CONFIG):
        super().__init__('joystick_control')
        if config is DEFAULT_CONFIG:
            config = load_ros_params(self)
        self.config = config
        self.movement_mode = self.MovementMode.LINEAR
        self.space_mode = self.SpaceMode.JOINT
        self.subsystem_mode = self.SubsystemMode.DRIVE  
        self.drive_gears = [0.25, 0.5, 0.75, 1.0]
        self.gear_index = 1  
        self.gear_up_debouncer = Debouncing()
        self.gear_down_debouncer = Debouncing()
        self.frame = config.base_frame_id
        self.gui = None

        self.gripper_controller = GripperController()
        self.command_sender = RosCommandSender(
            config.cartesian_command_in_topic, config.joint_command_in_topic, config.gripper_cmd_topic, config.preset_request_topic, self.get_parameter('use_sim_time').value)
        self.joy_receiver = RosJoyReceiver(config.joy_topic)

    def run(self):
        self.joy_receiver.register_callback(self.receive_command)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.command_sender)
        executor.add_node(self.joy_receiver)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    def receive_command(self, raw_axes: List[float], raw_buttons: List[int]):
        input = JoystickTranslator().translate({
            "axes": raw_axes,
            "buttons": raw_buttons
        })
        buttons = self._process_buttons(input)
        self._handle_buttons(buttons)
        # if buttons[self.Button.GOTO_IK_READY]:
        #     self._send_preset_request("ik_ready")
        # elif buttons[self.Button.GOTO_ZERO]:
        #     self._send_preset_request("zero")
        # elif buttons[self.Button.GOTO_GROUND]:
        #     self._send_preset_request("ground")
        # elif buttons[self.Button.GOTO_SIDE_BOX]:
        #     self._send_preset_request("side_box")
        # else:
        # self._update_gui(raw_axes, raw_buttons)
        axes = self._process_axes(input)
        self._publish_command(axes)
        # self._control_gripper(axes[self.Axis.GRIPPER]) # real robot
        self._control_gripper(
            axes[self.Axis.GRIPPER_CLOSE], axes[self.Axis.GRIPPER_OPEN])

    def _process_axes(self, input: Dict[str, float]) -> Dict[Axis, float]:
        return {
            self.Axis.LINEAR_X: -input["left_stick_vertical"],
            self.Axis.LINEAR_Y: max_abs(-input["left_stick_horizontal"], -input["right_stick_horizontal"]),
            self.Axis.LINEAR_Z: -input["right_stick_vertical"],
            self.Axis.ANGULAR_X: input["left_stick_horizontal"],
            self.Axis.ANGULAR_Y: max_abs(input["left_stick_vertical"], input["right_stick_vertical"]),
            self.Axis.ANGULAR_Z: -input["right_stick_horizontal"],
            # self.Axis.GRIPPER: max_abs(input["left_trigger"], input["right_trigger"]),
            self.Axis.GRIPPER_OPEN: input["right_trigger"],
            self.Axis.GRIPPER_CLOSE: input["left_trigger"],
            self.Axis.JOINT_1: -input["left_stick_horizontal"],
            self.Axis.JOINT_2: -input["left_stick_vertical"],
            self.Axis.JOINT_3: -input["right_stick_vertical"],
            self.Axis.JOINT_4: input["right_stick_horizontal"],
            self.Axis.JOINT_5: -input["left_stick_vertical"],
            self.Axis.JOINT_6: input["left_stick_horizontal"],
        }

    def _process_buttons(self, input: Dict[str, float]) -> Dict[Button, bool]:
        return {
            self.Button.SET_FRAME_BASE:
                input["start_button"] and (
                    input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_FRAME_TOOL:
                input["start_button"] and not (
                    input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_JOINT_MODE: input["back_button"],
            self.Button.CHANGE_MOVEMENT_MODE: input["left_bumper"] or input["right_bumper"],
            self.Button.GOTO_IK_READY: input["left_cross"],
            self.Button.GOTO_ZERO: input["up_cross"],
            self.Button.GOTO_GROUND: input["down_cross"],
            self.Button.GOTO_SIDE_BOX: input["right_cross"],
            self.Button.SET_ARM_MODE: input["X_button"],
            self.Button.SET_DRIVE_MODE: input["A_button"],
            self.Button.GEAR_UP: input["left_bumper"],
            self.Button.GEAR_DOWN: input["right_bumper"]
        }

    def _handle_buttons(self, buttons: Dict[Button, bool]):
        if buttons[self.Button.SET_DRIVE_MODE] and self.subsystem_mode != self.SubsystemMode.DRIVE:
        # stop arm motion before switching to drive
            self.command_sender.send_twist_command([0.0] * 6, self.frame)
            self.command_sender.send_joint_command([0.0] * 6, self.config.robot_joint_names)
            self.subsystem_mode = self.SubsystemMode.DRIVE

        elif buttons[self.Button.SET_ARM_MODE] and self.subsystem_mode != self.SubsystemMode.ARM:
            # stop wheels before switching to arm
            self.command_sender.send_drive_command(0.0, 0.0, 0.0)
            self.subsystem_mode = self.SubsystemMode.ARM

        if self.subsystem_mode == self.SubsystemMode.ARM
            if buttons[self.Button.SET_FRAME_TOOL]:
                self.frame = self.config.twist_cmd_frame_id
                self.space_mode = self.SpaceMode.CARTESIAN
            elif buttons[self.Button.SET_FRAME_BASE]:
                self.frame = self.config.base_frame_id
                self.space_mode = self.SpaceMode.CARTESIAN
            elif buttons[self.Button.SET_JOINT_MODE]:
                self.space_mode = self.SpaceMode.JOINT

            if buttons[self.Button.CHANGE_MOVEMENT_MODE]:
                self.movement_mode = self.MovementMode.ANGULAR
            else:
                self.movement_mode = self.MovementMode.LINEAR

        elif self.subsystem_mode == self.SubsystemMode.DRIVE:
            self.gear_up_debouncer.update(buttons[self.Button.GEAR_UP])
            self.gear_down_debouncer.update(buttons[self.Button.GEAR_DOWN])

            if self.gear_up_debouncer.is_rising_edge:
                self.gear_index = min(self.gear_index + 1, len(self.drive_gears) - 1)
                
            if self.gear_down_debouncer.is_rising_edge:
                self.gear_index = max(self.gear_index - 1, 0)
                
    def _publish_command(self, axes: Dict[Axis, float]):
        #DRIVE Logic
        if self.subsystem_mode == self.SubsystemMode.DRIVE:
            scale = self.drive_gears[self.gear_index]
            vx = axes[self.Axis.DRIVE_VX] * self.config.max_drive_linear * scale
            vy = axes[self.Axis.DRIVE_VY] * self.config.max_drive_linear * scale
            wz = axes[self.Axis.DRIVE_WZ] * self.config.max_drive_angular * scale
            self.command_sender.send_drive_command(vx, vy, wz)
            return 
        #ARM Logic
        if self.space_mode == self.SpaceMode.CARTESIAN:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.LINEAR_X] * self.config.max_ev[0],
                    axes[self.Axis.LINEAR_Y] * self.config.max_ev[1],
                    axes[self.Axis.LINEAR_Z] * self.config.max_ev[2],
                    0.0,
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    0.0,
                    axes[self.Axis.ANGULAR_X] * self.config.max_ev[3],
                    axes[self.Axis.ANGULAR_Y] * self.config.max_ev[4],
                    axes[self.Axis.ANGULAR_Z] * self.config.max_ev[5],
                ]
            self.command_sender.send_twist_command(command, self.frame)

        elif self.space_mode == self.SpaceMode.JOINT:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.JOINT_1] * self.config.max_qd[0],
                    axes[self.Axis.JOINT_2] * self.config.max_qd[1],
                    axes[self.Axis.JOINT_3] * self.config.max_qd[2],
                    axes[self.Axis.JOINT_4] * self.config.max_qd[3],
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    axes[self.Axis.JOINT_3] * self.config.max_qd[2],
                    axes[self.Axis.JOINT_4] * self.config.max_qd[3],
                    axes[self.Axis.JOINT_5] * self.config.max_qd[4],
                    axes[self.Axis.JOINT_6] * self.config.max_qd[5],
                ]
            self.command_sender.send_joint_command(
                command, self.config.robot_joint_names)

    # def add_gui(self, gui):
    #     self.gui = gui

    # def _update_gui(self, axes, buttons):
    #     if self.gui is not None:
    #         self.gui.update(axes, buttons, self.space_mode,
    #                         self.movement_mode, self.frame)

    # def _send_preset_request(self, preset_name: str):
    #     if preset_name not in MANIP_PRESET_DATABASE:
    #         print(f"Manip preset {preset_name} is not defined")
    #         return
    #     target_q = MANIP_PRESET_DATABASE[preset_name]
    #     self.command_sender.send_preset_command(target_q)

    # REAL ROBOT
    # def _control_gripper(self, gripper_axis: float):
    #     gripper_force_cmd = self.gripper_controller.step(gripper_axis)
    #     self.command_sender.send_gripper_command(gripper_force_cmd)

    def _control_gripper(self, gripper_axis_close: float, gripper_axis_open: float):
        gripper_force_cmd = self.gripper_controller.step(
            gripper_axis_close, gripper_axis_open)
        self.command_sender.send_gripper_command(gripper_force_cmd)


if __name__ == "__main__":
    print("Starting joystick control")
    joystick_control = JoystickControl()
    joystick_control.run()


def main(args=None):
    rclpy.init(args=args)

    print("Starting joystick control")
    joystick_control = JoystickControl()

    joystick_control.run()

    joystick_control.command_sender.destroy_node()
    joystick_control.joy_receiver.destroy_node()
    joystick_control.destroy_node()
    print("Joystick control finished")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
