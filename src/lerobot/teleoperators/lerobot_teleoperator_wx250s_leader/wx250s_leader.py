from typing import Any

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, 
    robot_startup, 
    create_interbotix_global_node,
    get_interbotix_global_node
)

from lerobot.teleoperators.teleoperator import Teleoperator
from .wx250s_leader_config import WX250SLeaderConfig
from pynput import keyboard

class WX250SLeader(Teleoperator):
    config_class = WX250SLeaderConfig
    name = "wx250s_leader"

    def __init__(self, config: WX250SLeaderConfig):
        super().__init__(config)
        self.config=config

        try:
            self.global_node = create_interbotix_global_node()
        except:
            self.global_node = get_interbotix_global_node()

        self.bot = InterbotixManipulatorXS(
            robot_model='wx250s',
            robot_name='arm_1',
            group_name='arm',
            gripper_name='gripper',
            node=self.global_node
        )

        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )

        self.gripper_command = False

    @property
    def action_features(self) -> dict:
        return {
            "waist": float,
            "shoulder": float,
            "elbow": float,
            "forearm_roll": float,
            "wrist_angle": float,
            "wrist_rotate": float,
            "gripper": bool
        }
    @property
    def feedback_features(self) -> dict:
        return {}

    def connect(self) -> None:
        try:
            robot_startup(self.global_node)
        except:
            pass
        self.keyboard_listener.start()

        self.connected = True

    def is_connected(self) -> bool:
        return self.connected
    
    def on_key_press(self, key):
        try:
            if key.char == 'p':
                self.gripper_command = True
                self.bot.gripper.grasp(0)
            elif key.char == 'o':
                self.gripper_command = False
                self.bot.gripper.release(0)
        except AttributeError:
            pass

    def on_key_release(self, key):
        pass

    def get_action(self) -> dict[str, Any]:

        actions = self.bot.arm.get_joint_positions()

        action_dict = {
            "waist": actions[0],
            "shoulder": actions[1],
            "elbow": actions[2],
            "forearm_roll": actions[3],
            "wrist_angle": actions[4],
            "wrist_rotate": actions[5],
            "gripper": self.gripper_command
        }

        return action_dict

    def disconnect(self) -> None:
        try:
            robot_shutdown(self.global_node)
        except:
            pass

    def calibrate(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return True

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass
