from typing import Any
from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, 
    robot_startup, 
    create_interbotix_global_node,
    get_interbotix_global_node
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


from . import WX250SFollowerConfig

class WX250SFollower(Robot):
    config_class = WX250SFollowerConfig
    name = "wx250s_follower"
    
    joints = [
        "waist",
        "shoulder",
        "elbow",
        "forearm_roll",
        "wrist_angle",
        "wrist_rotate"
    ]

    def __init__(self, config: WX250SFollowerConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        try:
            self.global_node = create_interbotix_global_node()
        except:
            self.global_node = get_interbotix_global_node()

        self.bot = InterbotixManipulatorXS(
            robot_model='wx250s',
            robot_name='arm_2',
            group_name='arm',
            gripper_name='gripper',
            node=self.global_node
        )

    @property
    def _motors_ft(self) -> dict[str,type]:
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
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }
    
    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}
    
    @property
    def action_features(self) -> dict:
        return self._motors_ft
    
    @property
    def is_connected(self) -> bool:
       return self.connected and all(cam.is_connected for cam in self.cameras.values())
    
    @property
    def is_calibrated(self) -> bool:
        return True
    
    def calibrate(self) -> None:
        pass
    
    def connect(self) -> None: 
        try:
            robot_startup(self.global_node)
        except:
            pass

        self.connected = True

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def disconnect(self) -> None:
        try:
            robot_shutdown(self.global_node)
        except:
            pass

        self.connected = False

        for cam in self.cameras.values():
            cam.disconnect()

    def configure(self) -> None:
        print("Configured robot")

    def get_gripper_state(self) -> bool:
        effort = self.bot.gripper.get_gripper_effort()
        finger_pos = self.bot.gripper.get_finger_position()

        gripper_state = False # true is closed .015

        if effort < -500: gripper_state = True
        elif effort > 500: gripper_state = False
        elif effort < -100 and finger_pos < 0.036:
            gripper_state = True
        elif effort > 100 and finger_pos > 0.016:
            gripper_state = False
        elif finger_pos < 0.026:
            gripper_state = True
        #else it stays false from initialization

        return gripper_state

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        gripper_state = self.get_gripper_state()

        # print(f'{gripper_state} ef:{effort} pos:{finger_pos}')

        # Read arm position
        pos_list = self.bot.arm.get_joint_positions()
        obs_dict = {self.joints[i] : pos_list[i] for i in range(6)}

        obs_dict['gripper'] = gripper_state

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:

        gripper_goal = action["gripper"]
        
        if gripper_goal:
            self.bot.gripper.grasp(0)
        else:
            self.bot.gripper.release(0)

        goal_pos = [val for _, val in action.items()]
        arm_goal = goal_pos[:6]
        success = self.bot.arm.set_joint_positions(arm_goal, moving_time=0.2, blocking=False)
        if not success:
            return {
                "waist": 0,
                "shoulder": 0,
                "elbow": 0,
                "forearm_roll": 0,
                "wrist_angle": 0,
                "wrist_rotate": 0,
                "gripper": gripper_goal
            }
        
        return action