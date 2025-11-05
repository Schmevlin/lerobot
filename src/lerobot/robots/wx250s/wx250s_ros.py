from typing import Any
from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus
from lerobot.robots import Robot
import numpy as np
from spatialmath import SE3, SO3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import ikpy.chain

from . import WX250S_Config

class WX250S(Robot):
    config_class = WX250S_Config
    name = "wx250s"
    
    joints = [
        "waist",
        "shoulder",
        "elbow",
        "forearm_roll",
        "wrist_angle",
        "wrist_rotate"
    ]

    def __init__(self, config: WX250S_Config):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        self.bot = InterbotixManipulatorXS(
            robot_model='wx250s',
            group_name='arm',
            gripper_name='gripper',
        )

        self.chain = ikpy.chain.Chain.from_urdf_file("/home/mercis/lerobot-ros-test/lerobot/src/lerobot/robots/wx250s/assets/wx250s.urdf",base_elements=["wx250s/base_link"])

        self.end_eff_control = config.end_eff_control
        self.pos = SE3(self.bot.arm.get_ee_pose_command())

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
        if not self.end_eff_control:
            return self._motors_ft
        else:
            return {
                "delta_x": float,
                "delta_y": float,
                "delta_z": float,
                "delta_roll": float,
                "delta_pitch": float,
                "delta_yaw": float,
                "gripper": bool
            }
    
    @property
    def is_connected(self) -> bool:
       return self.connected and all(cam.is_connected for cam in self.cameras.values())
    
    @property
    def is_calibrated(self) -> bool:
        return True
    
    def calibrate(self) -> None:
        pass
    
    def connect(self) -> None: 
        robot_startup()

        self.connected = True

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def disconnect(self) -> None:
        robot_shutdown()

        self.connected = False

        for cam in self.cameras.values():
            cam.disconnect()

    def configure(self) -> None:
        # self.bot.arm.go_to_home_pose()
        pass

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

        if self.end_eff_control:
            # Move the end-effector in Cartesian space
            if action['delta_x'] == 0 and action['delta_y'] == 0 and action['delta_z'] == 0 and action['delta_roll'] == 0 and action['delta_pitch'] == 0 and action['delta_yaw'] == 0:
                success = False
            else:
                transform = SE3.RPY(
                    action['delta_roll'],
                    action['delta_pitch'],
                    action['delta_yaw'],
                    unit='deg'
                ) @ SE3(
                    action['delta_x'],
                    action['delta_y'],
                    action['delta_z']
                )
                new_pos = self.pos @ transform

                # print()
                # print(self.pos)
                # print(transform)
                # print(new_pos)

                theta_list, success = self.bot.arm.set_ee_pose_matrix(
                    new_pos, 
                    blocking=False, 
                    moving_time=0.2,
                    custom_guess=self.bot.arm.get_joint_positions()[:6])
                if success:
                    self.pos = new_pos
                else:
                    # reset that joint
                    # print(f"failed but here's the theta list: ({theta_list})")
                    pass

                
            # print()
            # print(transform)
            # print(new_pos)

            # print(new_pos)
                # j = self.chain.inverse_kinematics_frame(new_pos)

                # print(SE3(self.chain.forward_kinematics(j)))

                # success = self.bot.arm.set_joint_positions(j[:6],moving_time=1)

            

            # success = self.bot.arm.set_ee_cartesian_trajectory(
            #     x=action['delta_x'],
            #     y=action['delta_y'],
            #     z=action['delta_z'],
            #     roll=action['delta_roll'],
            #     pitch=action['delta_pitch'],
            #     yaw=action['delta_yaw']
            # )
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
        else:
            goal_pos = [val for key, val in action.items()]
            arm_goal = goal_pos[:6]
            success = self.bot.arm.set_joint_positions(arm_goal)
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