from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("wx250s_follower")
@dataclass
class WX250SFollowerConfig(RobotConfig):
    port: str = None
    cameras: dict[str, CameraConfig] = field(
        default_factory= lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path="/dev/video0",
                fps=30,
                width=640,
                height=480,
            )
        }
    )