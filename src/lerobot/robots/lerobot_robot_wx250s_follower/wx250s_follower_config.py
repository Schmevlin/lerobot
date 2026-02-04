from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("wx250s_follower")
@dataclass
class WX250SFollowerConfig(RobotConfig):
    port: str = None
    cameras: dict[str, CameraConfig] = field(
        default_factory= lambda: {
            "camera1": OpenCVCameraConfig(
                index_or_path="/dev/video8",
                fps=30,
                width=640,
                height=480,
            ),
            "camera2": OpenCVCameraConfig(
                index_or_path="/dev/video6",
                fps=30,
                width=640,
                height=480,
            ),
            "camera3": RealSenseCameraConfig(
                serial_number_or_name="249322064122",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=True,
                rotation=Cv2Rotation.NO_ROTATION
)
        }
    )