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
            "camera1": OpenCVCameraConfig( # hand camera camera1
                index_or_path="/dev/video7",
                fps=30,
                width=640,
                height=480,
            ),
            "camera2": OpenCVCameraConfig( # side camera camera2
                index_or_path="/dev/video9",
                fps=30,
                width=640,
                height=480,
            ),
            "camera3": RealSenseCameraConfig( # front camera camera3
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