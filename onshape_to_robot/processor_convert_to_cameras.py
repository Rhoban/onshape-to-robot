from .processor import Processor
from .config import Config
from .robot import Robot, Camera
from .message import info

class ProcessorConvertToCameras(Processor):
   """Convert frames to camera elements.
   Config: "cameras": {"camera_name": "frame_name", ...}
   Each entry maps a camera name to a frame name on a link.
   """

  def __init__(self, config: Config):
    super().__init__(config)
    self.cameras = config.get("cameras", {})

  def process(self, robot: Robot):
    for camera_name, frame_name in self.cameras.items():
      for link in robot.links:
        if frame_name in link.frames:
          print(f"  Creating camera '{camera_name}' from frame '{frame_name}' on link '{link.name}'")
          robot.cameras.append(Camera(camera_name, link.name, link.frames.pop(frame_name)))
          break