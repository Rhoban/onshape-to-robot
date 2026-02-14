from .processor import Processor
from .config import Config
from .robot import Robot, Camera
from .message import info

class ProcessorConvertToCameras(Processor):
   """Convert frames to camera elements.
   Config: "cameras": {"camera_name": "frame_name", ...}
   or: "cameras": {"camera_name": {"frame": "frame_name", "fovy": 60, ...}, ...}
   Each entry maps a camera name to a frame name on a link.
   """

  def __init__(self, config: Config):
    super().__init__(config)
    self.cameras = config.get("cameras", {})

  def process(self, robot: Robot):
    for camera_name, camera_config in self.cameras.items():
      if isinstance(camera_config, str):
        frame_name = camera_config
        camera_kwargs = {}
      elif isinstance(camera_config, dict):
        frame_name = camera_config["frame"]
        camera_kwargs = {}
        if "fovy" in camera_config:
          camera_kwargs["fovy"] = camera_config["fovy"]
        if "mode" in camera_config:
          camera_kwargs["mode"] = camera_config["mode"]
        if "resolution" in camera_config:
          camera_kwargs["resolution"] = tuple(camera_config["resolution"])
      else:
        continue

      for link in robot.links:
        if frame_name in link.frames:
          print(info(f"Creating camera '{camera_name}' from frame '{frame_name}' on link '{link.name}'"))
          robot.cameras.append(Camera(camera_name, link.name, link.frames.pop(frame_name), **camera_kwargs))
          break