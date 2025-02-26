from pathlib import Path
from .message import bright, info, error, warning
from .processor import Processor
from .config import Config
from .robot import Robot, Part


class ProcessorNoCollisionMeshes(Processor):
    """
    This processor ensures no collision meshes are present in the robot
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # OpenSCAD pure shapes
        self.no_collision_meshes: bool = config.get("no_collision_meshes", False)

    def process(self, robot: Robot):
        """
        Runs the processor
        """
        if self.no_collision_meshes:
            print(info("+ Removing collision meshes"))
            for link in robot.links:
                for part in link.parts:
                    for mesh in part.meshes:
                        mesh.collision = False
                    part.prune_unused_geometry()
