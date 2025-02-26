from pathlib import Path
from .message import bright, info, error, warning
from .processor import Processor
from .config import Config
from .robot import Robot, Part
from .geometry import Mesh
import numpy as np


class ProcessorCollisionAsVisual(Processor):
    """
    This processor will update the part mesh and shapes to turn every collision into visual.

    Can be useful for debugging
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # OpenSCAD pure shapes
        self.collisions_as_visual: bool = config.get("collisions_as_visual", False)

    def process(self, robot: Robot):
        """
        Runs the processor
        """
        if self.collisions_as_visual:
            print(info("+ Converting collisions to visual"))
            for link in robot.links:
                for part in link.parts:
                    for mesh in part.meshes:
                        mesh.visual = mesh.collision
                    for shape in part.shapes:
                        shape.visual = shape.collision
                    part.prune_unused_geometry()
