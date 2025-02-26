import os
import subprocess
import shutil
import numpy as np
import pymeshlab
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .message import bright, info, error

class ProcessorSimplifySTLs(Processor):
    """
    Allow for mesh simplifications using MeshLab
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # STL merge / simplification
        self.simplify_stls = config.get("simplify_stls", False)
        self.max_stl_size = config.get("max_stl_size", 3)

    def process(self, robot: Robot):
        if self.simplify_stls:
            for link in robot.links:
                for part in link.parts:
                    self.simplify_stl(part.mesh_file)

    def reduce_faces(self, in_file: str, out_file: str, reduction: float = 0.9):
        ms = pymeshlab.MeshSet()
        # Add input mesh
        ms.load_new_mesh(in_file)

        # apply filter
        ms.apply_filter(
            'meshing_decimation_quadric_edge_collapse', 
            targetperc=reduction,
            qualitythr=0.5,
            preserveboundary=False,
            boundaryweight=1,
            preservenormal=True,
            preservetopology=False,
            optimalplacement=True,
            planarquadric=True,
            qualityweight=False,
            planarweight=0.001,
            autoclean=True,
            selected=False
        )

        # save mesh
        ms.save_current_mesh(out_file)

    def simplify_stl(self, stl_file: str):
        size_M = os.path.getsize(stl_file) / (1024 * 1024)

        if size_M > self.max_stl_size:
            print(
                info(
                    f"+ {os.path.basename(stl_file)} is {size_M:.2f} M, running mesh simplification"
                )
            )
            shutil.copyfile(stl_file, "/tmp/simplify.stl")
            self.reduce_faces("/tmp/simplify.stl", stl_file, self.max_stl_size / size_M)
