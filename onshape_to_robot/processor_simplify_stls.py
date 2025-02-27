import os
import subprocess
import shutil
import numpy as np
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

        if self.simplify_stls:
            self.pymeshlab = self.check_meshlab()
    
    def check_meshlab(self):
        print(bright("* Checking pymeshlab presence..."))
        try:
            import pymeshlab
            return pymeshlab
        except ImportError:
            self.simplify_stls = False
            print(
                error("No pymeshlab, disabling STL simplification support")
            )
            print(info("TIP: consider installing pymeshlab:"))
            print(info("pip install pymeshlab"))
            

    def process(self, robot: Robot):
        if self.simplify_stls:
            for link in robot.links:
                for part in link.parts:
                  for mesh in part.meshes:
                        self.simplify_stl(mesh.filename)

    def reduce_faces(self, filename: str, reduction: float = 0.9):
        ms = self.pymeshlab.MeshSet()
        # Add input mesh
        ms.load_new_mesh(filename)

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
        ms.save_current_mesh(filename)

    def simplify_stl(self, filename: str):
        size_M = os.path.getsize(filename) / (1024 * 1024)

        if size_M > self.max_stl_size:
            print(
                info(
                    f"+ {os.path.basename(in_file)} is {size_M:.2f} M, running mesh simplification"
                )
            )
            self.reduce_faces(filename, filename, self.max_stl_size / size_M)
