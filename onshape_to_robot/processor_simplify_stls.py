import os
import subprocess
import shutil
import numpy as np
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .message import bright, info, error
from stl import mesh, Mode

# The filter script used with Meshlab when running meshes simplification
FILTER_SCRIPT_MLX = """
<!DOCTYPE FilterScript>
<FilterScript>
<filter name="Simplification: Quadric Edge Collapse Decimation">

<Param type="RichFloat" value="%reduction%" name="TargetPerc"/>
<Param type="RichFloat" value="0.5" name="QualityThr"/>
<Param type="RichBool" value="false" name="PreserveBoundary"/>
<Param type="RichFloat" value="1" name="BoundaryWeight"/>
<Param type="RichBool" value="true" name="PreserveNormal"/>
<Param type="RichBool" value="false" name="PreserveTopology"/>
<Param type="RichBool" value="true" name="OptimalPlacement"/>
<Param type="RichBool" value="true" name="PlanarQuadric"/>
<Param type="RichBool" value="false" name="QualityWeight"/>
<Param type="RichFloat" value="0.001" name="PlanarWeight"/>
<Param type="RichBool" value="true" name="AutoClean"/>
<Param type="RichBool" value="false" name="Selected"/>


</filter>
</FilterScript>
"""


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
            self.check_meshlab()

    def check_meshlab(self):
        print(bright("* Checking MeshLab presence..."))
        if not os.path.exists("/usr/bin/meshlabserver") != 0:
            print(
                error("No /usr/bin/meshlabserver, disabling STL simplification support")
            )
            print(info("TIP: consider installing meshlab:"))
            print(info("sudo apt-get install meshlab"))
            self.simplify_stls = False

    def process(self, robot: Robot):
        if self.simplify_stls:
            for link in robot.links:
                for part in link.parts:
                    self.simplify_stl(part.mesh_file)

    def create_tmp_filter_file(
        self, filename: str = "filter_file_tmp.mlx", reduction: float = 0.9
    ):
        with open("/tmp/" + filename, "w", encoding="utf-8") as stream:
            stream.write(FILTER_SCRIPT_MLX.replace("%reduction%", str(reduction)))
        return "/tmp/" + filename

    def reduce_faces(self, in_file: str, out_file: str, reduction: float = 0.9):
        filter_script_path = self.create_tmp_filter_file(reduction=reduction)

        # Add input mesh
        command = "meshlabserver -i " + in_file
        # Add the filter script
        command += " -s " + filter_script_path
        # Add the output filename and output flags
        command += " -o " + out_file
        command += " > /tmp/meshlab.log 2>&1"
        # Execute command
        # print("Going to execute: " + command)
        output = subprocess.check_output(command, shell=True)
        # last_line = output.splitlines()[-1]
        # print("Done:")
        # print(in_file + " > " + out_file + ": " + last_line)

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
