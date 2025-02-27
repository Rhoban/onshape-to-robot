import hashlib
import os
from pathlib import Path
from .message import bright, info, error, warning
from .processor import Processor
from .config import Config
from .robot import Robot, Part
from .geometry import Mesh
import numpy as np
import pickle


class ProcessorConvexDecomposition(Processor):
    """
    Convex decomposition processor. Runs CoACD algorithm on collision meshes to use a convex approximation.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Enable convex decomposition
        self.convex_decomposition: bool = config.get("convex_decomposition", False)
        self.rainbow_colors: bool = config.get("rainbow_colors", False)

        self.check_coacd()

    def get_cache_path(self) -> Path:
        """
        Return the path to the user cache.
        """
        path = Path.home() / ".cache" / "onshape-to-robot-convex-decomposition"
        path.mkdir(parents=True, exist_ok=True)
        return path

    def check_coacd(self):
        if self.convex_decomposition:
            print(bright("* Checking CoACD presence..."))
            try:
                import coacd
                import trimesh
            except ImportError:
                print(bright("Can't import CoACD, disabling convex decomposition."))
                print(info("TIP: consider installing CoACD:"))
                print(info("pip install coacd trimesh"))
                self.convex_decomposition = False

    def process(self, robot: Robot):
        if self.convex_decomposition:
            os.makedirs(self.config.asset_path("convex_decomposition"), exist_ok=True)

            for link in robot.links:
                for part in link.parts:
                    self.convex_decompose(part)

    def convex_decompose(self, part: Part):
        import coacd
        import trimesh

        collision_meshes = [mesh for mesh in part.meshes if mesh.collision]
        if len(collision_meshes) > 0:
            if len(collision_meshes) > 1:
                print(
                    warning(
                        f"* Skipping convex decomposition for part {part.name} as it already has multiple collision meshes."
                    )
                )

            collision_mesh = collision_meshes[0]

            # Retrieving file SHA1
            sha1 = hashlib.sha1(open(collision_mesh.filename, "rb").read()).hexdigest()
            cache_filename = f"{self.get_cache_path()}/{sha1}.pkl"

            if os.path.exists(cache_filename):
                print(
                    info(
                        f"* Loading cached CoACD decomposition cache for part {part.name}"
                    )
                )
                with open(cache_filename, "rb") as f:
                    meshes = pickle.load(f)
            else:
                mesh = trimesh.load(collision_mesh.filename, force="mesh")
                mesh = coacd.Mesh(mesh.vertices, mesh.faces)
                meshes = coacd.run_coacd(mesh, max_convex_hull=16)
                with open(cache_filename, "wb") as f:
                    pickle.dump(meshes, f)

            part.collision_meshes = []
            filename = self.config.asset_path(
                f"convex_decomposition/{part.name}_%05d.stl"
            )
            for k, mesh in enumerate(meshes):
                mesh = trimesh.Trimesh(vertices=mesh[0], faces=mesh[1])
                mesh.export(filename % k)
                color = (
                    np.random.rand(3) if self.rainbow_colors else collision_mesh.color
                )
                part.meshes.append(
                    Mesh(
                        filename % k,
                        color,
                        visual=False,
                        collision=True,
                    )
                )
                part.collision_meshes.append(filename % k)

            collision_mesh.collision = False
            part.prune_unused_geometry()

            print(
                info(f"* Decomposed part {part.name} into {len(meshes)} convex shapes.")
            )
