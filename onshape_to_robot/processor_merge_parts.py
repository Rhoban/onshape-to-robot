import numpy as np
import fnmatch
import os
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .geometry import Mesh
from .message import bright, info, error
from stl import mesh, Mode


class ProcessorMergeParts(Processor):
    """
    This processor merge all parts into a single one, combining the STL
    """

    def __init__(self, config: Config):
        super().__init__(config)
        self.merge_stls = config.get("merge_stls", False)

    def process(self, robot: Robot):
        if self.merge_stls:
            os.makedirs(self.config.asset_path("merged"), exist_ok=True)

            if self.merge_stls is True:
                patterns = {"*": "both"}
            elif type(self.merge_stls) is str:
                patterns = {"*": self.merge_stls}
            else:
                patterns = self.merge_stls

            for link in robot.links:
                merge_visual = False
                merge_collision = False

                for entry in patterns:
                    exclude = False

                    if patterns[entry] not in ["visual", "collision", "both"]:
                        print(
                            error(
                                f"Invalid merge_stls option '{patterns[entry]}' for pattern '{entry}'"
                            )
                        )
                        continue

                    if entry.startswith("!"):
                        exclude = True
                        entry = entry[1:]

                    if fnmatch.fnmatch(link.name, entry):
                        which = patterns[entry]
                        if exclude:
                            if which == "visual":
                                merge_visual = False
                            elif which == "collision":
                                merge_collision = False
                            else:
                                merge_visual = False
                                merge_collision = False
                        else:
                            merge_visual = which == "visual" or which == "both"
                            merge_collision = which == "collision" or which == "both"

                if merge_visual or merge_collision:
                    self.merge_parts(link, merge_visual, merge_collision)

    def load_mesh(self, stl_file: str) -> mesh.Mesh:
        return mesh.Mesh.from_file(stl_file)

    def save_mesh(self, mesh: mesh.Mesh, stl_file: str):
        # Tweaking STL header to avoid timestamp
        # This ensures that same process will result in same STL file
        def get_header(name):
            header = "onshape-to-robot"
            return header[:80].ljust(80, " ")

        mesh.get_header = get_header
        mesh.save(stl_file, mode=Mode.BINARY)

    def transform_mesh(self, mesh: mesh.Mesh, matrix: np.ndarray):
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]

        def transform(points):
            return (rotation @ points.T).T + translation

        mesh.v0 = transform(mesh.v0)
        mesh.v1 = transform(mesh.v1)
        mesh.v2 = transform(mesh.v2)
        mesh.normals = transform(mesh.normals)

    def combine_meshes(self, m1: mesh.Mesh, m2: mesh.Mesh):
        return mesh.Mesh(np.concatenate([m1.data, m2.data]))

    def merge_parts(self, link: Link, merge_visual: bool, merge_collision: bool):
        print(info(f"+ Merging parts for {link.name}"))

        merge_everything = merge_visual and merge_collision

        # Computing the frame where the new part will be located at
        _, com, __ = link.get_dynamics()
        T_world_com = np.eye(4)
        T_world_com[:3, 3] = com

        # Computing a new color, weighting by masses
        color = np.zeros(4)
        total_mass = 0
        for part in link.parts:
            if len(part.meshes):
                meshes_color = np.mean([mesh.color for mesh in part.meshes], axis=0)
                color += meshes_color * part.mass
            total_mass += part.mass

        color /= total_mass

        # Changing shapes frame
        merged_shapes = []
        for part in link.parts:
            if part.shapes is not None:
                for shape in part.shapes:
                    if (shape.collision and merge_collision) or (
                        shape.visual and merge_visual
                    ):
                        # Changing the shape frame
                        T_world_shape = part.T_world_part @ shape.T_part_shape
                        shape.T_part_shape = np.linalg.inv(T_world_com) @ T_world_shape
                        merged_shapes.append(shape)

        # Merging STL files
        def accumulate_meshes(which: str):
            mesh = None
            for part in link.parts:
                for part_mesh in part.meshes:
                    if part_mesh.is_type(which):
                        if which == "visual":
                            part_mesh.visual = False
                        else:
                            part_mesh.collision = False

                        # Retrieving meshes
                        part_mesh = self.load_mesh(part_mesh.filename)

                        # Expressing meshes in the merged frame
                        T_com_part = np.linalg.inv(T_world_com) @ part.T_world_part
                        self.transform_mesh(part_mesh, T_com_part)

                        if mesh is None:
                            mesh = part_mesh
                        else:
                            mesh = self.combine_meshes(mesh, part_mesh)
            return mesh

        merged_meshes = []

        if merge_visual:
            visual_mesh = accumulate_meshes("visual")
            if visual_mesh is not None:
                filename = self.config.asset_path(
                    "merged/" + "/" + link.name + "_visual.stl"
                )
                self.save_mesh(visual_mesh, filename)
                merged_meshes.append(
                    Mesh(filename, color, visual=True, collision=False)
                )

        if merge_collision:
            collision_mesh = accumulate_meshes("collision")
            if collision_mesh is not None:
                filename = self.config.asset_path(
                    "merged/" + "/" + link.name + "_collision.stl"
                )
                self.save_mesh(collision_mesh, filename)
                merged_meshes.append(
                    Mesh(filename, color, visual=False, collision=True)
                )

        mass, com, inertia = link.get_dynamics(T_world_com)
        if merge_everything:
            # Remove all parts
            link.parts = []
        else:
            # We keep the existing parts and add a massless part with merged meshes
            mass = 0
            inertia *= 0

        # Replacing parts with a single one
        link.parts.append(
            Part(
                f"{link.name}_parts",
                T_world_com,
                mass,
                com,
                inertia,
                merged_meshes,
                merged_shapes,
            )
        )
