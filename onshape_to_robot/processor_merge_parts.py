import numpy as np
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .message import bright, info, error
from stl import mesh, Mode


class ProcessorMergeParts(Processor):
    """
    This processor merge all parts into a single one, combining the STL
    """

    def __init__(self, config: Config):
        super().__init__(config)
        self.merge_stls = config.get("merge_stls", False)
        self.meshes_to_write = {}

    def process(self, robot: Robot):
        if self.merge_stls:
            for link in robot.links:
                self.merge_parts(link)
            for stl_file, mesh in self.meshes_to_write.items():
                self.save_mesh(mesh, stl_file)

    def load_mesh(self, stl_file: str) -> mesh.Mesh:
        return mesh.Mesh.from_file(stl_file)

    def save_mesh(self, mesh: mesh.Mesh, stl_file: str):
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

    def merge_parts(self, link: Link):
        print(info(f"+ Merging parts for {link.name}"))

        # Computing the frame where the new part will be located at
        _, com, __ = link.get_dynamics()
        T_world_com = np.eye(4)
        T_world_com[:3, 3] = com

        # Computing a new color, weighting by masses
        color = np.zeros(3)
        total_mass = 0
        for part in link.parts:
            color += part.color * part.mass
            total_mass += part.mass
        color /= total_mass

        # Gathering shapes
        shapes = None
        for part in link.parts:
            if part.shapes is not None:
                if shapes is None:
                    shapes = []
                for shape in part.shapes:
                    # Changing the shape frame
                    T_world_shape = part.T_world_part @ shape.T_part_shape
                    shape.T_part_shape = np.linalg.inv(T_world_com) @ T_world_shape
                    shapes.append(shape)

        # Merging STL files
        mesh = None
        for part in link.parts:
            if part.mesh_file:
                # Retrieving meshes
                part_mesh = self.load_mesh(part.mesh_file)

                # Expressing meshes in the merged frame
                T_com_part = np.linalg.inv(T_world_com) @ part.T_world_part
                self.transform_mesh(part_mesh, T_com_part)

                if mesh is None:
                    mesh = part_mesh
                else:
                    mesh = self.combine_meshes(mesh, part_mesh)

        combined_stl_file = self.config.output_directory + "/" + link.name + ".stl"
        self.meshes_to_write[combined_stl_file] = mesh

        mass, com, inertia = link.get_dynamics(T_world_com)

        # Replacing parts with a single one
        link.parts = [
            Part(
                link.name,
                T_world_com,
                combined_stl_file,
                mass,
                com,
                inertia,
                color,
                shapes,
            )
        ]
