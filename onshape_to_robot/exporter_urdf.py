import numpy as np
import os
from .robot import Robot, Link, Part, Joint
from .config import Config
from .shapes import Box, Cylinder, Sphere
from .exporter import Exporter
from .exporter_utils import xml_escape, rotation_matrix_to_rpy


class ExporterURDF(Exporter):
    def __init__(self, robot: Robot, config: Config | None = None):
        super().__init__()
        self.robot: Robot = robot
        self.config: Config = config

        self.draw_collisions: bool = False
        self.no_dynamics: bool = False
        self.package_name: str = ""
        self.additional_xml: str = ""

        if config is not None:
            self.no_dynamics = config.no_dynamics
            self.draw_collisions: bool = config.get("drawCollisions", False)
            self.package_name: str = config.get("packageName", "")
            additional_xml_file = config.get("additionalUrdfFile", "")
            if additional_xml_file:
                with open(additional_xml_file, "r") as file:
                    self.additional_xml = file.read()

    def append(self, line: str):
        self.xml += line

    def build(self):
        self.xml = ""
        self.append('<?xml version="1.0" ?>')
        self.append("<!-- Generated using onshape-to-robot -->")
        self.append(f'<robot name="{self.robot.name}">')
        self.add_link(self.robot.get_base_link())

        if self.additional_xml:
            self.append(self.additional_xml)

        self.append("</robot>")

        return self.xml

    def add_inertial(self, mass: float, com: np.ndarray, inertia: np.ndarray):
        self.append("<inertial>")
        self.append(
            '<origin xyz="%.20g %.20g %.20g" rpy="0 0 0"/>'
            % (
                com[0],
                com[1],
                com[2],
            )
        )
        self.append('<mass value="%.20g" />' % mass)
        self.append(
            (
                '<inertia ixx="%.20g" ixy="%.20g"  ixz="%.20g" iyy="%.20g" iyz="%.20g" izz="%.20g" />'
                % (
                    inertia[0, 0],
                    inertia[0, 1],
                    inertia[0, 2],
                    inertia[1, 1],
                    inertia[1, 2],
                    inertia[2, 2],
                )
            )
        )
        self.append("</inertial>")

    def add_mesh(self, part: Part, node: str, T_world_link: np.ndarray):
        """
        Add a mesh node (e.g. STL) to the URDF file
        """
        self.append(f"<{node}>")

        T_link_part = np.linalg.inv(T_world_link) @ part.T_world_part
        self.append(self.origin(T_link_part))

        mesh_file = os.path.basename(part.mesh_file)
        if self.package_name:
            mesh_file = self.package_name + "/" + mesh_file

        self.append("<geometry>")
        self.append(f'<mesh filename="package://{xml_escape(mesh_file)}" />')
        self.append("</geometry>")

        if node == "visual":
            material_name = f"{part.name}_material"
            self.append(f'<material name="{xml_escape(material_name)}">')
            self.append(
                '<color rgba="%.20g %.20g %.20g 1.0"/>'
                % (part.color[0], part.color[1], part.color[2])
            )
            self.append("</material>")

        self.append(f"</{node}>")

    def add_shapes(self, part: Part, node: str, T_world_link: np.ndarray):
        """
        Add shapes (box, spher and cylinder) nodes to the URDF.
        """
        for shape in part.shapes:
            self.append(f"<{node}>")

            T_link_shape = (
                np.linalg.inv(T_world_link) @ part.T_world_part @ shape.T_part_shape
            )
            self.append(self.origin(T_link_shape))

            self.append("<geometry>")
            if isinstance(shape, Box):
                self.append('<box size="%.20g %.20g %.20g" />' % tuple(shape.size))
            elif isinstance(shape, Cylinder):
                self.append(
                    '<cylinder length="%.20g" radius="%.20g" />'
                    % (shape.length, shape.radius)
                )
            elif isinstance(shape, Sphere):
                self.append('<sphere radius="%.20g" />' % shape.radius)
            self.append("</geometry>")

            if node == "visual":
                material_name = f"{part.name}_material"
                self.append(f'<material name="{xml_escape(material_name)}">')
                self.append(
                    '<color rgba="%.20g %.20g %.20g 1.0"/>'
                    % (part.color[0], part.color[1], part.color[2])
                )
                self.append("</material>")

            self.append(f"</{node}>")

    def add_geometries(
        self, part: Part, T_world_link: np.ndarray, node: str, what: str
    ):
        """
        Add geometry nodes. "node" is the actual XML node produced, "what" is the logic used to produce it.
        Both can be "visual" or "collision"
        """
        if what == "collision" and part.shapes is not None:
            self.add_shapes(part, node, T_world_link)
        elif part.mesh_file:
            self.add_mesh(part, node, T_world_link)

    def add_joint(self, joint: Joint, T_world_link: np.ndarray):
        self.append(f"<!-- Joint from {joint.parent.name} to {joint.child.name} -->")
        self.append(f'<joint name="{joint.name}" type="{joint.joint_type}">')

        T_link_joint = np.linalg.inv(T_world_link) @ joint.T_world_joint
        self.append(self.origin(T_link_joint))

        self.append(f'<parent link="{joint.parent.name}" />')
        self.append(f'<child link="{joint.child.name}" />')
        self.append('<axis xyz="%.20g %.20g %.20g"/>' % tuple(joint.z_axis))

        lower_upper_limits = ""
        if joint.limits is not None:
            lower_upper_limits = 'lower="%.20g" upper="%.20g"' % joint.limits

        self.append(
            '<limit effort="%.20g" velocity="%.20g" %s/>'
            % (
                joint.max_effort,
                joint.max_velocity,
                lower_upper_limits,
            )
        )

        self.append('<joint_properties friction="0.0"/>')
        self.append("</joint>")

    def add_frame(
        self,
        link: Link,
        frame: str,
        T_world_link: np.ndarray,
        T_world_frame: np.ndarray,
    ):
        self.append(f"<!-- Frame {frame} (dummy link + fixed joint) -->")
        T_link_frame = np.linalg.inv(T_world_link) @ T_world_frame

        # Adding a dummy link to the assembly
        self.append(f'<link name="{frame}">')
        self.append(self.origin(np.eye(4)))

        self.append("<inertial>")
        self.append('<origin xyz="0 0 0" rpy="0 0 0" />')
        if self.no_dynamics:
            self.append('<mass value="0" />')
        else:
            self.append('<mass value="1e-9" />')
        self.append('<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />')
        self.append("</inertial>")

        self.append("</link>")

        # Attaching this dummy link to the parent frame using a fixed joint
        self.append(f'<joint name="{frame}_frame" type="fixed">')
        self.append(self.origin(T_link_frame))
        self.append(f'<parent link="{link.name}" />')
        self.append(f'<child link="{frame}" />')
        self.append('<axis xyz="0 0 0"/>')
        self.append("</joint>")

    def add_link(self, link: Link, T_world_link: np.ndarray = np.eye(4)):
        """
        Adds a link recursively to the URDF file
        """
        self.append(f"<!-- Link {link.name} -->")
        self.append(f'<link name="{link.name}">')

        # Adding inertial properties
        mass, com, inertia = link.get_dynamics(T_world_link)
        self.add_inertial(mass, com, inertia)

        # Adding geometry objects
        for part in link.parts:
            self.append(f"<!-- Part {part.name} -->")
            self.add_geometries(
                part,
                T_world_link,
                "visual",
                "collision" if self.draw_collisions else "visual",
            )
            self.add_geometries(part, T_world_link, "collision", "collision")

        self.append("</link>")

        # Adding frames attached to current link
        for frame, T_world_frame in link.frames.items():
            self.add_frame(link, frame, T_world_link, T_world_frame)

        # Adding joints and children links
        for joint in self.robot.get_link_joints(link):
            self.add_link(joint.child, joint.T_world_joint)
            self.add_joint(joint, T_world_link)

    def origin(self, matrix: np.ndarray):
        """
        Transforms a transformation matrix into a URDF origin tag
        """
        urdf = '<origin xyz="%.20g %.20g %.20g" rpy="%.20g %.20g %.20g" />'

        return urdf % (*matrix[:3, 3], *rotation_matrix_to_rpy(matrix))
