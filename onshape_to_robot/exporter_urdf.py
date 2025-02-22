import numpy as np
import os
from .message import warning
from .robot import Robot, Link, Part, Joint
from .config import Config
from .shapes import Box, Cylinder, Sphere
from .exporter import Exporter
from .exporter_utils import xml_escape, rotation_matrix_to_rpy


class ExporterURDF(Exporter):
    def __init__(self, config: Config | None = None):
        super().__init__()
        self.config: Config = config

        self.ext: str = "urdf"
        self.draw_collisions: bool = False
        self.no_dynamics: bool = False
        self.package_name: str = ""
        self.additional_xml: str = ""
        self.collisions_no_mesh: bool = False

        if config is not None:
            self.no_dynamics = config.no_dynamics
            self.collisions_no_mesh: bool = config.get("collisions_no_mesh", False)
            self.draw_collisions: bool = config.get("draw_collisions", False)
            self.package_name: str = config.get("package_name", "")
            additional_xml_file = config.get("additional_xml", "")
            if additional_xml_file:
                with open(
                    config.output_directory + "/" + additional_xml_file, "r"
                ) as file:
                    self.additional_xml = file.read()

    def append(self, line: str):
        self.xml += line

    def build(self, robot: Robot):
        self.xml = ""
        self.append('<?xml version="1.0" ?>')
        self.append("<!-- Generated using onshape-to-robot -->")
        if self.config:
            self.append(f"<!-- OnShape {self.config.printable_version()} -->")
        self.append(f'<robot name="{robot.name}">')

        if len(robot.base_links) > 1:
            print(
                warning(
                    "WARNING: Multiple base links detected, which is not supported by URDF."
                )
            )
            print(warning("Only the first base link will be considered."))

        if len(robot.base_links) > 0:
            self.add_link(robot, robot.base_links[0])

        if self.additional_xml:
            self.append(self.additional_xml)

        self.append("</robot>")

        return self.xml

    def add_inertial(
        self, mass: float, com: np.ndarray, inertia: np.ndarray, fixed: str = False
    ):
        # Unless "no_dynamics" is set, we make sure that mass and inertia
        # are not zero
        if not self.no_dynamics:
            mass = max(1e-9, mass)
            inertia[0, 0] = max(1e-9, inertia[0, 0])
            inertia[1, 1] = max(1e-9, inertia[1, 1])
            inertia[2, 2] = max(1e-9, inertia[2, 2])
        if fixed:
            # To mark an object as fixed in the world, sets its dynamics to zero
            mass = 0
            com = np.zeros(3)
            inertia = np.zeros((3, 3))

        self.append("<inertial>")
        self.append(
            '<origin xyz="%g %g %g" rpy="0 0 0"/>'
            % (
                com[0],
                com[1],
                com[2],
            )
        )
        self.append('<mass value="%g" />' % mass)
        self.append(
            '<inertia ixx="%g" ixy="%g"  ixz="%g" iyy="%g" iyz="%g" izz="%g" />'
            % (
                inertia[0, 0],
                inertia[0, 1],
                inertia[0, 2],
                inertia[1, 1],
                inertia[1, 2],
                inertia[2, 2],
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
                '<color rgba="%g %g %g 1.0"/>'
                % (part.color[0], part.color[1], part.color[2])
            )
            self.append("</material>")

        self.append(f"</{node}>")

    def add_shapes(self, part: Part, node: str, T_world_link: np.ndarray):
        """
        Add shapes (box, sphere and cylinder) nodes to the URDF.
        """
        for shape in part.shapes:
            self.append(f"<{node}>")

            T_link_shape = (
                np.linalg.inv(T_world_link) @ part.T_world_part @ shape.T_part_shape
            )
            self.append(self.origin(T_link_shape))

            self.append("<geometry>")
            if isinstance(shape, Box):
                self.append('<box size="%g %g %g" />' % tuple(shape.size))
            elif isinstance(shape, Cylinder):
                self.append(
                    '<cylinder length="%g" radius="%g" />'
                    % (shape.length, shape.radius)
                )
            elif isinstance(shape, Sphere):
                self.append('<sphere radius="%g" />' % shape.radius)
            self.append("</geometry>")

            if node == "visual":
                material_name = f"{part.name}_material"
                self.append(f'<material name="{xml_escape(material_name)}">')
                self.append(
                    '<color rgba="%g %g %g 1.0"/>'
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
        elif part.mesh_file and (what == "visual" or not self.collisions_no_mesh):
            self.add_mesh(part, node, T_world_link)

    def add_joint(self, joint: Joint, T_world_link: np.ndarray):
        self.append(f"<!-- Joint from {joint.parent.name} to {joint.child.name} -->")

        joint_type = joint.properties.get("type", joint.joint_type)
        self.append(f'<joint name="{joint.name}" type="{joint_type}">')

        T_link_joint = np.linalg.inv(T_world_link) @ joint.T_world_joint
        self.append(self.origin(T_link_joint))

        self.append(f'<parent link="{joint.parent.name}" />')
        self.append(f'<child link="{joint.child.name}" />')
        self.append('<axis xyz="%g %g %g"/>' % tuple(joint.z_axis))

        limits = ""
        if "max_effort" in joint.properties:
            limits += 'effort="%g" ' % joint.properties["max_effort"]
        else:
            limits += 'effort="10" '

        if "max_velocity" in joint.properties:
            limits += 'velocity="%g" ' % joint.properties["max_velocity"]
        else:
            limits += 'velocity="10" '

        joint_limits = joint.properties.get("limits", joint.limits)
        if joint_limits is not None:
            limits += 'lower="%g" upper="%g" ' % (joint_limits[0], joint_limits[1])
        elif joint_type == "revolute":
            limits += f'lower="{-np.pi}" upper="{np.pi}" '
        elif joint_type == "prismatic":
            limits += 'lower="-1" upper="1" '

        if limits:
            self.append(f"<limit {limits}/>")

        if "friction" in joint.properties:
            self.append(f'<joint_properties friction="{joint.properties}"/>')
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

    def add_link(self, robot: Robot, link: Link, T_world_link: np.ndarray = np.eye(4)):
        """
        Adds a link recursively to the URDF file
        """
        self.append(f"<!-- Link {link.name} -->")
        self.append(f'<link name="{link.name}">')

        # Adding inertial properties
        mass, com, inertia = link.get_dynamics(T_world_link)
        self.add_inertial(mass, com, inertia, link.fixed)

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
        for joint in robot.get_link_joints(link):
            self.add_link(robot, joint.child, joint.T_world_joint)
            self.add_joint(joint, T_world_link)

    def origin(self, matrix: np.ndarray):
        """
        Transforms a transformation matrix into a URDF origin tag
        """
        urdf = '<origin xyz="%g %g %g" rpy="%g %g %g" />'

        return urdf % (*matrix[:3, 3], *rotation_matrix_to_rpy(matrix))
