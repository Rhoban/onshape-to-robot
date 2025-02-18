import numpy as np
import os
from .message import success
from .robot import Robot, Link, Part, Joint
from .config import Config
from .shapes import Box, Cylinder, Sphere
from .exporter import Exporter
from .exporter_utils import xml_escape, rotation_matrix_to_rpy
from transforms3d.quaternions import mat2quat


class ExporterMuJoCo(Exporter):
    def __init__(self, config: Config | None = None):
        super().__init__()
        self.config: Config = config

        self.draw_collisions: bool = False
        self.no_dynamics: bool = False
        self.additional_xml: str = ""
        self.meshes: dict = {}

        if config is not None:
            self.no_dynamics = config.no_dynamics
            self.draw_collisions: bool = config.get("drawCollisions", False)
            additional_xml_file = config.get("additionalUrdfFile", "")
            if additional_xml_file:
                with open(additional_xml_file, "r") as file:
                    self.additional_xml = file.read()

    def append(self, line: str):
        self.xml += line

    def build(self, robot: Robot):
        self.xml = ""
        self.append('<?xml version="1.0" ?>')
        self.append("<!-- Generated using onshape-to-robot -->")
        if self.config:
            self.append(f"<!-- OnShape {self.config.printable_version()} -->")
        self.append(f'<mujoco model="{robot.name}">')
        self.append(f'<compiler angle="radian" meshdir="." />')
        self.append(f'<option noslip_iterations="1"></option>')

        # Boilerplate
        self.append("<default>")
        self.append('<joint frictionloss="0.1" armature="0.005"/>')
        self.append('<position inheritrange="1" kp="75" kv="5"/>')
        self.append('<default class="visual">')
        self.append('<geom type="mesh" contype="0" conaffinity="0" group="2"/>')
        self.append("</default>")
        self.append('<default class="collision">')
        self.append('<geom group="3"/>')
        self.append("</default>")
        self.append("</default>")

        self.append("<worldbody>")

        self.add_link(robot, robot.get_base_link())

        if self.additional_xml:
            self.append(self.additional_xml)

        self.append("</worldbody>")

        self.append("<asset>")
        for mesh_file in self.meshes:
            entry = self.meshes[mesh_file]
            color_str = "%.20g %.20g %.20g 1" % tuple(entry["color"])
            self.append(
                f'<material name="{entry["material_name"]}" rgba="{color_str}" />'
            )
            self.append(f'<mesh file="{mesh_file}" />')
        self.append("</asset>")

        self.append("</mujoco>")

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
        # Retrieving mesh file and material name
        mesh_file = os.path.basename(part.mesh_file)
        mesh_file_no_ext = ".".join(mesh_file.split(".")[:-1])
        material_name = mesh_file_no_ext + "_material"

        # Relative frame
        T_link_part = np.linalg.inv(T_world_link) @ part.T_world_part

        # Adding the geom node
        self.append(f"<!-- Mesh {part.name} -->")
        geom = f'<geom type="mesh" class="{node}" '
        geom += self.pos_quat(T_link_part) + " "
        geom += f'mesh="{xml_escape(mesh_file_no_ext)}" '
        geom += f'material="{xml_escape(material_name)}" '
        geom += " />"

        # Adding the mesh and material to appear in the assets section
        self.meshes[mesh_file] = {
            "mesh": mesh_file,
            "material_name": material_name,
            "color": part.color,
        }

        self.append(geom)

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
            # self.add_shapes(part, node, T_world_link)
            ...
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

        limits = ""
        if joint.max_effort is not None:
            limits += 'effort="%.20g" ' % joint.max_effort
        if joint.max_velocity is not None:
            limits += 'velocity="%.20g" ' % joint.max_velocity
        if joint.limits is not None:
            limits += 'lower="%.20g" upper="%.20g" ' % joint.limits

        if limits:
            self.append(f"<limit {limits}/>")

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

    def add_link(self, robot: Robot, link: Link, T_world_link: np.ndarray = np.eye(4)):
        """
        Adds a link recursively to the URDF file
        """
        self.append(f"<!-- Link {link.name} -->")
        self.append(f'<body name="{link.name}">')

        if link == robot.get_base_link():
            self.append('<freejoint name="root" />')

        # # Adding inertial properties
        # mass, com, inertia = link.get_dynamics(T_world_link)
        # self.add_inertial(mass, com, inertia)

        # Adding geometry objects
        for part in link.parts:
            self.append(f"<!-- Part {part.name} -->")
            self.add_geometries(
                part,
                T_world_link,
                "visual",
                "collision" if self.draw_collisions else "visual",
            )
            # self.add_geometries(part, T_world_link, "collision", "collision")

        # # Adding frames attached to current link
        # for frame, T_world_frame in link.frames.items():
        #     self.add_frame(link, frame, T_world_link, T_world_frame)

        # # Adding joints and children links
        # for joint in robot.get_link_joints(link):
        #     self.add_link(robot, joint.child, joint.T_world_joint)
        #     self.add_joint(joint, T_world_link)

        self.append("</body>")

    def pos_quat(self, matrix: np.ndarray) -> str:
        """
        Turn a transformation matrix into 'pos="..." quat="..."' attributes
        """
        pos = matrix[:3, 3]
        quat = mat2quat(matrix[:3, :3])
        xml = 'pos="%.20g %.20g %.20g" quat="%.20g %.20g %.20g %.20g"' % (*pos, *quat)

        return xml

    def write_xml(self, robot: Robot, filename: str) -> str:
        scene_xml: str = (
            os.path.dirname(os.path.realpath(__file__)) + "/assets/scene.xml"
        )
        scene_xml = open(scene_xml, "r").read()
        super().write_xml(robot, filename)

        dirname = os.path.dirname(filename)
        with open(dirname + "/scene.xml", "w") as file:
            file.write(scene_xml)
            print(success(f"* Writing {dirname}/scene.xml"))
