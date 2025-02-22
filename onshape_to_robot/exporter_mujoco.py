import numpy as np
import os
from .message import success, warning
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
        self.meshes: list = []
        self.materials: dict = {}
        self.collisions_no_mesh: bool = False

        if config is not None:
            self.no_dynamics = config.no_dynamics
            self.collisions_no_mesh: bool = config.get("collisions_no_mesh", False)
            self.draw_collisions: bool = config.get("draw_collisions", False)
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
        self.append(f'<mujoco model="{robot.name}">')
        self.append(f'<compiler angle="radian" meshdir="." autolimits="true" />')
        self.append(f'<option noslip_iterations="1"></option>')

        if self.additional_xml:
            self.append(self.additional_xml)

        # Boilerplate
        self.default_class = robot.name
        self.append("<default>")
        self.append(f'<default class="{self.default_class}">')
        self.append('<joint frictionloss="0.1" armature="0.005"/>')
        self.append('<position kp="50" dampratio="1" />')
        self.append('<default class="visual">')
        self.append('<geom type="mesh" contype="0" conaffinity="0" group="2"/>')
        self.append("</default>")
        self.append('<default class="collision">')
        self.append('<geom group="3"/>')
        self.append("</default>")
        self.append("</default>")
        self.append("</default>")

        # Adding robot links
        self.append("<worldbody>")
        for base_link in robot.base_links:
            self.add_link(robot, base_link)
        self.append("</worldbody>")

        # Asset (mesh & materials)
        self.append("<asset>")
        for mesh_file in set(self.meshes):
            self.append(f'<mesh file="{mesh_file}" />')
        for material_name, color in self.materials.items():
            color_str = "%g %g %g 1" % tuple(color)
            self.append(f'<material name="{material_name}" rgba="{color_str}" />')
        self.append("</asset>")

        # Adding actuators
        self.add_actuators(robot)

        # Adding equalities (loop closure)
        self.add_equalities(robot)

        self.append("</mujoco>")

        return self.xml

    def add_actuators(self, robot: Robot):
        self.append("<actuator>")

        for joint in robot.joints:
            if joint.joint_type == "fixed":
                continue

            if joint.properties.get("actuated", True):
                type = joint.properties.get("type", "position")
                actuator: str = f'<{type} class="{self.default_class}" name="{joint.name}" joint="{joint.name}" '

                for key in "class", "kp", "kv", "dampratio":
                    if key in joint.properties:
                        actuator += f'{key}="{joint.properties[key]}" '

                if "forcerange" in joint.properties:
                    actuator += f'forcerange="-{joint.properties["forcerange"]} {joint.properties["forcerange"]}" '

                joint_limits = joint.properties.get("limits", joint.limits)
                limits_are_set = joint.properties.get("limits", False) != False
                if joint_limits and (type == "position" or limits_are_set):
                    actuator += f'ctrlrange="{joint_limits[0]} {joint_limits[1]}" '

                actuator += "/>"
                self.append(actuator)

        self.append("</actuator>")

    def add_equalities(self, robot: Robot):
        self.append("<equality>")
        for type, frame1, frame2 in robot.closures:
            if type == "fixed":
                self.append(f'<weld site1="{frame1}" site2="{frame2}" />')
            elif type == "point":
                self.append(f'<connect site1="{frame1}" site2="{frame2}" />')
            else:
                raise ValueError(f"Unknown closure type: {type}")

        self.append("</equality>")

    def add_inertial(self, mass: float, com: np.ndarray, inertia: np.ndarray):
        # Ensuring epsilon masses and inertias
        mass = max(1e-9, mass)
        inertia[0, 0] = max(1e-9, inertia[0, 0])
        inertia[1, 1] = max(1e-9, inertia[1, 1])
        inertia[2, 2] = max(1e-9, inertia[2, 2])

        # Populating body inertial properties
        # https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-inertial
        inertial: str = "<inertial "
        inertial += 'pos="%g %g %g" ' % tuple(com)
        inertial += 'mass="%g" ' % mass
        inertial += 'fullinertia="%g %g %g %g %g %g" ' % (
            inertia[0, 0],
            inertia[1, 1],
            inertia[2, 2],
            inertia[0, 1],
            inertia[0, 2],
            inertia[1, 2],
        )
        inertial += " />"
        self.append(inertial)

    def add_mesh(self, part: Part, class_: str, T_world_link: np.ndarray):
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
        geom = f'<geom type="mesh" class="{class_}" '
        geom += self.pos_quat(T_link_part) + " "
        geom += f'mesh="{xml_escape(mesh_file_no_ext)}" '
        geom += f'material="{xml_escape(material_name)}" '
        geom += " />"

        # Adding the mesh and material to appear in the assets section
        self.meshes.append(mesh_file)
        self.materials[material_name] = part.color

        self.append(geom)

    def add_shapes(self, part: Part, class_: str, T_world_link: np.ndarray):
        """
        Add pure shape geometry.
        """
        for shape in part.shapes:
            geom = f'<geom class="{class_}" '

            T_link_shape = (
                np.linalg.inv(T_world_link) @ part.T_world_part @ shape.T_part_shape
            )
            geom += self.pos_quat(T_link_shape) + " "

            if isinstance(shape, Box):
                geom += 'type="box" size="%g %g %g" ' % tuple(shape.size / 2)
            elif isinstance(shape, Cylinder):
                geom += 'type="cylinder" size="%g %g" ' % (
                    shape.radius,
                    shape.length / 2,
                )
            elif isinstance(shape, Sphere):
                geom += 'type="sphere" size="%g" ' % shape.radius

            if class_ == "visual":
                material_name = f"{part.name}_material"
                self.materials[material_name] = part.color
                geom += f'material="{xml_escape(material_name)}" '

            geom += " />"
            self.append(geom)

    def add_geometries(
        self, part: Part, T_world_link: np.ndarray, class_: str, what: str
    ):
        """
        Add geometry nodes. "class_" is the class that will be used, "what" is the logic used to produce it.
        Both can be "visual" or "collision"
        """
        if what == "collision" and part.shapes is not None:
            self.add_shapes(part, class_, T_world_link)
        elif part.mesh_file and (what == "visual" or not self.collisions_no_mesh):
            self.add_mesh(part, class_, T_world_link)

    def add_joint(self, joint: Joint):
        self.append(f"<!-- Joint from {joint.parent.name} to {joint.child.name} -->")
        if joint.joint_type == "fixed":
            self.append(f'<!-- Joint is "fixed", skipping it -->')
            return

        joint_xml: str = "<joint "
        joint_xml += f'name="{joint.name}" '
        if joint.joint_type == Joint.REVOLUTE:
            joint_xml += 'type="hinge" '
        elif joint.joint_type == Joint.PRISMATIC:
            joint_xml += 'type="slide" '

        if joint.limits is not None and joint.properties.get("range", True):
            joint_xml += f'range="{joint.limits[0]} {joint.limits[1]}" '

        for key in (
            "class",
            "frictionloss",
            "armature",
            "damping",
            "stiffness",
        ):
            if key in joint.properties:
                joint_xml += f'{key}="{joint.properties[key]}" '

        joint_xml += " />"
        self.append(joint_xml)

    def add_frame(
        self,
        link: Link,
        frame: str,
        T_world_link: np.ndarray,
        T_world_frame: np.ndarray,
    ):
        self.append(f"<!-- Frame {frame} (dummy link + fixed joint) -->")
        T_link_frame = np.linalg.inv(T_world_link) @ T_world_frame

        site: str = f'<site name="{frame}" '
        site += self.pos_quat(T_link_frame) + " "
        site += " />"
        self.append(site)

    def add_link(
        self,
        robot: Robot,
        link: Link,
        parent_joint: Joint | None = None,
        T_world_parent: np.ndarray = np.eye(4),
    ):
        """
        Adds a link recursively to the URDF file
        """
        if parent_joint is None:
            T_world_link = np.eye(4)
        else:
            T_world_link = parent_joint.T_world_joint

        childclass = ""
        if parent_joint is None:
            childclass = f'childclass="{self.default_class}" '
        self.append(f"<!-- Link {link.name} -->")
        T_parent_link = np.linalg.inv(T_world_parent) @ T_world_link
        self.append(f'<body name="{link.name}" {self.pos_quat(T_parent_link)} {childclass}>')

        if parent_joint is None:
            if not link.fixed:
                self.append('<freejoint name="root" />')
        else:
            self.add_joint(parent_joint)

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

        # Adding frames attached to current link
        for frame, T_world_frame in link.frames.items():
            self.add_frame(link, frame, T_world_link, T_world_frame)

        # Adding joints and children links
        for joint in robot.get_link_joints(link):
            self.add_link(robot, joint.child, joint, T_world_link)

        self.append("</body>")

    def pos_quat(self, matrix: np.ndarray) -> str:
        """
        Turn a transformation matrix into 'pos="..." quat="..."' attributes
        """
        pos = matrix[:3, 3]
        quat = mat2quat(matrix[:3, :3])
        xml = 'pos="%g %g %g" quat="%g %g %g %g"' % (*pos, *quat)

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
            print(success(f"* Writing scene.xml"))
