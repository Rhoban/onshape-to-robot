from __future__ import annotations
import numpy as np
import os
import fnmatch
from .message import success, warning, info
from .robot import Robot, Link, Part, Joint, Closure
from .config import Config
from .geometry import Box, Cylinder, Sphere, Mesh, Shape
from .exporter import Exporter
from .exporter_utils import xml_escape, rotation_matrix_to_rpy
from transforms3d.quaternions import mat2quat


class ExporterMuJoCo(Exporter):
    def __init__(self, config: Config | None = None):
        super().__init__()
        self.config: Config = config

        self.no_dynamics: bool = False
        self.additional_xml: str = ""
        self.meshes: list = []
        self.materials: dict = {}

        if config is not None:
            self.equalities = self.config.get("equalities", {})
            self.no_dynamics = config.no_dynamics
            additional_xml_file = config.get("additional_xml", None, required=False)
            if isinstance(additional_xml_file, str):
                self.add_additional_xml(additional_xml_file)
            elif isinstance(additional_xml_file, list):
                for filename in additional_xml_file:
                    self.add_additional_xml(filename)

    def add_additional_xml(self, xml_file: str):
        self.additional_xml += f"<!-- Additional {xml_file} -->"
        with open(self.config.output_directory + "/" + xml_file, "r") as file:
            self.additional_xml += file.read()

    def append(self, line: str):
        self.xml += line

    def build(self, robot: Robot):
        self.xml = ""
        self.append('<?xml version="1.0" ?>')
        self.append("<!-- Generated using onshape-to-robot -->")
        if self.config:
            self.append(f"<!-- Onshape {self.config.printable_version()} -->")
        self.append(f'<mujoco model="{robot.name}">')
        self.append(
            f'<compiler angle="radian" meshdir="{self.config.assets_directory}" autolimits="true" />'
        )

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

        if self.additional_xml:
            self.append(self.additional_xml)

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

            # Suppose joints with relation equality is not actuated, unless specified
            guess_actuated = joint.relation is None

            if (
                joint.properties.get("actuated", guess_actuated)
                and joint.joint_type != Joint.BALL
            ):
                type = joint.properties.get("type", "position")
                actuator_class = joint.properties.get("class", self.default_class)
                actuator: str = (
                    f'<{type} class="{actuator_class}" name="{joint.name}" joint="{joint.name}" '
                )

                for key in "kp", "kv", "dampratio":
                    if key in joint.properties:
                        actuator += f'{key}="{joint.properties[key]}" '

                if "forcerange" in joint.properties:
                    actuator += f'forcerange="-{joint.properties["forcerange"]} {joint.properties["forcerange"]}" '

                joint_limits = joint.properties.get("limits", joint.limits)
                limits_are_set = joint.properties.get("limits", False) != False
                if joint_limits and (type == "position" or limits_are_set):
                    if joint.properties.get("range", True) and type == "position":
                        actuator += f'inheritrange="1" '
                    else:
                        actuator += f'ctrlrange="{joint_limits[0]} {joint_limits[1]}" '

                actuator += "/>"
                self.append(actuator)

        self.append("</actuator>")

    def get_equality_attributes(self, closure: Closure) -> str:
        all_attributes = {}
        for name, attributes in self.equalities.items():
            if fnmatch.fnmatch(closure.frame1, name) and fnmatch.fnmatch(
                closure.frame2, name
            ):
                all_attributes.update(attributes)

        if len(all_attributes) > 0:
            return (
                " ".join([f'{key}="{value}"' for key, value in all_attributes.items()])
                + " "
            )

        return ""

    def add_equalities(self, robot: Robot):
        self.append("<equality>")
        for closure in robot.closures:
            attributes = self.get_equality_attributes(closure)

            if closure.closure_type == Closure.FIXED:
                self.append(
                    f'<weld site1="{closure.frame1}" site2="{closure.frame2}" {attributes}/>'
                )
            elif closure.closure_type == Closure.REVOLUTE:
                self.append(
                    f'<connect site1="{closure.frame1}" site2="{closure.frame2}" {attributes}/>'
                )
            elif closure.closure_type == Closure.BALL:
                self.append(
                    f'<connect site1="{closure.frame1}" site2="{closure.frame2}" {attributes}/>'
                )
            else:
                print(
                    warning(
                        f"Closure type: {closure.closure_type} is not supported with MuJoCo equality constraints"
                    )
                )

        for joint in robot.joints:
            if joint.relation is not None:
                self.append(
                    f'<joint joint1="{joint.name}" joint2="{joint.relation.source_joint}" polycoef="0 {joint.relation.ratio} 0 0 0" />'
                )

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

    def add_mesh(self, part: Part, class_: str, T_world_link: np.ndarray, mesh: Mesh):
        """
        Add a mesh node (e.g. STL) to the MuJoCo file
        """
        # Retrieving mesh file and material name
        mesh_file = os.path.relpath(mesh.filename, self.config.asset_path(""))
        mesh_file_no_ext = ".".join(os.path.basename(mesh_file).split(".")[:-1])
        material_name = mesh_file_no_ext + "_material"

        # Relative frame
        T_link_part = np.linalg.inv(T_world_link) @ part.T_world_part

        # Adding the geom node
        geom = f'<geom type="mesh" class="{class_}" '
        geom += self.pos_quat(T_link_part) + " "
        geom += f'mesh="{xml_escape(mesh_file_no_ext)}" '
        geom += f'material="{xml_escape(material_name)}" '
        geom += " />"

        # Adding the mesh and material to appear in the assets section
        self.meshes.append(mesh_file)
        self.materials[material_name] = mesh.color

        self.append(geom)

    def add_shape(
        self, part: Part, class_: str, T_world_link: np.ndarray, shape: Shape
    ):
        """
        Add pure shape geometry.
        """
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
            self.materials[material_name] = shape.color
            geom += f'material="{xml_escape(material_name)}" '

        geom += " />"
        self.append(geom)

    def add_geometries(self, part: Part, T_world_link: np.ndarray):
        """
        Add a part geometries
        """
        for shape in part.shapes:
            if shape.visual:
                self.add_shape(part, "visual", T_world_link, shape)
            if shape.collision:
                self.add_shape(part, "collision", T_world_link, shape)

        for mesh in part.meshes:
            if mesh.visual:
                self.add_mesh(part, "visual", T_world_link, mesh)
            if mesh.collision:
                self.add_mesh(part, "collision", T_world_link, mesh)

    def add_joint(self, joint: Joint):
        self.append(f"<!-- Joint from {joint.parent.name} to {joint.child.name} -->")
        if joint.joint_type == "fixed":
            self.append(f'<!-- Joint is "fixed", skipping it -->')
            return

        joint_xml: str = "<joint "
        joint_xml += 'axis="%g %g %g" ' % tuple(joint.axis)
        joint_xml += f'name="{joint.name}" '
        if joint.joint_type == Joint.REVOLUTE:
            joint_xml += 'type="hinge" '
        elif joint.joint_type == Joint.PRISMATIC:
            joint_xml += 'type="slide" '
        elif joint.joint_type == Joint.BALL:
            joint_xml += 'type="ball" '
        else:
            print(warning(f"Unknown joint type: {joint.joint_type}"))

        joint_limits = joint.properties.get("limits", joint.limits)
        if joint_limits is not None and joint.properties.get("range", True):
            joint_xml += f'range="{joint_limits[0]} {joint_limits[1]}" '

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
        frame: str,
        T_world_link: np.ndarray,
        T_world_frame: np.ndarray,
        group: int = 0,
    ):
        self.append(f"<!-- Frame {frame} -->")
        T_link_frame = np.linalg.inv(T_world_link) @ T_world_frame

        site: str = f'<site group="{group}" name="{frame}" '
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
        self.append(
            f'<body name="{link.name}" {self.pos_quat(T_parent_link)} {childclass}>'
        )

        if parent_joint is None:
            if not link.fixed:
                self.append(f'<freejoint name="{link.name}_freejoint" />')
        else:
            self.add_joint(parent_joint)

        # Adding inertial properties
        mass, com, inertia = link.get_dynamics(T_world_link)
        self.add_inertial(mass, com, inertia)

        # Adding geometry objects
        for part in link.parts:
            self.append(f"<!-- Part {part.name} -->")
            self.add_geometries(part, T_world_link)

        # Adding frames attached to current link
        for frame, T_world_frame in link.frames.items():
            self.add_frame(frame, T_world_link, T_world_frame, group=3)

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
        super().write_xml(robot, filename)

        dirname = os.path.dirname(filename)
        scene_filename = dirname + "/scene.xml"
        if not os.path.exists(scene_filename):
            scene_xml: str = (
                os.path.dirname(os.path.realpath(__file__)) + "/assets/scene.xml"
            )
            scene_xml = open(scene_xml, "r").read()
            scene_xml = scene_xml.format(robot_filename=os.path.basename(filename))
            with open(scene_filename, "w") as file:
                file.write(scene_xml)
                print(success(f"* Writing scene.xml"))
        else:
            print(info(f"* scene.xml already exists, not over-writing it"))
