import numpy as np
import os
from .message import success
from .robot import Robot, Link, Part, Joint
from .config import Config
from .shapes import Box, Cylinder, Sphere
from .exporter import Exporter
from .exporter_utils import xml_escape, rotation_matrix_to_rpy

MODEL_CONFIG_XML = """
<?xml version="1.0" ?>
<model>
    <name>%s</name>
    <version>1.0</version>
    <sdf version="1.7">%s</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
"""


class ExporterSDF(Exporter):
    def __init__(self, config: Config | None = None):
        super().__init__()
        self.config: Config = config

        self.ext: str = "sdf"
        self.draw_collisions: bool = False
        self.no_dynamics: bool = False
        self.additional_xml: str = ""
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
        self.append('<sdf version="1.7">')
        self.append(f'<model name="{xml_escape(robot.name)}">')

        for base_link in robot.base_links:
            self.add_link(robot, base_link)

        if self.additional_xml:
            self.append(self.additional_xml)

        self.append("</model>")
        self.append("</sdf>")

        return self.xml

    def add_inertial(
        self, mass: float, com: np.ndarray, inertia: np.ndarray, frame: str = ""
    ):
        # Unless "no_dynamics" is set, we make sure that mass and inertia
        # are not zero
        if not self.no_dynamics:
            mass = max(1e-9, mass)
            inertia[0, 0] = max(1e-9, inertia[0, 0])
            inertia[1, 1] = max(1e-9, inertia[1, 1])
            inertia[2, 2] = max(1e-9, inertia[2, 2])

        self.append("<inertial>")
        self.append(
            '<pose relative_to="%s">%g %g %g 0 0 0</pose>'
            % (
                frame,
                com[0],
                com[1],
                com[2],
            )
        )
        self.append("<mass>%g</mass>" % mass)
        self.append(
            "<inertia><ixx>%g</ixx><ixy>%g</ixy><ixz>%g</ixz><iyy>%g</iyy><iyz>%g</iyz><izz>%g</izz></inertia>"
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

    def append_material(self, color: np.ndarray):
        self.append(f"<material>")
        self.append("<ambient>%g %g %g 1.0</ambient>" % (color[0], color[1], color[2]))
        self.append("<diffuse>%g %g %g 1.0</diffuse>" % (color[0], color[1], color[2]))
        self.append("<specular>0.1 0.1 0.1 1</specular>")
        self.append("<emissive>0 0 0 0</emissive>")
        self.append("</material>")

    def add_mesh(self, link: Link, part: Part, node: str, T_world_link: np.ndarray):
        """
        Add a mesh node (e.g. STL) to the SDF file
        """
        self.append(f'<{node} name="{part.name}_{node}_mesh">')

        T_link_part = np.linalg.inv(T_world_link) @ part.T_world_part
        self.append(self.pose(T_link_part, relative_to=link.name))

        mesh_file = os.path.basename(part.mesh_file)

        self.append("<geometry>")
        self.append(
            f"<mesh><uri>model://{self.config.robot_name}/{xml_escape(mesh_file)}</uri></mesh>"
        )
        self.append("</geometry>")

        if node == "visual":
            self.append_material(part.color)

        self.append(f"</{node}>")

    def add_shapes(self, link: Link, part: Part, node: str, T_world_link: np.ndarray):
        """
        Add shapes (box, sphere and cylinder) nodes to the SDF.
        """
        shape_n = 1
        for shape in part.shapes:
            self.append(f'<{node} name="{part.name}_{node}_shapes_{shape_n}">')
            shape_n += 1

            T_link_shape = (
                np.linalg.inv(T_world_link) @ part.T_world_part @ shape.T_part_shape
            )
            self.append(self.pose(T_link_shape, relative_to=link.name))

            self.append("<geometry>")
            if isinstance(shape, Box):
                self.append("<box><size>%g %g %g</size></box>" % tuple(shape.size))
            elif isinstance(shape, Cylinder):
                self.append(
                    "<cylinder><length>%g</length><radius>%g</radius></cylinder>"
                    % (shape.length, shape.radius)
                )
            elif isinstance(shape, Sphere):
                self.append("<sphere><radius>%g</radius></sphere>" % shape.radius)
            self.append("</geometry>")

            if node == "visual":
                self.append_material(part.color)

            self.append(f"</{node}>")

    def add_geometries(
        self, link: Link, part: Part, T_world_link: np.ndarray, node: str, what: str
    ):
        """
        Add geometry nodes. "node" is the actual XML node produced, "what" is the logic used to produce it.
        Both can be "visual" or "collision"
        """
        if what == "collision" and part.shapes is not None:
            self.add_shapes(link, part, node, T_world_link)
        elif part.mesh_file and (what == "visual" or not self.collisions_no_mesh):
            self.add_mesh(link, part, node, T_world_link)

    def add_joint(self, joint: Joint, T_world_link: np.ndarray):
        self.append(f"<!-- Joint from {joint.parent.name} to {joint.child.name} -->")

        joint_type = joint.properties.get("type", joint.joint_type)
        self.append(f'<joint name="{joint.name}" type="{joint_type}">')

        T_link_joint = np.linalg.inv(T_world_link) @ joint.T_world_joint
        self.append(self.pose(T_link_joint, relative_to=joint.parent.name))

        self.append(f"<parent>{joint.parent.name}</parent>")
        self.append(f"<child>{joint.child.name}</child>")
        self.append("<axis>")
        self.append("<xyz>%g %g %g</xyz>" % tuple(joint.z_axis))

        self.append("<limit>")
        if "max_effort" in joint.properties:
            self.append(f"<effort>%g</effort>" % joint.properties["max_effort"])
        else:
            self.append(f"<effort>10</effort>")

        if "max_velocity" in joint.properties:
            self.append(f"<velocity>%g</velocity>" % joint.properties["max_velocity"])
        else:
            self.append(f"<velocity>10</velocity>")
        self.append("</limit>")

        joint_limits = joint.properties.get("limits", joint.limits)
        if joint_limits is None:
            if joint_type == "revolute":
                joint_limits = [-np.pi, np.pi]
            if joint_type == "prismatic":
                joint_limits = [-1, 1]

        self.append(f"<lower>{joint_limits[0]}</lower>")
        self.append(f"<upper>{joint_limits[1]}</upper>")
        self.append("</axis>")

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

        self.append(f'<frame name="{frame}">')
        self.append(self.pose(T_link_frame, relative_to=link.name))
        self.append("</frame>")

    def add_link(self, robot: Robot, link: Link, joint: Joint = None):
        """
        Adds a link recursively to the SDF file
        """
        self.append(f"<!-- Link {link.name} -->")
        self.append(f'<link name="{link.name}">')

        T_world_link = np.eye(4)
        if joint is not None:
            T_world_link = joint.T_world_joint

        # Adding inertial properties
        mass, com, inertia = link.get_dynamics(T_world_link)
        self.add_inertial(mass, com, inertia, link.name)

        relative_to = ""
        if joint is not None:
            relative_to = joint.name
        self.append(self.pose(np.eye(4), relative_to=relative_to))

        # Adding geometry objects
        for part in link.parts:
            self.append(f"<!-- Part {part.name} -->")
            self.add_geometries(
                link,
                part,
                T_world_link,
                "visual",
                "collision" if self.draw_collisions else "visual",
            )
            self.add_geometries(link, part, T_world_link, "collision", "collision")

        self.append("</link>")

        if link.fixed:
            self.append(f'<joint name="{link.name}_fixed_world" type="fixed">')
            self.append(f"<parent>world</parent>")
            self.append(f"<child>{link.name}</child>")
            self.append("</joint>")

        # Adding frames attached to current link
        for frame, T_world_frame in link.frames.items():
            self.add_frame(link, frame, T_world_link, T_world_frame)

        # Adding joints and children links
        for joint in robot.get_link_joints(link):
            self.add_link(robot, joint.child, joint)
            self.add_joint(joint, T_world_link)

    def pose(self, matrix: np.ndarray, relative_to: str = ""):
        """
        Transforms a transformation matrix into a SDF pose tag
        """
        relative = ""
        if relative_to:
            relative = f' relative_to="{relative_to}"'

        sdf = "<pose%s>%g %g %g %g %g %g</pose>"

        return sdf % (relative, *matrix[:3, 3], *rotation_matrix_to_rpy(matrix))

    def write_xml(self, robot: Robot, filename: str) -> str:
        model_config = MODEL_CONFIG_XML % (robot.name, os.path.basename(filename))

        super().write_xml(robot, filename)

        dirname = os.path.dirname(filename)
        with open(dirname + "/model.config", "w") as file:
            file.write(model_config)
            print(success(f"* Writing model.config"))
