import numpy as np
import os
import hashlib
import json
from .message import warning, info, success, error, dim, bright
from .assembly import Assembly, INSTANCE_ROOT
from .config import Config
from .robot_description import RobotURDF, RobotSDF, RobotDescription
from .csg import process as csg_process


class RobotBuilder:
    def __init__(self, config: Config, assembly: Assembly):
        self.config: Config = config
        self.assembly: Assembly = assembly
        self.robot: RobotDescription = None
        self.part_names = {}

        self.make_robot()
        self.build_robot(INSTANCE_ROOT)
        self.export_robot()

    def export_robot(self):
        """
        Export the robot to the output file
        """
        self.robot.finalize()
        print(bright(f"* Writing {self.robot.ext.upper()} file"))

        with open(
            self.config.output_directory + "/robot." + self.robot.ext,
            "w",
            encoding="utf-8",
        ) as stream:
            stream.write(self.robot.xml)

        if len(self.config.post_import_commands):
            print(bright(f"* Executing post-import commands"))
            for command in self.config.post_import_commands:
                print("* " + command)
                os.system(command)

    def part_is_ignored(self, name: str) -> bool:
        """
        Checks if a given part should be ignored by config
        """
        if self.config.whitelist is None:
            return name in self.config.ignore
        else:
            return name not in self.config.whitelist

    def slugify(self, value: str) -> str:
        """
        Turns a value into a slug
        """
        return "".join(c if c.isalnum() else "_" for c in value).strip("_")

    def make_robot(self) -> RobotDescription:
        """
        Create the robot description
        """
        # Creating robot for output
        if self.config.output_format == "urdf":
            self.robot = RobotURDF(self.config.robot_name)
        elif self.config.output_format == "sdf":
            self.robot = RobotSDF(self.config.robot_name)
        else:
            raise Exception(
                f"ERROR: Unknown output format: {self.config.output_format} (supported are urdf and sdf)"
            )

        self.robot.drawCollisions = self.config.draw_collisions
        self.robot.jointMaxEffort = self.config.joint_max_effort
        self.robot.jointMaxVelocity = self.config.joint_max_velocity
        self.robot.mergeSTLs = self.config.merge_stls
        self.robot.maxSTLSize = self.config.max_stl_size
        self.robot.simplifySTLs = self.config.simplify_stls
        self.robot.noDynamics = self.config.no_dynamics
        self.robot.packageName = self.config.package_name
        self.robot.addDummyBaseLink = self.config.add_dummy_base_link
        self.robot.robotName = self.config.robot_name
        self.robot.additionalXML = self.config.additional_xml
        self.robot.useFixedLinks = self.config.use_fixed_links
        self.robot.meshDir = self.config.output_directory

    def part_name(self, part: dict):
        """
        Retrieve the name from a part.
        i.e "Base link <1>" -> "base_link"
        """
        name, configuration = part["name"], part["configuration"]
        parts = name.split(" ")
        del parts[-1]
        base_part_name = "_".join(parts).lower()

        # Only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        if configuration != "default":
            if len(configuration) < 40:
                parts += ["_" + configuration.replace("=", "_").replace(" ", "_")]
            else:
                parts += ["_" + hashlib.md5(configuration.encode("utf-8")).hexdigest()]

        return base_part_name, "_".join(parts).lower()

    def unique_part_name(self, part: dict):
        """
        Get unique part name (plate, plate_2, plate_3, ...)
        In the case where multiple parts have the same name in OnShape, they will result in different names in the URDF
        """
        _, name = self.part_name(part)

        if name in self.part_names:
            self.part_names[name] += 1
            return f"{name}_{self.part_names[name]}"
        else:
            self.part_names[name] = 1
            return name

    def get_stl(self, prefix: str, instance: dict) -> str:
        """
        Download and store STL file
        """
        filename = self.slugify(prefix) + ".stl"

        stl = self.assembly.client.part_studio_stl_m(
            instance["documentId"],
            instance["documentMicroversion"],
            instance["elementId"],
            instance["partId"],
            instance["configuration"],
        )
        with open(self.config.output_directory + "/" + filename, "wb") as stream:
            stream.write(stl)

        # Storing metadata for imported instances in the .part file
        stl_metadata = self.slugify(prefix) + ".part"
        with open(
            self.config.output_directory + "/" + stl_metadata, "w", encoding="utf-8"
        ) as stream:
            json.dump(instance, stream, indent=4, sort_keys=True)

        return self.config.output_directory + "/" + filename

    def get_color(self, instance: dict) -> np.ndarray:
        """
        Retrieve the color of a part
        """
        if self.config.color is not None:
            color = self.config.color
        else:
            metadata = self.assembly.client.part_get_metadata(
                instance["documentId"],
                instance["documentMicroversion"],
                instance["elementId"],
                instance["partId"],
                instance["configuration"],
            )

            color = [0.5, 0.5, 0.5]

            # XXX: There must be a better way to retrieve the part color
            for entry in metadata["properties"]:
                if (
                    "value" in entry
                    and type(entry["value"]) is dict
                    and "color" in entry["value"]
                ):
                    rgb = entry["value"]["color"]
                    color = np.array([rgb["red"], rgb["green"], rgb["blue"]]) / 255.0

        return color

    def get_dynamics(self, instance: dict, prefix: str) -> tuple:
        """
        Retrieve the dynamics (mass, com, inertia) of a given instance
        """
        if self.config.no_dynamics:
            mass = 0
            com = [0] * 3
            inertia = [0] * 12
        else:
            if prefix in self.config.dynamics_override:
                entry = self.config.dynamics_override[prefix]
                mass = entry["mass"]
                com = entry["com"]
                inertia = entry["inertia"]
            else:
                if instance["isStandardContent"]:
                    mass_properties = (
                        self.assembly.client.standard_cont_mass_properties(
                            instance["documentId"],
                            instance["documentVersion"],
                            instance["elementId"],
                            instance["partId"],
                            self.config.document_id,
                            instance["configuration"],
                        )
                    )
                else:
                    mass_properties = self.assembly.client.part_mass_properties(
                        instance["documentId"],
                        instance["documentMicroversion"],
                        instance["elementId"],
                        instance["partId"],
                        instance["configuration"],
                    )

                if instance["partId"] not in mass_properties["bodies"]:
                    print(
                        warning(
                            f"WARNING: part {instance['name']} has no dynamics (maybe it is a surface)"
                        )
                    )
                    return
                mass_properties = mass_properties["bodies"][instance["partId"]]
                mass = mass_properties["mass"][0]
                com = mass_properties["centroid"]
                inertia = mass_properties["inertia"]

                if abs(mass) < 1e-9:
                    print(
                        warning(
                            f"WARNING: part {instance['name']} has no mass, maybe you should assign a material to it ?"
                        )
                    )
        return mass, com, inertia

    def add_part(self, occurrence: dict, T_world_link: np.ndarray):
        """
        Add a part to the current link
        """
        instance = occurrence["instance"]

        if instance["suppressed"]:
            return

        if instance["partId"] == "":
            print(warning(f"WARNING: Part '{instance['name']}' has no partId"))
            return

        part_name, prefix = self.part_name(instance)
        extra = ""
        if instance["configuration"] != "default":
            extra = dim(" (configuration: " + instance["configuration"] + ")")
        symbol = "+"
        if self.part_is_ignored(part_name):
            symbol = "-"
            extra += dim(" / ignoring visual and collision")

        print(success(f"{symbol} Adding part {instance['name']}{extra}"))

        if self.part_is_ignored(part_name):
            stl_file = None
        else:
            stl_file = self.get_stl(prefix, instance)

        # Import the SCAD files pure shapes
        shapes = None
        if self.config.use_scads:
            scad_file = prefix + ".scad"
            if os.path.exists(self.config.output_directory + "/" + scad_file):
                shapes = csg_process(
                    self.config.output_directory + "/" + scad_file,
                    self.config.pure_shape_dilatation,
                )

        # Obtain metadatas about part to retrieve color
        color = self.get_color(instance)

        # Obtain the instance dynamics
        mass, com, inertia = self.get_dynamics(instance, prefix)

        T_world_part = np.array(occurrence["transform"]).reshape(4, 4)

        self.robot.addPart(
            np.linalg.inv(T_world_link) @ T_world_part,
            stl_file,
            mass,
            com,
            inertia,
            color,
            shapes,
            prefix,
        )

    def build_robot(self, body_id: int, T_world_link: np.ndarray = np.eye(4)):
        """
        Add recursively body nodes to the robot description.
        """
        instance = self.assembly.body_instance(body_id)
        link_name = self.unique_part_name(instance)

        # Adding all the parts in the current link
        self.robot.startLink(link_name, T_world_link)
        for occurrence in self.assembly.body_occurrences(body_id):
            if occurrence["instance"]["type"] == "Part":
                self.add_part(occurrence, T_world_link)
        self.robot.endLink()

        # Adding frames to the link
        for frame in self.assembly.frames:
            if frame.body_id == body_id:
                T_world_frame = frame.T_world_frame
                T_link_frame = np.linalg.inv(T_world_link) @ T_world_frame

                self.robot.addFrame(frame.name, T_link_frame)

        for children_body in self.assembly.tree_children[body_id]:
            dof = self.assembly.get_dof(body_id, children_body)
            child_body = dof.other_body(body_id)
            T_world_axis = dof.T_world_mate.copy()

            if self.robot.relative:
                child_link_name = self.build_robot(child_body, T_world_axis)
            else:
                child_link_name = self.build_robot(child_body, T_world_link)

            self.robot.addJoint(
                dof.joint_type,
                link_name,
                child_link_name,
                np.linalg.inv(T_world_link) @ T_world_axis,
                dof.name,
                dof.limits,
                dof.z_axis,
            )

        return link_name
