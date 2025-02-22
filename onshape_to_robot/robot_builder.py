import numpy as np
import os
import hashlib
import json
import fnmatch
from .message import warning, info, success, error, dim, bright
from .assembly import Assembly
from .config import Config
from .robot import Part, Joint, Link, Robot
from .csg import process as csg_process


class RobotBuilder:
    def __init__(self, config: Config):
        self.config: Config = config
        self.assembly: Assembly = Assembly(config)
        self.robot: Robot = Robot(config.robot_name)
        self.robot.closures = self.assembly.closures
        self.unique_names = {}

        for node in self.assembly.root_nodes:
            link = self.build_robot(node)
            self.robot.base_links.append(link)

    def part_is_ignored(self, name: str) -> bool:
        """
        Checks if a given part should be ignored by config
        """
        if self.config.whitelist is None:
            for entry in self.config.ignore:
                if fnmatch.fnmatch(name, entry):
                    return True
            return False
        else:
            for entry in self.config.whitelist:
                if fnmatch.fnmatch(name, entry):
                    return False
            return True

    def slugify(self, value: str) -> str:
        """
        Turns a value into a slug
        """
        return "".join(c if c.isalnum() else "_" for c in value).strip("_")

    def printable_configuration(self, instance: dict) -> str:
        """
        Retrieve configuration enums to replace "List_..." with proper enum names
        """
        configuration = instance["configuration"]

        if instance["configuration"] != "default":
            if "documentVersion" in instance:
                version = instance["documentVersion"]
                wmv = "v"
            else:
                version = instance["documentMicroversion"]
                wmv = "m"
            elements = self.assembly.client.elements_configuration(
                instance["documentId"],
                version,
                instance["elementId"],
                wmv=wmv,
                linked_document_id=self.config.document_id,
            )
            for entry in elements["configurationParameters"]:
                type_name = entry["typeName"]
                message = entry["message"]

                if type_name.startswith("BTMConfigurationParameterEnum"):
                    parameter_name = message["parameterName"]
                    parameter_id = message["parameterId"]
                    configuration = configuration.replace(parameter_id, parameter_name)

        return configuration

    def part_name(self, part: dict):
        """
        Retrieve the name from a part.
        i.e "Base link <1>" -> "base_link"
        """
        name = part["name"]
        parts = name.split(" ")
        del parts[-1]
        base_part_name = "_".join(parts).lower()

        # Only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        configuration = self.printable_configuration(part)
        if configuration != "default":
            if len(configuration) < 40:
                parts += ["_" + configuration.replace("=", "_").replace(" ", "_")]
            else:
                parts += ["_" + hashlib.md5(configuration.encode("utf-8")).hexdigest()]

        return base_part_name, "_".join(parts).lower()

    def unique_name(self, part: dict, type: str):
        """
        Get unique part name (plate, plate_2, plate_3, ...)
        In the case where multiple parts have the same name in OnShape, they will result in different names in the URDF
        """
        while True:
            _, name = self.part_name(part)

            if type not in self.unique_names:
                self.unique_names[type] = {}

            if name in self.unique_names[type]:
                self.unique_names[type][name] += 1
                name = f"{name}_{self.unique_names[type][name]}"
            else:
                self.unique_names[type][name] = 1
                name = name

            if name not in [frame.name for frame in self.assembly.frames]:
                return name

    def instance_request_params(self, instance: dict) -> dict:
        """
        Build parameters to make an API call for a given instance
        """
        params = {}

        if "documentVersion" in instance:
            params["wmvid"] = instance["documentVersion"]
            params["wmv"] = "v"
        else:
            params["wmvid"] = instance["documentMicroversion"]
            params["wmv"] = "m"

        params["did"] = instance["documentId"]
        params["eid"] = instance["elementId"]
        params["linked_document_id"] = self.config.document_id
        params["configuration"] = instance["configuration"]

        return params

    def get_stl(self, prefix: str, instance: dict) -> str:
        """
        Download and store STL file
        """
        filename = self.slugify(prefix) + ".stl"

        params = self.instance_request_params(instance)
        stl = self.assembly.client.part_studio_stl_m(
            **params,
            partid=instance["partId"],
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
            color = np.array(self.config.color)
        else:
            params = self.instance_request_params(instance)
            metadata = self.assembly.client.part_get_metadata(
                **params,
                partid=instance["partId"],
            )

            color = np.array([0.5, 0.5, 0.5])

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

    def get_dynamics(self, instance: dict, part_name_config: str) -> tuple:
        """
        Retrieve the dynamics (mass, com, inertia) of a given instance
        """
        if self.config.no_dynamics:
            mass = 0
            com = [0] * 3
            inertia = [0] * 12
        else:
            if instance["isStandardContent"]:
                mass_properties = self.assembly.client.standard_cont_mass_properties(
                    instance["documentId"],
                    instance["documentVersion"],
                    instance["elementId"],
                    instance["partId"],
                    configuration=instance["configuration"],
                    linked_document_id=self.config.document_id,
                )
            else:
                params = self.instance_request_params(instance)
                mass_properties = self.assembly.client.part_mass_properties(
                    **params,
                    partid=instance["partId"],
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

        return mass, com[:3], np.reshape(inertia[:9], (3, 3))

    def add_part(self, occurrence: dict):
        """
        Add a part to the current link
        """
        instance = occurrence["instance"]

        if instance["suppressed"]:
            return

        if instance["partId"] == "":
            print(warning(f"WARNING: Part '{instance['name']}' has no partId"))
            return

        part_name, part_name_config = self.part_name(instance)
        extra = ""
        if instance["configuration"] != "default":
            extra = dim(
                " (configuration: " + self.printable_configuration(instance) + ")"
            )
        symbol = "+"
        if self.part_is_ignored(part_name):
            symbol = "-"
            extra += dim(" / ignoring visual and collision")

        print(success(f"{symbol} Adding part {instance['name']}{extra}"))

        if self.part_is_ignored(part_name):
            stl_file = None
        else:
            stl_file = self.get_stl(part_name_config, instance)

        # Obtain metadatas about part to retrieve color
        color = self.get_color(instance)

        # Obtain the instance dynamics
        mass, com, inertia = self.get_dynamics(instance, part_name_config)

        T_world_part = np.array(occurrence["transform"]).reshape(4, 4)

        unique_part_name = self.unique_name(instance, "part")

        part = Part(unique_part_name, T_world_part, stl_file, mass, com, inertia, color)
        self.robot.links[-1].parts.append(part)

    def build_robot(self, body_id: int):
        """
        Add recursively body nodes to the robot description.
        """
        instance = self.assembly.body_instance(body_id)

        if body_id in self.assembly.link_names:
            link_name = self.assembly.link_names[body_id]
        else:
            link_name = self.unique_name(instance, "link")

        # Adding all the parts in the current link
        link = Link(link_name)
        self.robot.links.append(link)
        for occurrence in self.assembly.body_occurrences(body_id):
            if occurrence["instance"]["type"] == "Part":
                self.add_part(occurrence)
            if occurrence["fixed"]:
                link.fixed = True

        # Adding frames to the link
        for frame in self.assembly.frames:
            if frame.body_id == body_id:
                self.robot.links[-1].frames[frame.name] = frame.T_world_frame

        for children_body in self.assembly.tree_children[body_id]:
            dof = self.assembly.get_dof(body_id, children_body)
            child_body = dof.other_body(body_id)
            T_world_axis = dof.T_world_mate.copy()

            child_link = self.build_robot(child_body)

            default_properties = self.config.joint_properties.get("default", {})
            properties = self.config.joint_properties.get(dof.name, {})
            properties = {**default_properties, **properties}

            joint = Joint(
                dof.name,
                dof.joint_type,
                self.robot.get_link(link.name),
                self.robot.get_link(child_link.name),
                T_world_axis,
                properties,
                dof.limits,
                dof.z_axis,
            )
            self.robot.joints.append(joint)

        return link
