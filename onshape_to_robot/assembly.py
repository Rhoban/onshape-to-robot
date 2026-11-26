from __future__ import annotations
from collections import Counter
import copy
from dataclasses import dataclass
import re
import numpy as np
from .config import Config
from .message import error, info, bright, success, warning
from .onshape_api.client import Client
from .robot import Joint
from .expression import ExpressionParser

INSTANCE_IGNORE = -1


@dataclass(frozen=True)
class _FeatureContext:
    """Feature and expression data for one assembly element."""

    features: dict
    expression_parser: ExpressionParser
    configuration_parameters: dict[str, str]
    matevalues: dict | None


class Frame:
    """
    Represents a frame attached
    """

    def __init__(
        self,
        body_id: int,
        name: str,
        T_world_frame: np.ndarray,
        source_path: list[str] | None = None,
    ):
        self.body_id: int = body_id
        self.name: str = name
        self.T_world_frame: np.ndarray = T_world_frame
        # For frame_ entries: path of the sub-assembly instance they come from
        # ([] at the root); None for frames that can't be renamed (closures)
        self.source_path: list[str] | None = source_path


class DOF:
    """
    Represents a DOF
    """

    def __init__(
        self,
        body1_id: int,
        body2_id: int,
        name: str,
        joint_type: str,
        T_world_mate: np.ndarray,
        limits: tuple | None,
        axis: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ):
        self.body1_id: int = body1_id
        self.body2_id: int = body2_id
        self.name: str = name
        self.joint_type: str = joint_type
        self.T_world_mate: np.ndarray = T_world_mate
        self.limits: tuple | None = limits
        # Path of the sub-assembly instance the DOF comes from (None at the root)
        self.source_path: list[str] | None = None
        self.axis: np.ndarray = axis

    def flip(self, flip_limits: bool = True):
        if flip_limits and self.limits is not None:
            self.limits = (-self.limits[1], -self.limits[0])

        # Flipping the joint around X axis
        flip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.T_world_mate[:3, :3] = self.T_world_mate[:3, :3] @ flip

    def other_body(self, body_id: int):
        if body_id == self.body1_id:
            return self.body2_id
        elif body_id == self.body2_id:
            return self.body1_id
        else:
            raise Exception(f"ERROR: body {body_id} is not part of this DOF")


class Assembly:
    """
    Main entry point to process an assembly
    """

    def __init__(self, config: Config):
        self.config: Config = config

        # Creating Onshape API client
        self.client = Client(logging=False, creds=self.config.config_file)
        self.expression_parser = ExpressionParser()
        self.expression_parser.variables_lazy_loading = self.load_variables

        self.document_id: str = config.document_id
        self.workspace_id: str | None = config.workspace_id
        self.version_id: str | None = config.version_id

        # All (raw) data from assembly
        self.assembly_data: dict = {}
        # Map a (top-level) instance id to a body id
        self.current_body_id: int = 0
        self.instance_body: dict[str, int] = {}
        # Frames object
        self.frames: list[Frame] = []
        # Loop closure constraints
        self.closures: list = []
        # Degrees of freedom
        self.dofs: list[DOF] = []
        # Features data
        self.features: dict = {}
        # Feature data for nested assembly elements
        self._feature_contexts: dict[
            tuple[str, str, str, str, str], _FeatureContext
        ] = {}
        # Configuration values
        self.configuration_parameters: dict = {}
        # Dictionnary mapping items to their children in the tree
        self.tree_children: dict = {}
        # Root nodes
        self.root_nodes: list = []
        # Overriden link names
        self.link_names: dict[int, str] = {}
        # Path of the sub-assembly instance each link_ name comes from ([] at the root)
        self.link_name_paths: dict[int, list[str]] = {}
        # Relation indexed by target joints, values are [source joint, ratio]
        self.relations: dict = {}
        # (path, subassembly data, name prefix) for each sub-assembly instance
        self.subassembly_instances: list[tuple[list[str], dict, str]] = []
        # Paths of the sub-assembly instances containing dof_ mates
        self.articulated_paths: set[tuple[str, ...]] = set()
        self._mated_paths_cache: set[tuple[str, ...]] | None = None

        self.ensure_workspace_or_version()
        self.find_assembly()
        self.check_configuration()
        self.retrieve_assembly()
        self.find_instances()
        self.load_features()
        self.ignore_locked_subassemblies()
        self.load_configuration()
        self.process_mates()
        self.build_trees()
        self.find_relations()
        self.finalize_dof_names()
        self.finalize_frame_and_link_names()
        print("")

    def ensure_workspace_or_version(self):
        """
        Ensure either a workspace id or a version id is set
        If none, try to retrieve the current workspace ID from API
        """
        if self.version_id:
            print(bright(f"* Using configuration version ID {self.version_id} ..."))
        elif self.workspace_id:
            print(bright(f"* Using configuration workspace ID {self.workspace_id} ..."))
        else:
            print(
                bright(
                    "* Not workspace ID specified, retrieving the current workspace ..."
                )
            )
            document = self.client.get_document(self.config.document_id)
            self.workspace_id = document["defaultWorkspace"]["id"]
            print(success(f"+ Using workspace id: {self.workspace_id}"))

    def find_assembly(self):
        """
        Find the wanted assembly from the document
        """
        if self.config.element_id:
            print(
                bright(f"* Using configuration element ID {self.config.element_id} ...")
            )
            self.element_id = self.config.element_id
            return

        print(
            bright(
                "\n* Retrieving elements in the document, searching for the assembly..."
            )
        )

        elements = self.client.list_elements(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            "v" if self.version_id else "w",
        )

        self.element_id = None
        assemblies: dict = {}
        for element in elements:
            if element["type"] == "Assembly":
                assemblies[element["name"]] = element["id"]

        if self.config.assembly_name:
            if self.config.assembly_name in assemblies:
                self.element_id = assemblies[self.config.assembly_name]
            else:
                raise Exception(
                    f"ERROR: Unable to find required assembly {self.config.assembly_name} in this document"
                )
        else:
            if len(assemblies) == 0:
                raise Exception("ERROR: No assembly found in this document\n")
            elif len(assemblies) == 1:
                self.element_id = list(assemblies.values())[0]
            else:
                raise Exception(
                    f"ERROR: Multiple assemblies found, please specify the assembly name\n"
                    + '       to export (use "assemblyName" in the configuration file)\n'
                    + f"       Available assemblies: {', '.join(assemblies.keys())}"
                )

        if self.element_id == None:
            raise Exception(f"ERROR: Unable to find assembly in this document")

    def check_configuration(self):
        """
        Retrieve configuration items for given assembly and parsing config configuration
        """

        if self.config.configuration != "default":
            # Retrieving available config parameters
            elements = self.client.elements_configuration(
                self.document_id,
                self.version_id if self.version_id else self.workspace_id,
                self.element_id,
                wmv=("v" if self.version_id else "w"),
            )

            parameters = {}
            for entry in elements["configurationParameters"]:
                type_name = entry["typeName"]
                message = entry["message"]

                if type_name.startswith("BTMConfigurationParameterEnum"):
                    # The very first label typed is kept as the internal name for the enum, under the "option"
                    # key. However, the user label that can be changed later is "optionName"
                    option_names = [
                        option["message"]["optionName"] for option in message["options"]
                    ]
                    options = [
                        option["message"]["option"] for option in message["options"]
                    ]
                    parameters[message["parameterName"]] = [
                        "enum",
                        message["parameterId"],
                        option_names,
                        options,
                    ]
                elif type_name.startswith("BTMConfigurationParameterBoolean"):
                    parameters[message["parameterName"]] = ["bool"]
                elif type_name.startswith("BTMConfigurationParameterQuantity"):
                    parameters[message["parameterName"]] = ["quantity"]

            # Parsing configuration
            parts = self.config.configuration.split(";")
            processed_configuration = []
            for part in parts:
                kv = part.split("=")
                if len(kv) == 2:
                    key, value = kv
                    if key not in parameters:
                        raise Exception(
                            f'ERROR: Unknown configuration parameter "{key}" in the configuration'
                        )
                    if parameters[key][0] == "enum":
                        if value not in parameters[key][2]:
                            raise Exception(
                                f'ERROR: Unknown value "{value}" for configuration parameter "{key}"'
                            )

                        value = parameters[key][3][parameters[key][2].index(value)]
                        key = parameters[key][1]
                    processed_configuration.append(f"{key}={value.replace(' ', '+')}")

            # Re-writing the configuration
            self.config.configuration = ";".join(processed_configuration)

    def retrieve_assembly(self):
        """
        Retrieve all assembly data
        """
        print(bright(f"* Retrieving assembly with id {self.element_id}"))

        self.assembly_data: dict = self.client.get_assembly(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.element_id,
            wmv=("v" if self.version_id else "w"),
            configuration=self.config.configuration,
        )

        self.microversion_id: str = self.assembly_data["rootAssembly"][
            "documentMicroversion"
        ]
        self.occurrences: dict = {}
        for occurrence in self.assembly_data["rootAssembly"]["occurrences"]:
            self.occurrences[tuple(occurrence["path"])] = occurrence

    def find_instances(self, prefix: list = [], instances=None):
        """
        Walking all the instances and associating them with their occurrences
        """
        if instances is None:
            instances = self.assembly_data["rootAssembly"]["instances"]

        for instance in instances:
            if "type" in instance:
                path = prefix + [instance["id"]]
                self.get_occurrence(path)["instance"] = instance

                if instance["type"] == "Assembly":
                    if not instance.get("suppressed", False):
                        d = instance["documentId"]
                        m = instance["documentMicroversion"]
                        e = instance["elementId"]
                        c = instance["configuration"]
                        for sub_assembly in self.assembly_data["subAssemblies"]:
                            if (
                                sub_assembly["documentId"] == d
                                and sub_assembly["documentMicroversion"] == m
                                and sub_assembly["elementId"] == e
                                and sub_assembly["configuration"] == c
                            ):
                                self.subassembly_instances.append(
                                    (path, sub_assembly, self.name_prefix(path))
                                )
                                if self.has_dof_mate(sub_assembly):
                                    self.articulated_paths.add(tuple(path))
                                self.find_instances(path, sub_assembly["instances"])

    def load_features(self):
        """
        Load features
        """

        self.features = self.client.get_features(
            self.document_id,
            self.microversion_id,
            self.element_id,
            wmv="m",
            configuration=self.config.configuration,
        )

        self.matevalues = self.client.matevalues(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.element_id,
            wmv="v" if self.version_id else "w",
            configuration=self.config.configuration,
        )

    def ignore_locked_subassemblies(self):
        """
        Sub-assembly instances locked in Onshape (padlock in the instance list)
        are exported as a single rigid link: their dof_ mates are ignored.

        The API doesn't expose the lock on the instance, but the mate values of
        a locked instance are missing from the root assembly's mate values
        (they are listed with their ownerOccurrencePath otherwise).
        """
        values = (self.matevalues or {}).get("mateValues") or []
        if not values:
            if self.articulated_paths:
                # Articulated sub-assemblies always have mate values unless
                # locked: an empty answer more likely means the values couldn't
                # be read, so don't freeze everything.
                print(
                    warning(
                        "WARNING: no mate values returned for the assembly, locked"
                        " sub-assemblies can't be detected (all are considered unlocked)"
                    )
                )
            return

        # Only mates that move have a value (a FASTENED dof_ mate has none), so
        # only sub-assemblies with such a dof_ mate can be checked
        subassemblies = {
            tuple(path): sub for path, sub, _ in self.subassembly_instances
        }

        def has_valued_dof(subassembly: dict) -> bool:
            return any(
                feature["featureType"] == "mate"
                and not feature["suppressed"]
                and feature["featureData"]["name"].startswith("dof_")
                and feature["featureData"].get("mateType") != "FASTENED"
                for feature in subassembly.get("features", [])
            )

        owners = [tuple(value.get("ownerOccurrencePath") or []) for value in values]
        for path in sorted(self.articulated_paths, key=len):
            if path not in self.articulated_paths:
                continue
            if not has_valued_dof(subassemblies[path]):
                continue
            if not any(owner[: len(path)] == path for owner in owners):
                locked = {
                    other
                    for other in self.articulated_paths
                    if other[: len(path)] == path
                }
                self.articulated_paths -= locked
                name = self.get_occurrence(list(path))["instance"]["name"]
                print(
                    info(
                        f"* Sub-assembly {name} is locked in Onshape, its dof_ mates are ignored"
                    )
                )

    def load_configuration(self):
        """
        Load configuration parameters
        """

        self.variable_values = None

        # Extracting configuration variables
        parts = self.assembly_data["rootAssembly"]["fullConfiguration"].split(";")
        for part in parts:
            key_value = part.split("=")
            if len(key_value) == 2:
                key, value = key_value
                value = value.replace("+", " ")
                self.configuration_parameters[key] = value
                try:
                    param_value = self.expression_parser.eval_expr(value)
                    self.expression_parser.variables[key] = param_value
                except ValueError:
                    pass

    def load_variables(self):
        """
        Load variables values (only if needed) in the expression parser
        """
        variables = self.client.get_variables(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.element_id,
            wmv="v" if self.version_id else "w",
            configuration=self.config.configuration,
        )
        for entry in variables:
            for variable in entry["variables"]:
                if variable["value"] is not None:
                    self.expression_parser.variables[variable["name"]] = (
                        self.expression_parser.eval_expr(variable["value"])
                    )

    def get_occurrence(self, path: list):
        """
        Retrieve occurrence from its path
        """
        return self.occurrences[tuple(path)]

    def get_occurrence_transform(self, path: list) -> np.ndarray:
        """
        Retrieve occurrence transform from its path
        """
        T_world_part = np.array(self.get_occurrence(path)["transform"]).reshape(4, 4)

        return T_world_part

    @staticmethod
    def _path_key(path: list[str]) -> str:
        """Convert an occurrence path list to a flat instance_body key."""
        return "/".join(path)

    @staticmethod
    def has_dof_mate(subassembly: dict) -> bool:
        """Whether a sub-assembly defines an active dof_ mate."""
        return any(
            feature["featureType"] == "mate"
            and not feature["suppressed"]
            and feature["featureData"]["name"].startswith("dof_")
            for feature in subassembly.get("features", [])
        )

    def _body_key_for_path(self, path: list[str]) -> str:
        """Return the instance_body key of the body containing an occurrence.

        An occurrence inside an articulated sub-assembly belongs to the child
        of the deepest such sub-assembly containing it. Any other occurrence
        belongs to its top-level instance.
        """
        for depth in range(len(path) - 1, 0, -1):
            if tuple(path[:depth]) in self.articulated_paths:
                return self._path_key(path[: depth + 1])
        return path[0]

    @staticmethod
    def _slugify_instance_name(name: str) -> str:
        """Turn an instance name like "Right arm <1>" into "right_arm"."""
        name = re.sub(r"\s*<\d+>\s*$", "", name)
        name = re.sub(r"[^a-z0-9]+", "_", name.lower())
        return name.strip("_")

    def name_prefix(self, path: list[str]) -> str:
        """Prefix for names coming from the sub-assembly instance at path.

        It joins the names of all instances along the path, so that two
        instances of the same sub-assembly never produce the same names.
        """
        return "_".join(
            self._slugify_instance_name(
                self.get_occurrence(path[:depth])["instance"]["name"]
            )
            for depth in range(1, len(path) + 1)
        )

    def _prefix_feature_data(
        self, data: dict, prefix: list[str], name_prefix: str
    ) -> dict:
        """Return a copy of a sub-assembly feature expressed from the root.

        Mated occurrence paths are prefixed with the sub-assembly path, and
        the feature name with name_prefix (keeping dof_, fix_, closing_ and
        frame_ first).
        """
        patched = copy.copy(data)
        patched["matedEntities"] = []
        for entity in data["matedEntities"]:
            patched_entity = copy.copy(entity)
            patched_entity["matedOccurrence"] = prefix + entity["matedOccurrence"]
            patched["matedEntities"].append(patched_entity)

        raw_name = data["name"]
        for known_prefix in ("dof_", "fix_", "closing_", "frame_"):
            if raw_name.startswith(known_prefix):
                patched["name"] = (
                    f"{known_prefix}{name_prefix}_{raw_name[len(known_prefix):]}"
                )
                return patched
        patched["name"] = f"{name_prefix}_{raw_name}"
        return patched

    def cs_to_transformation(self, cs: dict) -> np.ndarray:
        """
        Convert a coordinate system to a transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = np.stack(
            (
                np.array(cs["xAxis"]),
                np.array(cs["yAxis"]),
                np.array(cs["zAxis"]),
            )
        ).T
        T[:3, 3] = cs["origin"]

        return T

    def get_mate_transform(self, mated_entity: dict):
        return self.cs_to_transformation(mated_entity["matedCS"])

    def make_body(self, id: str):
        """
        Make the given instance id a body
        """
        self.instance_body[id] = self.current_body_id
        self.current_body_id += 1

    def _drop_empty_wrapper_instances(self) -> None:
        """Ignore bodies that contain neither a DOF endpoint nor a leaf part.

        Nested assemblies can leave wrapper bodies after their children have
        been assigned to articulated bodies. Empty wrappers must not become
        floating roots in the exported model.
        """
        dof_bodies = {dof.body1_id for dof in self.dofs} | {
            dof.body2_id for dof in self.dofs
        }
        part_bodies = {
            body_id
            for path, occurrence in self.occurrences.items()
            if occurrence.get("instance", {}).get("type") == "Part"
            if (body_id := self._occurrence_body(list(path))) is not None
        }
        empty_bodies = {
            body_id
            for body_id in self.instance_body.values()
            if body_id != INSTANCE_IGNORE
            and body_id not in dof_bodies
            and body_id not in part_bodies
        }
        for instance_key, body_id in self.instance_body.items():
            if body_id in empty_bodies:
                self.instance_body[instance_key] = INSTANCE_IGNORE

    def merge_body_ids(self, body1_id: int, body2_id: int):
        """Merge two bodies by id (lower id is kept)."""
        if body1_id == body2_id:
            return
        if body1_id > body2_id:
            body1_id, body2_id = body2_id, body1_id

        for occurrence in self.instance_body:
            if self.instance_body[occurrence] == body2_id:
                self.instance_body[occurrence] = body1_id

        for dof in self.dofs:
            if dof.body1_id == body2_id:
                dof.body1_id = body1_id
            if dof.body2_id == body2_id:
                dof.body2_id = body1_id

    def merge_bodies(self, occurrence_A: str, occurrence_B: str):
        # Ensure occurrences are body
        if occurrence_A not in self.instance_body:
            self.make_body(occurrence_A)
        if occurrence_B not in self.instance_body:
            self.make_body(occurrence_B)

        self.merge_body_ids(
            self.instance_body[occurrence_A], self.instance_body[occurrence_B]
        )

    def _merge_articulated_wrappers(self) -> None:
        """Attach each articulated assembly wrapper to its base child body."""
        assemblies = sorted(
            self.subassembly_instances,
            key=lambda entry: len(entry[0]),
            reverse=True,
        )
        for prefix, subassembly, _name_prefix in assemblies:
            if tuple(prefix) not in self.articulated_paths:
                continue

            children = [
                instance
                for instance in subassembly["instances"]
                if self.is_active_instance(instance)
            ]
            with_body = [
                instance
                for instance in children
                if self._path_key(prefix + [instance["id"]]) in self.instance_body
            ]
            if not with_body:
                continue

            # Children with no body (no mate at all) follow the wrapper, i.e.
            # the base body: prefer the part fixed in the sub-assembly
            base = with_body[0]
            unmated = [
                instance
                for instance in children
                if instance not in with_body
                and tuple(prefix + [instance["id"]]) not in self._mated_paths()
            ]
            if unmated:
                fixed_ids = self._fixed_instance_ids(subassembly)
                fixed = [
                    instance for instance in with_body if instance["id"] in fixed_ids
                ]
                if fixed:
                    base = fixed[0]
                    reason = f"the fixed part {base['name']}"
                else:
                    reason = (
                        f"{base['name']} (no fixed part with mates in the sub-assembly)"
                    )
                wrapper_name = self.get_occurrence(prefix)["instance"]["name"]
                print(
                    warning(
                        f"WARNING: {', '.join(i['name'] for i in unmated)} in"
                        f" {wrapper_name} has no mate, attached to {reason}"
                    )
                )

            wrapper_key = self._path_key(prefix)
            base_key = self._path_key(prefix + [base["id"]])
            if wrapper_key not in self.instance_body:
                self.instance_body[wrapper_key] = self.instance_body[base_key]
                continue

            self.merge_body_ids(
                self.instance_body[wrapper_key],
                self.instance_body[base_key],
            )

    def translation(self, x: float, y: float, z: float) -> np.ndarray:
        return np.array(
            [
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1],
            ]
        )

    def is_active_instance(self, instance: dict) -> bool:
        """
        An instance is active if it is resolved (unresolved instances, for example
        referencing a deleted element, have no type) and not suppressed
        """
        return "type" in instance and not instance.get("suppressed", False)

    def process_mates(self):
        """
        Pre-assign all top-level instances to a separate body id
        """
        top_level_instances = self.assembly_data["rootAssembly"]["instances"]
        first_instance = next(
            filter(self.is_active_instance, top_level_instances),
            top_level_instances[0],
        )
        self.make_body(first_instance["id"])

        # We first search for DOFs
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            if data["name"].startswith("dof_"):
                # Process the DOF name, removing dof prefix and inv suffix
                parts = data["name"].split("_")
                del parts[0]
                data["inverted"] = False
                if parts[-1] == "inv" or parts[-1] == "inverted":
                    data["inverted"] = True
                    del parts[-1]
                name = "_".join(parts)

                if name == "":
                    raise Exception(
                        f"ERROR: the following dof should have a name {data['name']}"
                    )

                # Finding joint type and limits
                limits = None
                if data["mateType"] == "REVOLUTE" or data["mateType"] == "CYLINDRICAL":
                    if "wheel" in parts or "continuous" in parts:
                        joint_type = Joint.CONTINUOUS
                    else:
                        joint_type = Joint.REVOLUTE

                    if not self.config.ignore_limits:
                        limits = self.get_limits(
                            joint_type,
                            data["name"],
                            data.get("_source_name"),
                            data.get("_subassembly"),
                        )
                elif data["mateType"] == "SLIDER":
                    joint_type = Joint.PRISMATIC
                    if not self.config.ignore_limits:
                        limits = self.get_limits(
                            joint_type,
                            data["name"],
                            data.get("_source_name"),
                            data.get("_subassembly"),
                        )
                elif data["mateType"] == "FASTENED":
                    joint_type = Joint.FIXED
                elif data["mateType"] == "BALL":
                    joint_type = Joint.BALL
                    if not self.config.ignore_limits:
                        limits = self.get_limits(
                            joint_type,
                            data["name"],
                            data.get("_source_name"),
                            data.get("_subassembly"),
                        )
                else:
                    raise Exception(
                        f"ERROR: {name} is declared as a DOF but the mate type is {data['mateType']}\n"
                        + "       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported"
                    )

                # We compute the axis in the world frame
                mated_entity = data["matedEntities"][0]
                T_world_part = self.get_occurrence_transform(
                    mated_entity["matedOccurrence"]
                )

                # jointToPart is the (rotation only) matrix from joint to the part
                # it is attached to
                T_part_mate = self.get_mate_transform(mated_entity)

                T_world_mate = T_world_part @ T_part_mate

                limits_str = ""
                if limits is not None:
                    limits_str = f"[{round(limits[0], 3)}: {round(limits[1], 3)}]"
                print(success(f"+ Found DOF: {name} ({joint_type}) {limits_str}"))

                # Ensure occurrences are body
                if occurrence_A not in self.instance_body:
                    self.make_body(occurrence_A)
                if occurrence_B not in self.instance_body:
                    self.make_body(occurrence_B)

                dof = DOF(
                    self.instance_body[occurrence_A],
                    self.instance_body[occurrence_B],
                    name,
                    joint_type,
                    T_world_mate,
                    limits,
                )

                if data["inverted"]:
                    dof.flip()

                # Sub-assembly instance path the DOF comes from (None at the root)
                dof.source_path = data.get("_path")
                self.dofs.append(dof)

        # Merging fixed links
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            if data["name"].startswith("fix_") or (
                data["mateType"] == "FASTENED"
                and not data["name"].startswith("dof_")
                and not data["name"].startswith("closing_")
                and not data["name"].startswith("frame_")
            ):
                self.merge_bodies(occurrence_A, occurrence_B)

        # Merging mate gorups
        for group in self.feature_mate_groups():
            for k in range(1, len(group)):
                occurrence_A = group[0]
                occurrence_B = group[k]

                self.merge_bodies(occurrence_A, occurrence_B)

        self._merge_articulated_wrappers()

        # Processing frame mates
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            if data["name"].startswith("frame_"):
                name = "_".join(data["name"].split("_")[1:])
                if (
                    occurrence_A not in self.instance_body
                    and occurrence_B in self.instance_body
                ):
                    parent, child = occurrence_B, occurrence_A
                    mated_entity = data["matedEntities"][0]
                elif (
                    occurrence_B not in self.instance_body
                    and occurrence_A in self.instance_body
                ):
                    parent, child = occurrence_A, occurrence_B
                    mated_entity = data["matedEntities"][1]
                else:
                    raise Exception(
                        f"Frame {name} should mate an orphan body to a body in the kinematics tree"
                    )

                T_world_part = self.get_occurrence_transform(
                    mated_entity["matedOccurrence"]
                )

                self.frames.append(
                    Frame(
                        self.instance_body[parent],
                        name,
                        T_world_part,
                        data.get("_path", []),
                    )
                )

                if self.config.draw_frames:
                    self.merge_bodies(parent, child)
                else:
                    self.instance_body[child] = INSTANCE_IGNORE

        # Checking that all intances are assigned to a body
        for instance in self.assembly_data["rootAssembly"]["instances"]:
            if "type" not in instance:
                # Unresolved instance (e.g. referencing a deleted element), the API
                # only returns a stub for it, with no type or suppressed status
                name = instance.get("name", instance["id"])
                status = instance.get("status", "unknown")
                print(
                    warning(
                        f"WARNING: ignoring unresolved instance {name} (status: {status})"
                    )
                )
            elif (
                self.is_active_instance(instance)
                and instance["id"] not in self.instance_body
            ):
                self.make_body(instance["id"])

        self._drop_empty_wrapper_instances()

        # Processing loop closing frames
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            is_hinge_closure = data["mateType"] == "REVOLUTE"

            if data["name"].startswith("closing_"):
                for k in 0, 1:
                    mated_entity = data["matedEntities"][k]
                    occ_path = mated_entity["matedOccurrence"]
                    body_id = self._occurrence_body(occ_path)

                    T_world_part = self.get_occurrence_transform(occ_path)
                    T_part_mate = self.get_mate_transform(mated_entity)
                    T_world_mate = T_world_part @ T_part_mate

                    self.frames.append(
                        Frame(
                            body_id,
                            f"{data['name']}_{k+1}",
                            T_world_mate,
                        )
                    )

                    if is_hinge_closure:
                        self.frames.append(
                            Frame(
                                body_id,
                                f"{data['name']}_{k+1}_z",
                                T_world_mate @ self.translation(0, 0, 0.1),
                            )
                        )

                closure_types = {
                    "FASTENED": "fixed",
                    "REVOLUTE": "revolute",
                    "BALL": "ball",
                    "SLIDER": "slider",
                }

                self.closures.append(
                    [
                        closure_types.get(data["mateType"], "unknown"),
                        f"{data['name']}_1",
                        f"{data['name']}_2",
                    ]
                )
                if is_hinge_closure:
                    self.closures.append(
                        [
                            closure_types.get(data["mateType"], "unknown"),
                            f"{data['name']}_1_z",
                            f"{data['name']}_2_z",
                        ]
                    )

        # Search for mate connector named "link_..." to override link names
        for feature in self.assembly_data["rootAssembly"]["features"]:
            # Suppressed mate connectors reference occurrences that may no longer
            # exist in the assembly, so skip them like mates and mate groups do.
            if feature.get("suppressed"):
                continue

            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("link_"):
                link_name = "_".join(feature["featureData"]["name"].split("_")[1:])
                body_id = self._occurrence_body(feature["featureData"]["occurrence"])
                self.link_names[body_id] = link_name
                self.link_name_paths[body_id] = []

            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("frame_"):
                name = "_".join(feature["featureData"]["name"].split("_")[1:])
                occurrence = feature["featureData"]["occurrence"]
                T_world_occurrence = self.get_occurrence_transform(occurrence)
                body_id = self._occurrence_body(occurrence)
                T_occurrence_mate = self.cs_to_transformation(
                    feature["featureData"]["mateConnectorCS"]
                )
                T_world_mate = T_world_occurrence @ T_occurrence_mate
                self.frames.append(Frame(body_id, name, T_world_mate, []))

        # Same for mate connectors inside sub-assemblies
        for path, _, name_prefix, data in self.subassembly_features("mateConnector"):
            occurrence = path + data["occurrence"]
            body_id = self._occurrence_body(occurrence)
            name = f"{name_prefix}_" + "_".join(data["name"].split("_")[1:])

            if data["name"].startswith("link_"):
                self.link_names[body_id] = name
                self.link_name_paths[body_id] = path

            if data["name"].startswith("frame_"):
                T_world_occurrence = self.get_occurrence_transform(occurrence)
                T_occurrence_mate = self.cs_to_transformation(data["mateConnectorCS"])
                T_world_mate = T_world_occurrence @ T_occurrence_mate
                self.frames.append(Frame(body_id, name, T_world_mate, path))

        print(success(f"* Found total {len(self.dofs)} degrees of freedom"))

    def build_trees(self):
        """
        Perform checks on the produced tree
        """
        self.body_in_tree = []
        for body_id in self.instance_body.values():
            if body_id != INSTANCE_IGNORE and body_id not in self.body_in_tree:
                self.build_tree(body_id)

        print(success(f"* Found {len(self.root_nodes)} root nodes:"))
        for root_node in self.root_nodes:
            instance = self.body_instance(root_node)
            label = instance["name"] if instance else f"body_{root_node}"
            print(success(f"  - {label}"))

    def build_tree(self, root_node: int):
        """
        Building a tree starting a root_node
        """
        # Append the root node
        self.root_nodes.append(root_node)

        # Checking that the graph is actually a tree (no loop)
        exploring = [root_node]
        dofs = self.dofs.copy()
        while len(exploring) > 0:
            current = exploring.pop()
            self.body_in_tree.append(current)

            children = []
            dofs_to_remove = []
            for dof in dofs:
                if dof.body1_id == current:
                    dof.flip(flip_limits=False)
                    children.append(dof.body2_id)
                    dofs_to_remove.append(dof)
                elif dof.body2_id == current:
                    children.append(dof.body1_id)
                    dofs_to_remove.append(dof)
            for dof in dofs_to_remove:
                dofs.remove(dof)

            self.tree_children[current] = children
            for child in children:
                if child in self.body_in_tree:
                    raise Exception(
                        "The DOF graph is not a tree, check for loops in your DOFs"
                    )
                elif child not in exploring:
                    exploring.append(child)

    def feature_mating_two_occurrences(self):
        """
        Iterate over all valid mating features with two occurrences.

        Yields root-level mates first, then internal mates from each
        subassembly instance (with occurrence paths prefixed to root scope).
        """
        for feature in self.assembly_data["rootAssembly"]["features"]:
            if feature["featureType"] == "mate" and not feature["suppressed"]:
                data = feature["featureData"]

                if (
                    "matedEntities" not in data
                    or len(data["matedEntities"]) != 2
                    or len(data["matedEntities"][0]["matedOccurrence"]) == 0
                    or len(data["matedEntities"][1]["matedOccurrence"]) == 0
                ):
                    continue

                path_a = data["matedEntities"][0]["matedOccurrence"]
                path_b = data["matedEntities"][1]["matedOccurrence"]
                occurrence_A = self._body_key_for_path(path_a)
                occurrence_B = self._body_key_for_path(path_b)

                yield data, occurrence_A, occurrence_B

        for path, subassembly, name_prefix, data in self.subassembly_features("mate"):
            if (
                "matedEntities" not in data
                or len(data["matedEntities"]) != 2
                or len(data["matedEntities"][0]["matedOccurrence"]) == 0
                or len(data["matedEntities"][1]["matedOccurrence"]) == 0
            ):
                continue

            prefixed_data = self._prefix_feature_data(data, path, name_prefix)
            prefixed_data["_source_name"] = data["name"]
            prefixed_data["_subassembly"] = subassembly
            prefixed_data["_path"] = path
            path_a = prefixed_data["matedEntities"][0]["matedOccurrence"]
            path_b = prefixed_data["matedEntities"][1]["matedOccurrence"]
            occurrence_A = self._body_key_for_path(path_a)
            occurrence_B = self._body_key_for_path(path_b)
            yield prefixed_data, occurrence_A, occurrence_B

    def contains_articulated(self, path: list[str]) -> bool:
        """Whether the sub-assembly at path is articulated or contains one.

        Features of other sub-assemblies are ignored: such a sub-assembly is
        a single rigid body, as if it had no internal features.
        """
        return any(
            articulated_path[: len(path)] == tuple(path)
            for articulated_path in self.articulated_paths
        )

    def subassembly_features(self, feature_type: str):
        """
        Yield (path, subassembly, name prefix, feature data) for active features
        of the given type in sub-assemblies that are articulated or contain one.
        """
        for path, subassembly, name_prefix in self.subassembly_instances:
            if not self.contains_articulated(path):
                continue
            for feature in subassembly.get("features", []):
                if feature["featureType"] == feature_type and not feature["suppressed"]:
                    yield path, subassembly, name_prefix, feature["featureData"]

    def feature_mate_groups(self):
        """
        Find mate groups in the assembly (root-level and inside subassemblies).
        """
        groups = []

        for feature in self.assembly_data["rootAssembly"]["features"]:
            group = []
            if feature["featureType"] == "mateGroup" and not feature["suppressed"]:
                data = feature["featureData"]

                for occurrence in data["occurrences"]:
                    group.append(self._body_key_for_path(occurrence["occurrence"]))
            if group:
                groups.append(group)

        for path, _, _, data in self.subassembly_features("mateGroup"):
            group = [
                self._body_key_for_path(path + occurrence["occurrence"])
                for occurrence in data["occurrences"]
            ]
            if group:
                groups.append(group)

        return groups

    def get_feature_by_id(self, feature_id: str):
        """
        Find a specific feature by its ID
        """
        for feature in self.features["features"]:
            if feature["message"]["featureId"] == feature_id:
                return feature

        return None

    def find_relations(self):
        """
        Finding relations features in the assembly
        """
        for feature in self.features["features"]:
            if feature["typeName"] == "BTMMateRelation":
                relation_name = feature["message"]["name"]

                mated_dofs = None
                ratio = None
                reverse = None
                for parameter in feature["message"]["parameters"]:
                    if parameter["message"]["parameterId"] == "matesQuery":
                        queries = parameter["message"]["queries"]
                        if len(queries) == 2:
                            dof1_feature = self.get_feature_by_id(
                                queries[0]["message"]["featureId"]
                            )
                            dof2_feature = self.get_feature_by_id(
                                queries[1]["message"]["featureId"]
                            )
                            if dof1_feature is not None and dof2_feature is not None:
                                dof1 = dof1_feature["message"]["name"]
                                dof2 = dof2_feature["message"]["name"]
                                if dof1.startswith("dof_") and dof2.startswith("dof_"):
                                    mated_dofs = [dof1[4:], dof2[4:]]
                    elif parameter["message"]["parameterId"] == "relationRatio":
                        ratio = self.read_expression(parameter["message"]["expression"])
                    elif parameter["message"]["parameterId"] == "reverseDirection":
                        reverse = parameter["message"]["value"]

                if mated_dofs is not None and ratio is not None and reverse is not None:
                    if not reverse:
                        ratio = -ratio

                    print(
                        success(
                            f"+ Found relation {relation_name} mating {mated_dofs} with ratio {ratio}"
                        )
                    )
                    if mated_dofs[1] in self.relations:
                        print(
                            warning(
                                f"Multiple relations found with {mated_dofs[1]} as target"
                            )
                        )

                    self.relations[mated_dofs[1]] = [mated_dofs[0], ratio]

    def _duplicate_suffixes(self, group: list[DOF]) -> list[str]:
        """
        Suffixes telling apart DOFs that share a name: the "<n>" of the
        instances they come from, at the levels where they differ (Leg <4>
        gives "4"), or 1, 2, 3, ... in the order found if that doesn't work.
        """
        return self._instance_number_suffixes([dof.source_path for dof in group])

    def _instance_number_suffixes(self, paths: list[list[str] | None]) -> list[str]:
        """
        Suffixes telling apart entries coming from sub-assembly instances whose
        names only differ by their "<n>": the "<n>" at the levels where they
        differ, or 1, 2, 3, ... in the given order if that doesn't work.
        """

        def instance_numbers(path: list[str] | None) -> list[str]:
            path = path or []
            numbers = []
            for depth in range(1, len(path) + 1):
                name = self.get_occurrence(path[:depth])["instance"]["name"]
                match = re.search(r"<(\d+)>\s*$", name)
                numbers.append(match.group(1) if match else "")
            return numbers

        group = paths
        numbers = [instance_numbers(path) for path in paths]
        if all(numbers) and len({len(n) for n in numbers}) == 1:
            differing = [
                depth
                for depth in range(len(numbers[0]))
                if len({n[depth] for n in numbers}) > 1
            ]
            suffixes = ["_".join(n[depth] for depth in differing) for n in numbers]
            if (
                differing
                and all(all(n[depth] for depth in differing) for n in numbers)
                and len(set(suffixes)) == len(suffixes)
            ):
                return suffixes
        return [str(index) for index in range(1, len(group) + 1)]

    def finalize_frame_and_link_names(self):
        """
        Number the names coming from frame_ and link_ entries that are
        duplicated (both end up as link names in the export), like DOFs: with
        the "<n>" of the sub-assembly instance they come from (Arm <2> gives
        arm_hand_2), or 1, 2, 3, ... in the order found.
        """
        entries: dict[str, list[tuple]] = {}
        for frame in self.frames:
            if frame.source_path is not None:
                entries.setdefault(frame.name, []).append(
                    ("frame", frame, frame.source_path)
                )
        for body_id, name in self.link_names.items():
            path = self.link_name_paths.get(body_id, [])
            entries.setdefault(name, []).append(("link", body_id, path))

        for name, group in entries.items():
            if len(group) < 2:
                continue
            suffixes = self._instance_number_suffixes([path for _, _, path in group])
            for (kind, target, _), suffix in zip(group, suffixes):
                new_name = f"{name}_{suffix}"
                print(
                    info(
                        f"* Several frames or links are named {name}, renaming one to {new_name}"
                    )
                )
                if kind == "frame":
                    target.name = new_name
                else:
                    self.link_names[target] = new_name

    def finalize_dof_names(self):
        """
        Give every DOF its final, unique name.

        - DOFs sharing the same name (e.g. the same articulated sub-assembly
          instantiated several times) are all numbered: as soon as there is a
          "_1", there is a "_2". The number is the "<n>" of the Onshape instance
          they come from (Arm <2> gives arm_elbow_2). If that doesn't tell them
          apart (e.g. at the root), they are numbered in the order found.
        - The "rename_dofs" config entry is then applied: each key is a regular
          expression matched against the whole DOF name, and the first matching
          key gives the new name (groups like \\1 can be used). A plain name
          simply renames that DOF.
        """
        final_names = {}
        groups: dict[str, list[DOF]] = {}
        for dof in self.dofs:
            groups.setdefault(dof.name, []).append(dof)

        for name, group in groups.items():
            if len(group) < 2:
                continue
            for dof, suffix in zip(group, self._duplicate_suffixes(group)):
                dof.name = f"{name}_{suffix}"
                print(
                    info(f"* Several DOFs are named {name}, renaming one to {dof.name}")
                )

        if not isinstance(self.config.rename_dofs, dict):
            raise Exception(
                'ERROR: "rename_dofs" should be a dict {"name or pattern": "new name"}'
            )
        rules = []
        for pattern, replacement in self.config.rename_dofs.items():
            try:
                rules.append([re.compile(pattern), replacement, False])
            except re.error as e:
                raise Exception(f"ERROR: invalid rename_dofs pattern {pattern!r}: {e}")

        for dof in self.dofs:
            for rule in rules:
                match = rule[0].fullmatch(dof.name)
                if match:
                    new_name = match.expand(rule[1])
                    print(success(f"+ Renaming DOF {dof.name} to {new_name}"))
                    final_names[dof.name] = new_name
                    dof.name = new_name
                    rule[2] = True
                    break

        for pattern, _, used in rules:
            if not used:
                print(
                    warning(
                        f"WARNING: rename_dofs entry {pattern.pattern!r} matched no DOF"
                    )
                )

        duplicates = [
            n for n, c in Counter(dof.name for dof in self.dofs).items() if c > 1
        ]
        if duplicates:
            raise Exception(
                f"ERROR: several DOFs are named {', '.join(duplicates)}\n"
                + '       Use the "rename_dofs" entry of config.json to give them distinct names'
            )

        # Relations refer to DOFs by name
        if final_names:
            self.relations = {
                final_names.get(target, target): [
                    final_names.get(source, source),
                    ratio,
                ]
                for target, (source, ratio) in self.relations.items()
            }

    def read_parameter_value(
        self,
        parameter: dict,
        name: str,
        expression_parser: ExpressionParser | None = None,
        configuration_parameters: dict[str, str] | None = None,
    ) -> float | None:
        """
        Try to read a parameter value from Onshape
        """
        parser = expression_parser or self.expression_parser
        configured_values = (
            configuration_parameters
            if configuration_parameters is not None
            else self.configuration_parameters
        )

        # This is an expression
        if parameter["typeName"] == "BTMParameterNullableQuantity":
            return parser.eval_expr(parameter["message"]["expression"])
        if parameter["typeName"] == "BTMParameterConfigured":
            message = parameter["message"]
            parameterValue = configured_values[message["configurationParameterId"]]

            for value in message["values"]:
                if value["typeName"] == "BTMConfiguredValueByBoolean":
                    booleanValue = parameterValue == "true"
                    if value["message"]["booleanValue"] == booleanValue:
                        return parser.eval_expr(
                            value["message"]["value"]["message"]["expression"]
                        )
                elif value["typeName"] == "BTMConfiguredValueByEnum":
                    if value["message"]["enumValue"] == parameterValue:
                        return parser.eval_expr(
                            value["message"]["value"]["message"]["expression"]
                        )
                else:
                    raise Exception(
                        "Can't read value of parameter {name} configured with {value['typeName']}"
                    )

            print(error(f"Coud not find the value for {name}"))
        else:
            raise Exception(f"Unknown feature type for {name}: {parameter['typeName']}")

    def read_expression(self, expression: str):
        """
        Reading an expression from Onshape
        """
        return self.expression_parser.eval_expr(expression)

    def get_offset(self, name: str, matevalues: dict | None):
        """
        Retrieve the offset from current joint position in the assembly
        Currently, this only works with workspace in the API
        """
        if matevalues is None:
            return None

        for entry in matevalues["mateValues"]:
            if entry["mateName"] == name:
                if "rotationZ" in entry:
                    return entry["rotationZ"]
                elif "translationZ" in entry:
                    return entry["translationZ"]
                else:
                    print(warning(f"Unknown offset type for {name}"))
        return None

    @staticmethod
    def _configuration_values(configuration: str) -> dict[str, str]:
        """Parse an Onshape full-configuration string."""
        values = {}
        for item in configuration.split(";"):
            parts = item.split("=", maxsplit=1)
            if len(parts) != 2:
                continue
            values[parts[0]] = parts[1].replace("+", " ")
        return values

    def _subassembly_context(self, subassembly: dict) -> tuple[str, str]:
        """(wmv, id) to use for API calls on a sub-assembly element."""
        version_id = subassembly.get("documentVersion")
        if version_id is not None:
            # Linked from another document: pinned to a version of that document
            return "v", version_id
        if subassembly["documentId"] == self.document_id:
            # Same document: it shares the root's workspace or version. The
            # variables and matevalues endpoints don't accept a microversion.
            if self.version_id:
                return "v", self.version_id
            return "w", self.workspace_id
        return "m", subassembly["documentMicroversion"]

    def _fixed_instance_ids(self, subassembly: dict) -> set[str]:
        """
        Ids of the instances fixed inside a sub-assembly. The parent assembly
        definition doesn't tell it, so this needs the sub-assembly's own one.
        """
        wmv, element_version_id = self._subassembly_context(subassembly)
        definition = self.client.get_assembly(
            subassembly["documentId"],
            element_version_id,
            subassembly["elementId"],
            wmv=wmv,
            configuration=subassembly["configuration"],
        )
        return {
            occurrence["path"][0]
            for occurrence in definition["rootAssembly"]["occurrences"]
            if occurrence.get("fixed") and len(occurrence["path"]) == 1
        }

    def _mated_paths(self) -> set[tuple[str, ...]]:
        """
        Occurrence paths (and their prefixes) referenced by an active mate or
        mate group.
        """
        if self._mated_paths_cache is None:
            mated = set()
            sources = [([], self.assembly_data["rootAssembly"])] + [
                (path, subassembly)
                for path, subassembly, _ in self.subassembly_instances
            ]
            for prefix, assembly in sources:
                for feature in assembly.get("features", []):
                    if feature.get("suppressed"):
                        continue
                    data = feature["featureData"]
                    paths = [
                        e["matedOccurrence"] for e in data.get("matedEntities", [])
                    ]
                    paths += [o["occurrence"] for o in data.get("occurrences", [])]
                    for path in paths:
                        path = prefix + path
                        for depth in range(1, len(path) + 1):
                            mated.add(tuple(path[:depth]))
            self._mated_paths_cache = mated
        return self._mated_paths_cache

    def _load_feature_context(self, subassembly: dict) -> _FeatureContext:
        """Load and cache feature data for a nested assembly."""
        wmv, element_version_id = self._subassembly_context(subassembly)
        key = (
            subassembly["documentId"],
            element_version_id,
            subassembly["elementId"],
            subassembly["configuration"],
            wmv,
        )
        cached = self._feature_contexts.get(key)
        if cached is not None:
            return cached

        parser = ExpressionParser()
        pending_variables = [
            variable
            for entry in self.client.get_variables(
                key[0],
                key[1],
                key[2],
                wmv=key[4],
                configuration=key[3],
            )
            for variable in entry["variables"]
            if variable["value"] is not None
        ]
        while pending_variables:
            unresolved = []
            for variable in pending_variables:
                try:
                    parser.variables[variable["name"]] = parser.eval_expr(
                        variable["value"]
                    )
                except ValueError:
                    unresolved.append(variable)
            if len(unresolved) == len(pending_variables):
                break
            pending_variables = unresolved

        context = _FeatureContext(
            features=self.client.get_features(
                key[0],
                key[1],
                key[2],
                wmv=key[4],
                configuration=key[3],
            ),
            expression_parser=parser,
            configuration_parameters=self._configuration_values(
                subassembly.get("fullConfiguration", key[3])
            ),
            matevalues=self.client.matevalues(
                key[0],
                key[1],
                key[2],
                wmv=key[4],
                configuration=key[3],
            ),
        )
        self._feature_contexts[key] = context
        return context

    def get_limits(
        self,
        joint_type: str,
        name: str,
        source_name: str | None = None,
        subassembly: dict | None = None,
    ) -> tuple[float, float] | None:
        """
        Retrieve (low, high) limits for a given joint, if any
        """
        feature_name = source_name or name
        features = self.features
        parser = self.expression_parser
        configured_values = self.configuration_parameters
        matevalues = self.matevalues
        if subassembly is not None:
            context = self._load_feature_context(subassembly)
            features = context.features
            parser = context.expression_parser
            configured_values = context.configuration_parameters
            matevalues = context.matevalues

        enabled = False
        minimum, maximum = 0, 0
        for feature in features["features"]:
            # Find coresponding joint
            if feature_name == feature["message"]["name"]:
                # Find min and max values
                for parameter in feature["message"]["parameters"]:
                    if parameter["message"]["parameterId"] == "limitsEnabled":
                        enabled = parameter["message"]["value"]

                if enabled:
                    for parameter in feature["message"]["parameters"]:
                        if joint_type == Joint.REVOLUTE:
                            if parameter["message"]["parameterId"] == "limitAxialZMin":
                                minimum = self.read_parameter_value(
                                    parameter,
                                    feature_name,
                                    parser,
                                    configured_values,
                                )
                            if parameter["message"]["parameterId"] == "limitAxialZMax":
                                maximum = self.read_parameter_value(
                                    parameter,
                                    feature_name,
                                    parser,
                                    configured_values,
                                )
                        elif joint_type == Joint.PRISMATIC:
                            if parameter["message"]["parameterId"] == "limitZMin":
                                minimum = self.read_parameter_value(
                                    parameter,
                                    feature_name,
                                    parser,
                                    configured_values,
                                )
                            if parameter["message"]["parameterId"] == "limitZMax":
                                maximum = self.read_parameter_value(
                                    parameter,
                                    feature_name,
                                    parser,
                                    configured_values,
                                )
                        elif joint_type == Joint.BALL:
                            if (
                                parameter["message"]["parameterId"]
                                == "limitEulerConeAngleMax"
                            ):
                                minimum = 0
                                maximum = self.read_parameter_value(
                                    parameter,
                                    feature_name,
                                    parser,
                                    configured_values,
                                )
                        else:
                            print(
                                warning(
                                    f"WARNING: Can't read limits for a joint of type {joint_type}"
                                )
                            )
                            print(parameter)
        if enabled:
            if joint_type != Joint.BALL:
                offset = self.get_offset(feature_name, matevalues)
                if offset is not None:
                    minimum -= offset
                    maximum -= offset
            return (minimum, maximum)
        else:
            if joint_type != Joint.CONTINUOUS:
                print(
                    warning(f"WARNING: joint {name} of type {joint_type} has no limits")
                )
            return None

    def body_instance(self, body_id: int):
        """
        Get the (first) instance associated with a given body, used to name it.

        For a body inside an articulated sub-assembly, an instance of the
        sub-assembly is used (the first one of the shallowest level, in the
        sub-assembly's order), i.e. its part rather than the articulated
        sub-assembly instance mapped to the same body: the base body of
        "Leg 2 <1>" is named after its "DoubleU" part, as on master, not
        "leg_2". Otherwise, root-level instances are searched first. Path keys
        are matched without splitting on ``/`` because Onshape IDs may contain
        slashes.
        """
        nested = [
            path
            for path in self.occurrences
            if len(path) >= 2
            and path not in self.articulated_paths
            and self.instance_body.get(self._path_key(list(path))) == body_id
            and "instance" in self.occurrences[path]
        ]
        if nested:
            # The first instance of the shallowest level, in the order of the
            # sub-assembly's instance list (as root instances are searched)
            children = {
                tuple(path): [i["id"] for i in subassembly["instances"]]
                for path, subassembly, _ in self.subassembly_instances
            }

            def order(path):
                siblings = children.get(path[:-1], [])
                index = siblings.index(path[-1]) if path[-1] in siblings else 0
                return (len(path), index)

            return self.occurrences[min(nested, key=order)]["instance"]

        for instance in self.assembly_data["rootAssembly"]["instances"]:
            if (
                instance["id"] in self.instance_body
                and self.instance_body[instance["id"]] == body_id
            ):
                return instance

        for key, mapped_body_id in self.instance_body.items():
            if mapped_body_id != body_id:
                continue
            # Do not split keys on "/": Onshape instance IDs can contain slashes.
            for occ_path, occurrence in self.occurrences.items():
                if self._path_key(list(occ_path)) != key:
                    continue
                instance = occurrence.get("instance")
                if instance is not None:
                    return instance

        return None

    def _occurrence_body(self, path: list[str]) -> int | None:
        """Resolve the body for an occurrence using longest-prefix matching.

        Tries progressively shorter prefixes of *path* as keys in
        instance_body, returning the first match. This allows nested
        instances split by internal DOFs to be correctly resolved while
        unsplit internal instances fall back to their parent subassembly body.
        """
        for depth in range(len(path), 0, -1):
            key = self._path_key(path[:depth])
            if key in self.instance_body:
                return self.instance_body[key]
        return None

    def body_occurrences(self, body_id: int):
        """
        Retrieve all occurrences associated to a given body id.

        Uses longest-prefix matching so that internal instances split by
        subassembly DOFs resolve to their own body rather than the parent.
        Occurrences below a suppressed sub-assembly have no instance and are
        skipped.
        """
        for occurrence in self.assembly_data["rootAssembly"]["occurrences"]:
            if "instance" not in occurrence:
                continue
            resolved = self._occurrence_body(occurrence["path"])
            if resolved == body_id:
                yield occurrence

    def get_dof(self, body1_id: int, body2_id: int):
        """
        Get a DOF for given bodies
        """
        for dof in self.dofs:
            if (dof.body1_id == body1_id and dof.body2_id == body2_id) or (
                dof.body1_id == body2_id and dof.body2_id == body1_id
            ):
                return dof

        raise Exception(f"ERROR: no DOF found between {body1_id} and {body2_id}")
