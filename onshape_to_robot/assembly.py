from __future__ import annotations
import numpy as np
from .config import Config
from .message import error, info, bright, success, warning
from .onshape_api.client import Client
from .robot import Joint
from .expression import ExpressionParser

INSTANCE_IGNORE = -1


class Frame:
    """
    Represents a frame attached
    """

    def __init__(self, body_id: int, name: str, T_world_frame: np.ndarray):
        self.body_id: int = body_id
        self.name: str = name
        self.T_world_frame: np.ndarray = T_world_frame


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
        # Configuration values
        self.configuration_parameters: dict = {}
        # Dictionnary mapping items to their children in the tree
        self.tree_children: dict = {}
        # Root nodes
        self.root_nodes: list = []
        # Overriden link names
        self.link_names: dict[int, str] = {}
        # Relation indexed by target joints, values are [source joint, ratio]
        self.relations: dict = {}

        self.ensure_workspace_or_version()
        self.find_assembly()
        self.check_configuration()
        self.retrieve_assembly()
        self.find_instances()
        self.load_features()
        self.load_configuration()
        self.process_mates()
        self.build_trees()
        self.find_relations()
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
                    if not instance["suppressed"]:
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
                                self.find_instances(
                                    prefix + [instance["id"]], sub_assembly["instances"]
                                )

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

    def load_configuration(self):
        """
        Load configuration parameters
        """

        self.variable_values = None

        # Extracting configuration v ariables
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

    def merge_bodies(self, occurrence_A: str, occurrence_B: str):
        # Ensure occurrences are body
        if occurrence_A not in self.instance_body:
            self.make_body(occurrence_A)
        if occurrence_B not in self.instance_body:
            self.make_body(occurrence_B)

        # Merging bodies
        body1_id = self.instance_body[occurrence_A]
        body2_id = self.instance_body[occurrence_B]
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

    def translation(self, x: float, y: float, z: float) -> np.ndarray:
        return np.array(
            [
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1],
            ]
        )

    def process_mates(self):
        """
        Pre-assign all top-level instances to a separate body id
        """
        top_level_instances = self.assembly_data["rootAssembly"]["instances"]
        self.make_body(top_level_instances[0]["id"])

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
                        limits = self.get_limits(joint_type, data["name"])
                elif data["mateType"] == "SLIDER":
                    joint_type = Joint.PRISMATIC
                    if not self.config.ignore_limits:
                        limits = self.get_limits(joint_type, data["name"])
                elif data["mateType"] == "FASTENED":
                    joint_type = Joint.FIXED
                elif data["mateType"] == "BALL":
                    joint_type = Joint.BALL
                    if not self.config.ignore_limits:
                        limits = self.get_limits(joint_type, data["name"])
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
                    Frame(self.instance_body[parent], name, T_world_part)
                )

                if self.config.draw_frames:
                    self.merge_bodies(parent, child)
                else:
                    self.instance_body[child] = INSTANCE_IGNORE

        # Checking that all intances are assigned to a body
        for instance in self.assembly_data["rootAssembly"]["instances"]:
            if instance["id"] not in self.instance_body and not instance["suppressed"]:
                self.make_body(instance["id"])

        # Processing loop closing frames
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            is_hinge_closure = data["mateType"] == "REVOLUTE"

            if data["name"].startswith("closing_"):
                for k in 0, 1:
                    mated_entity = data["matedEntities"][k]
                    occurrence = mated_entity["matedOccurrence"][0]

                    T_world_part = self.get_occurrence_transform(
                        mated_entity["matedOccurrence"]
                    )
                    T_part_mate = self.get_mate_transform(mated_entity)
                    T_world_mate = T_world_part @ T_part_mate

                    self.frames.append(
                        Frame(
                            self.instance_body[occurrence],
                            f"{data['name']}_{k+1}",
                            T_world_mate,
                        )
                    )

                    if is_hinge_closure:
                        self.frames.append(
                            Frame(
                                self.instance_body[occurrence],
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
            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("link_"):
                link_name = "_".join(feature["featureData"]["name"].split("_")[1:])
                body_id = self.instance_body[feature["featureData"]["occurrence"][0]]
                self.link_names[body_id] = link_name

            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("frame_"):
                name = "_".join(feature["featureData"]["name"].split("_")[1:])
                occurrence = feature["featureData"]["occurrence"]
                T_world_occurrence = self.get_occurrence_transform(occurrence)
                body_id = self.instance_body[occurrence[0]]
                T_occurrence_mate = self.cs_to_transformation(
                    feature["featureData"]["mateConnectorCS"]
                )
                T_world_mate = T_world_occurrence @ T_occurrence_mate
                self.frames.append(Frame(body_id, name, T_world_mate))

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
            print(success(f"  - {self.body_instance(root_node)['name']}"))

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
        Iterate over all valid mating feature with two occurrences
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

                occurrence_A = data["matedEntities"][0]["matedOccurrence"][0]
                occurrence_B = data["matedEntities"][1]["matedOccurrence"][0]

                yield data, occurrence_A, occurrence_B

    def feature_mate_groups(self):
        """
        Find mate groups in the assembly
        """
        groups = []

        for feature in self.assembly_data["rootAssembly"]["features"]:
            group = []
            if feature["featureType"] == "mateGroup" and not feature["suppressed"]:
                data = feature["featureData"]

                for occurrence in data["occurrences"]:
                    group.append(occurrence["occurrence"][0])
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
                            dof1 = self.get_feature_by_id(
                                queries[0]["message"]["featureId"]
                            )["message"]["name"]
                            dof2 = self.get_feature_by_id(
                                queries[1]["message"]["featureId"]
                            )["message"]["name"]
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

    def read_parameter_value(self, parameter: str, name: str):
        """
        Try to read a parameter value from Onshape
        """

        # This is an expression
        if parameter["typeName"] == "BTMParameterNullableQuantity":
            return self.read_expression(parameter["message"]["expression"])
        if parameter["typeName"] == "BTMParameterConfigured":
            message = parameter["message"]
            parameterValue = self.configuration_parameters[
                message["configurationParameterId"]
            ]

            for value in message["values"]:
                if value["typeName"] == "BTMConfiguredValueByBoolean":
                    booleanValue = parameterValue == "true"
                    if value["message"]["booleanValue"] == booleanValue:
                        return self.read_expression(
                            value["message"]["value"]["message"]["expression"]
                        )
                elif value["typeName"] == "BTMConfiguredValueByEnum":
                    if value["message"]["enumValue"] == parameterValue:
                        return self.read_expression(
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

    def get_offset(self, name: str):
        """
        Retrieve the offset from current joint position in the assembly
        Currently, this only works with workspace in the API
        """
        if self.matevalues is None:
            return None

        for entry in self.matevalues["mateValues"]:
            if entry["mateName"] == name:
                if "rotationZ" in entry:
                    return entry["rotationZ"]
                elif "translationZ" in entry:
                    return entry["translationZ"]
                else:
                    print(warning(f"Unknown offset type for {name}"))
        return None

    def get_limits(self, joint_type: str, name: str):
        """
        Retrieve (low, high) limits for a given joint, if any
        """
        enabled = False
        minimum, maximum = 0, 0
        for feature in self.features["features"]:
            # Find coresponding joint
            if name == feature["message"]["name"]:
                # Find min and max values
                for parameter in feature["message"]["parameters"]:
                    if parameter["message"]["parameterId"] == "limitsEnabled":
                        enabled = parameter["message"]["value"]

                if enabled:
                    for parameter in feature["message"]["parameters"]:
                        if joint_type == Joint.REVOLUTE:
                            if parameter["message"]["parameterId"] == "limitAxialZMin":
                                minimum = self.read_parameter_value(parameter, name)
                            if parameter["message"]["parameterId"] == "limitAxialZMax":
                                maximum = self.read_parameter_value(parameter, name)
                        elif joint_type == Joint.PRISMATIC:
                            if parameter["message"]["parameterId"] == "limitZMin":
                                minimum = self.read_parameter_value(parameter, name)
                            if parameter["message"]["parameterId"] == "limitZMax":
                                maximum = self.read_parameter_value(parameter, name)
                        elif joint_type == Joint.BALL:
                            if (
                                parameter["message"]["parameterId"]
                                == "limitEulerConeAngleMax"
                            ):
                                minimum = 0
                                maximum = self.read_parameter_value(parameter, name)
                        else:
                            print(
                                warning(
                                    f"WARNING: Can't read limits for a joint of type {joint_type}"
                                )
                            )
                            print(parameter)
        if enabled:
            if joint_type != Joint.BALL:
                offset = self.get_offset(name)
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
        Get the (first) instance associated with a given body
        """
        for instance in self.assembly_data["rootAssembly"]["instances"]:
            if (
                instance["id"] in self.instance_body
                and self.instance_body[instance["id"]] == body_id
            ):
                return instance

        return None

    def body_occurrences(self, body_id: int):
        """
        Retrieve all occurrences associated to a given body id
        """
        for occurrence in self.assembly_data["rootAssembly"]["occurrences"]:
            key = occurrence["path"][0]
            if key in self.instance_body and self.instance_body[key] == body_id:
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
