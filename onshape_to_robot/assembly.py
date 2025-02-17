import math
from .config import Config
from .message import error, info, bright, success, warning
from .onshape_api.client import Client
from .features import init as features_init, getLimits


class Assembly:
    """
    Main entry point to process an assembly
    """

    def __init__(self, config: Config):
        self.config: Config = config

        # Creating OnShape API client
        self.client = Client(logging=False, creds=self.config.config_file)
        # TODO: Investigate on the use/clean-ness of this
        self.client.useCollisionsConfigurations = (
            self.config.use_collisions_configurations
        )

        self.document_id: str = config.document_id
        self.workspace_id: str | None = config.workspace_id
        self.version_id: str | None = config.version_id

        # All (raw) data from assembly
        self.assembly_data: dict = {}
        # Map a (top-level) instance id to a body id
        self.instance_body: dict[str, int] = {}
        # Features data
        self.features: dict = {}
        # Configuration values
        self.configuration_parameters: dict = {}

        self.ensure_workspace_or_version()
        self.find_assembly()
        self.retrieve_assembly()
        self.load_features()
        self.load_configuration()
        self.preassign_instances()
        self.merge_instances()

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

        self.assembly_id = None
        self.assembly_name = ""
        for element in elements:
            if element["type"] == "Assembly" and (
                (not self.config.assembly_name)
                or element["name"] == self.config.assembly_name
            ):
                print(
                    success(
                        f"+ Found assembly, id: {element['id']}, name: {element['name']}"
                    )
                )
                self.assembly_name = element["name"]
                self.assembly_id = element["id"]

        if self.assembly_id == None:
            raise Exception(f"ERROR: Unable to find assembly in this document")

    def retrieve_assembly(self):
        """
        Retrieve all assembly data
        """
        print(
            bright(
                f'* Retrieving assembly "{self.assembly_name}" with id {self.assembly_id}'
            )
        )

        self.assembly_data: dict = self.client.get_assembly(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.assembly_id,
            "v" if self.version_id else "w",
            configuration=self.config.configuration,
        )

    def load_features(self):
        """
        Load features
        """

        self.features = self.client.get_features(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.assembly_id,
            "v" if self.version_id else "w",
            configuration=self.config.configuration,
        )

        if not self.version_id:
            self.matevalues = self.client.matevalues(
                self.document_id,
                self.workspace_id,
                self.assembly_id,
                configuration=self.config.configuration,
            )
        else:
            self.matevalues = None

    def load_configuration(self):
        """
        Load configuration parameters
        """

        parts = self.assembly_data["rootAssembly"]["fullConfiguration"].split(";")
        for part in parts:
            key_value = part.split("=")
            if len(key_value) == 2:
                key, value = key_value
                self.configuration_parameters[key] = value.replace("+", " ")

    def preassign_instances(self):
        """
        Pre-assign all top-level instances to a separate body id
        """
        top_level_instances = self.assembly_data["rootAssembly"]["instances"]
        body_id = 0
        for instance in top_level_instances:
            self.instance_body[instance["id"]] = body_id
            body_id += 1

    def merge_instances(self):
        pass

    def read_parameter_value(self, parameter: str, name: str):
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
        Reading an expression from OnShape
        """
        # Expression can itself be a variable from configuration
        # XXX: This doesn't handle all expression, only values and variables
        if expression[0] == "#":
            expression = self.configuration_parameters[expression[1:]]
        if expression[0:2] == "-#":
            expression = "-" + self.configuration_parameters[expression[2:]]

        parts = expression.split(" ")

        # Checking the unit, returning only radians and meters
        if parts[1] == "deg":
            return math.radians(float(parts[0]))
        elif parts[1] in ["radian", "rad"]:
            # looking for PI
            if isinstance(parts[0], str):
                if parts[0] == "(PI)":
                    value = math.pi
                else:
                    raise ValueError(f"{parts[0]} variable isn't supported")
            else:
                value = parts[0]
            return float(value)
        elif parts[1] == "mm":
            return float(parts[0]) / 1000.0
        elif parts[1] == "cm":
            return float(parts[0]) / 100.0
        elif parts[1] == "m":
            return float(parts[0])
        elif parts[1] == "in":
            return float(parts[0]) * 0.0254
        else:
            raise Exception(f"Unknown unit: {parts[1]}")

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

                    if joint_type == "revolute":
                        if parameter["message"]["parameterId"] == "limitAxialZMin":
                            minimum = self.read_parameter_value(parameter, name)
                        if parameter["message"]["parameterId"] == "limitAxialZMax":
                            maximum = self.read_parameter_value(parameter, name)
                    elif joint_type == "prismatic":
                        if parameter["message"]["parameterId"] == "limitZMin":
                            minimum = self.read_parameter_value(parameter, name)
                        if parameter["message"]["parameterId"] == "limitZMax":
                            maximum = self.read_parameter_value(parameter, name)
        if enabled:
            offset = self.get_offset(name)
            if offset is not None:
                minimum -= offset
                maximum -= offset
            return (minimum, maximum)
        else:
            if joint_type != "continuous":
                print(
                    warning(f"WARNING: joint {name} of type {joint_type} has no limits")
                )
            return None
