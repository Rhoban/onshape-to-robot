from sys import exit
import sys
import os
import commentjson as json
import subprocess
from .message import error, bright, info


class Config:
    def __init__(self, robot_path: str):
        self.config_file: str = robot_path + "/config.json"

        # Loading JSON configuration
        if not os.path.exists(self.config_file):
            raise Exception(f"ERROR: The file {self.config_file} can't be found")
        with open(self.config_file, "r", encoding="utf8") as stream:
            self.config: dict = json.load(stream)

        self.read_configuration()
        self.check_tools()

        # Output directory, making it if it doesn't exists
        self.output_directory: str = robot_path
        try:
            os.makedirs(self.output_directory)
        except OSError:
            pass

    def get(self, name: str, default=None, required: bool = True, values_list=None):
        """
        Gets an entry from the configuration

        Args:
            name (str): entry name
            default: default fallback value if the entry is not present. Defaults to None.
            required (bool, optional): whether the configuration entry is required. Defaults to False.
            values_list: list of allowed values. Defaults to None.
        """
        if name in self.config:
            value = self.config[name]
            if values_list is not None and value not in values_list:
                raise Exception(
                    f"Value for {name} should be onf of: {','.join(values_list)}"
                )
            return value
        elif required and default is None:
            raise Exception(f"ERROR: missing required key {name} in config")

        return default

    def read_configuration(self):
        """
        Load and check configuration entries
        """

        # Main settings
        self.document_id: str = self.get("documentId")
        self.version_id: str | None = self.get("versionId", required=False)
        self.workspace_id: str | None = self.get("workspaceId", required=False)

        if self.version_id and self.workspace_id:
            raise Exception("You can't specify workspaceId and versionId")

        self.draw_frames: bool = self.get("drawFrames", False)
        self.draw_collisions: bool = self.get("drawCollisions", False)
        self.assembly_name: str = self.get("assemblyName", required=False)
        self.output_format: str = self.get("outputFormat", "urdf")
        self.use_fixed_links: bool = self.get("useFixedLinks", False)
        self.configuration: str = self.get("configuration", "default")
        self.ignore_limits: bool = self.get("ignoreLimits", False)

        # Joint efforts
        self.joint_max_effort: float | dict = self.get("jointMaxEffort", 1)
        self.joint_max_velocity: float | dict = self.get("jointMaxVelocity", 20)
        self.no_dynamics: bool = self.get("noDynamics", False)

        # Ignore / whitelists
        self.ignore: list[str] = self.get("ignore", [])
        self.whitelist: list[str] | None = self.get("whitelist", required=False)

        # Color override
        self.color: str | None = self.get("color", required=False)

        # STL merge / simplification
        self.merge_stls = self.get(
            "mergeSTLs", "no", values_list=["no", "visual", "collision", "all"]
        )
        self.max_stl_size = self.get("maxSTLSize", 3)
        self.simplify_stls = self.get(
            "simplifySTLs", "no", values_list=["no", "visual", "collision", "all"]
        )

        # Post-import commands
        self.post_import_commands: list[str] = self.get("postImportCommands", [])

        # Whether to use collision configuration
        self.use_collisions_configurations: bool = self.get(
            "useCollisionsConfigurations", True
        )

        # ROS support
        self.package_name: str = self.get("packageName", "")
        self.add_dummy_base_link: bool = self.get("addDummyBaseLink", False)
        self.robot_name: str = self.get("robotName", "onshape")

        # Additional XML
        self.additional_xml: str = ""
        additional_xml_file: str = ""
        if self.output_format == "urdf":
            additional_xml_file = self.get("additionalUrdfFile", "")
        else:
            additional_xml_file = self.get("additionalSdfFile", "")
        if additional_xml_file:
            self.additional_xml = open(additional_xml_file, "r").read()

        # Dynamics override
        self.dynamics_override = {}
        overrides = self.get("dynamics", {})
        for key, entry in overrides.items():
            if entry == "fixed":
                self.dynamics_override[key.lower()] = {
                    "com": [0, 0, 0],
                    "mass": 0,
                    "inertia": [0, 0, 0, 0, 0, 0, 0, 0, 0],
                }
            else:
                self.dynamics_override[key.lower()] = entry

    def check_tools(self):
        self.check_meshlab()

    def check_meshlab(self):
        print(bright("* Checking MeshLab presence..."))
        if not os.path.exists("/usr/bin/meshlabserver") != 0:
            print(
                error("No /usr/bin/meshlabserver, disabling STL simplification support")
            )
            print(info("TIP: consider installing meshlab:"))
            print(info("sudo apt-get install meshlab"))
            self.simplify_stls = False
