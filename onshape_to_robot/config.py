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

        # Output directory, making it if it doesn't exists
        self.output_directory: str = robot_path
        try:
            os.makedirs(self.output_directory)
        except OSError:
            pass

    def to_camel_case(self, snake_str: str) -> str:
        """
        Converts a string to camel case
        """
        components = snake_str.split("_")
        return components[0] + "".join(x.title() for x in components[1:])

    def get(self, name: str, default=None, required: bool = True, values_list=None):
        """
        Gets an entry from the configuration

        Args:
            name (str): entry name
            default: default fallback value if the entry is not present. Defaults to None.
            required (bool, optional): whether the configuration entry is required. Defaults to False.
            values_list: list of allowed values. Defaults to None.
        """
        camel_name = self.to_camel_case(name)

        if name in self.config or camel_name in self.config:
            if name in self.config:
                value = self.config[name]
            else:
                value = self.config[camel_name]

            if values_list is not None and value not in values_list:
                raise Exception(
                    f"Value for {name} should be onf of: {','.join(values_list)}"
                )
            return value
        elif required and default is None:
            raise Exception(f"ERROR: missing required key {name} in config")

        return default

    def printable_version(self) -> str:
        version = f"document_id: {self.document_id}"
        if self.version_id:
            version += f" / version_id: {self.version_id}"
        elif self.workspace_id:
            version += f" / workspace_id: {self.workspace_id}"

        return version

    def read_configuration(self):
        """
        Load and check configuration entries
        """

        # Robot name
        self.robot_name: str = self.get("robot_name", "onshape")
        self.output_filename: str = self.get("output_filename", "robot")

        # Main settings
        self.document_id: str = self.get("document_id")
        self.version_id: str | None = self.get("version_id", required=False)
        self.workspace_id: str | None = self.get("workspace_id", required=False)

        if self.version_id and self.workspace_id:
            raise Exception("You can't specify workspace_id and version_id")

        self.draw_frames: bool = self.get("draw_frames", False)

        self.assembly_name: str = self.get("assembly_name", required=False)
        self.output_format: str = self.get("output_format", "urdf")
        self.configuration: str = self.get("configuration", "default")
        self.ignore_limits: bool = self.get("ignore_limits", False)

        # Joint specs
        self.joint_properties: dict = self.get("joint_properties", {})
        self.no_dynamics: bool = self.get("no_dynamics", False)

        # Ignore / whitelists
        self.ignore: list[str] = self.get("ignore", [])
        self.whitelist: list[str] | None = self.get("whitelist", required=False)

        # Color override
        self.color: str | None = self.get("color", required=False)

        # Post-import commands
        self.post_import_commands: list[str] = self.get("post_import_commands", [])
