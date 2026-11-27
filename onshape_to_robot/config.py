from __future__ import annotations
import numpy as np
import re
import os
import sys
import urllib.parse
import commentjson as json


class Config:
    def __init__(self, robot_path: str, safe: bool = False):
        self.safe: bool = safe
        self.config_file: str = robot_path

        if os.path.isdir(robot_path):
            self.config_file += os.path.sep + "config.json"

        # Loading JSON configuration
        if not os.path.exists(self.config_file):
            raise Exception(f"ERROR: The file {self.config_file} can't be found")
        with open(self.config_file, "r", encoding="utf8") as stream:
            self.config: dict = json.load(stream)

        # Loaded processors
        self.processors: list = []

        self.read_configuration()

        # Output directory, making it if it doesn't exists
        self.output_directory: str = os.path.dirname(os.path.abspath(self.config_file))

        if self.robot_name is None:
            # Name of the robot directory (os.path, so that it also works with
            # Windows paths)
            self.robot_name = os.path.basename(
                os.path.dirname(os.path.abspath(self.config_file))
            )

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
                    f"Value for {name} should be one of: {','.join(values_list)}"
                )
            return value
        elif required and default is None:
            raise Exception(f"ERROR: missing required key {name} in config")

        return default

    def printable_version(self) -> str:
        if self.url is not None:
            return self.url
        else:
            version = f"document_id: {self.document_id}"
            if self.version_id:
                version += f" / version_id: {self.version_id}"
            elif self.workspace_id:
                version += f" / workspace_id: {self.workspace_id}"

            return version

    def parse_url(self):
        # The query string and fragment (e.g. "?renderMode=0", added when the
        # URL is copied from the browser) are not part of the element id
        path = urllib.parse.urlparse(self.url).path
        match = re.search(r"/([^/]+)/([wvm])/([^/]+)/e/([^/]+)/?$", path)

        if not self.url.startswith("https://") or match is None:
            raise Exception(f"Invalid URL: {self.url}")

        document_id, wvm, wvm_id, element_id = match.groups()
        if wvm == "m":
            raise Exception(
                f"Microversion URLs (/m/) are not supported, use the URL of a "
                f"workspace (/w/) or a version (/v/): {self.url}"
            )

        self.document_id = document_id
        if wvm == "w":
            self.workspace_id = wvm_id
        else:
            self.version_id = wvm_id
        self.element_id = element_id

    def asset_path(self, asset_name: str) -> str:
        return f"{self.output_directory}/{self.assets_directory}/{asset_name}"

    def read_configuration(self):
        """
        Load and check configuration entries
        """

        # Robot name
        self.robot_name: str = self.get("robot_name", None, required=False)
        self.output_filename: str = self.get("output_filename", "robot")
        # Securing filename
        self.output_filename = "".join(
            c for c in self.output_filename if c.isalnum() or c in ("_", "-")
        ).rstrip()
        self.assets_directory: str = self.get("assets_directory", "assets")

        # Main settings
        self.document_id: str = self.get("document_id", required=False)
        self.version_id: str | None = self.get("version_id", required=False)
        self.workspace_id: str | None = self.get("workspace_id", required=False)
        self.element_id: str | None = self.get("element_id", required=False)

        if self.version_id and self.workspace_id:
            raise Exception("You can't specify workspace_id and version_id")

        self.url: str = self.get("url", None, required=False)
        if self.url is not None:
            self.parse_url()

        if self.url is None and self.document_id is None:
            raise Exception("You need to specify either a url or a document_id")

        self.draw_frames: bool = self.get("draw_frames", False)

        self.assembly_name: str = self.get("assembly_name", required=False)
        self.output_format: str = self.get("output_format")
        self.configuration: str | dict = self.get("configuration", "default")
        self.ignore_limits: bool = self.get("ignore_limits", False)

        if isinstance(self.configuration, dict):
            self.configuration = ";".join(
                [f"{k}={v}" for k, v in self.configuration.items()]
            )

        # Joint specs
        self.joint_properties: dict = self.get("joint_properties", {})
        self.rename_dofs: dict[str, str] = self.get("rename_dofs", {})
        self.geom_properties: dict = self.get("geom_properties", {})
        self.no_dynamics: bool = self.get("no_dynamics", False)

        # Ignore / whitelists
        self.ignore: list[str] = self.get("ignore", {})
        if isinstance(self.ignore, list):
            self.ignore = {entry: "all" for entry in self.ignore}

        # Color override
        self.color: str | None = self.get("color", required=False)

        # Post-import commands
        self.post_import_commands: list[str] = self.get("post_import_commands", [])

        # Whether to include configuration suffix in part names
        self.include_configuration_suffix: bool = self.get(
            "include_configuration_suffix", True
        )

        # Number of decimals to keep for small numbers
        self.round_decimals = self.get("round_decimals", 12)

        # Loading processors
        from . import processors

        loaded_modules = {}
        processors_list: list[str] | None = self.get("processors", None, required=False)
        if processors_list is None or self.safe:
            self.processors = [
                processor(self)
                for processor in processors.default_processors
                if (processor.is_safe or not self.safe)
            ]
        else:
            for entry in processors_list:
                parts = entry.split(":")

                if len(parts) == 1:
                    processor = eval(f"processors.{entry}")
                else:
                    module, cls = parts
                    if module not in loaded_modules:
                        # Custom processors can be in the current directory or
                        # in the robot directory, which are not in the import
                        # path when running the onshape-to-robot command
                        for directory in (
                            os.getcwd(),
                            os.path.dirname(os.path.abspath(self.config_file)),
                        ):
                            if directory not in sys.path:
                                sys.path.append(directory)
                        loaded_modules[module] = __import__(module, fromlist=[cls])
                    processor = getattr(loaded_modules[module], cls)

                if processor is None:
                    raise Exception(f"ERROR: Processor {entry} not found")

                self.processors.append(processor(self))

    def round(self, object: float | list | tuple | np.ndarray):
        """
        Round the given number or list of numbers using the configuration decimals
        """
        if isinstance(object, float):
            return round(object, self.round_decimals)
        elif isinstance(object, np.ndarray):
            return object.round(self.round_decimals)
        else:
            original_type = type(object)
            return original_type(np.array(object).round(self.round_decimals))