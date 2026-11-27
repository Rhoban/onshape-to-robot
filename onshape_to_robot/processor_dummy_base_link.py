import numpy as np
from .processor import Processor
from .config import Config
from .robot import Robot, Link, Joint
from .message import warning


class ProcessorDummyBaseLink(Processor):
    """
    Dummy base link processor.
    When enabled, add a fixed "base_link" link, to which the base links are attached.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.add_dummy_base_link: bool = config.get("add_dummy_base_link", False)

    def process(self, robot: Robot):
        if self.add_dummy_base_link:
            # The name of an existing link (or frame, which is a link in URDF)
            # can't be reused
            used_names = set()
            for link in robot.links:
                used_names.add(link.name)
                used_names.update(link.frames)
            name = "base_link"
            if name in used_names:
                name = "dummy_base_link"
                k = 1
                while name in used_names:
                    k += 1
                    name = f"dummy_base_link_{k}"
                print(
                    warning(
                        f"WARNING: a link or frame is already named base_link, naming the dummy base link {name}"
                    )
                )

            new_base_link = Link(name)
            new_base_link.fixed = True
            robot.links.append(new_base_link)

            for base_link in robot.base_links:
                robot.joints.append(
                    Joint(
                        f"{name}_to_{base_link.name}",
                        "fixed",
                        new_base_link,
                        base_link,
                        np.eye(4),
                    )
                )

            robot.base_links = [new_base_link]
