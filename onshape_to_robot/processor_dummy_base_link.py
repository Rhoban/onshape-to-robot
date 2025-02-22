import numpy as np
from .processor import Processor
from .config import Config
from .robot import Robot, Link, Joint


class ProcessorDummyBaseLink(Processor):
    """
    Fixed links processor.
    When enabled, crawl all the links, and split them into sublinks containing all one part.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.add_dummy_base_link: bool = config.get("add_dummy_base_link", False)

    def process(self, robot: Robot):
        if self.add_dummy_base_link:
            new_base_link = Link("base_link")
            new_base_link.fixed = True
            robot.links.append(new_base_link)

            for base_link in robot.base_links:
                robot.joints.append(
                    Joint(
                        "base_link_to_" + base_link.name,
                        "fixed",
                        new_base_link,
                        base_link,
                        np.eye(4),
                    )
                )

            robot.base_links = [new_base_link]
