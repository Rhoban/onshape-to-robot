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
            base_link = robot.get_base_link()
            new_base = Link("base_link")

            robot.links = [new_base] + robot.links
            robot.joints.append(
                Joint("base_link_to_base", "fixed", new_base, base_link, np.eye(4))
            )
