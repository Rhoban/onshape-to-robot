from .processor import Processor
from .config import Config
from .robot import Robot, Link, Joint
from .message import info
import fnmatch


class ProcessorFixedLinks(Processor):
    """
    Fixed links processor.
    When enabled, crawl all the links, and split them into sublinks containing all one part.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.use_fixed_links: bool | list = config.get("use_fixed_links", False)

    def should_fix_links(self, link_name: str) -> bool:
        if self.use_fixed_links == True:
            return True
        elif isinstance(self.use_fixed_links, list):
            for entry in self.use_fixed_links:
                if fnmatch.fnmatch(link_name, entry):
                    return True
        return False

    def process(self, robot: Robot):
        if self.use_fixed_links:
            print(info(f"Using fixed links ({self.use_fixed_links})"))
            new_links = []
            for link in robot.links:
                if self.should_fix_links(link.name):
                    for part in link.parts:
                        part_link = Link(f"{link.name}_{part.name}")
                        part_link.parts = [part]
                        new_links.append([link, part_link])
                    link.parts = []

            for parent_link, new_link in new_links:
                robot.links.append(new_link)
                robot.joints.append(
                    Joint(
                        f"{new_link.name}_fixed",
                        Joint.FIXED,
                        parent_link,
                        new_link,
                        new_link.parts[0].T_world_part,
                    )
                )
