from .processor import Processor
from .config import Config
from .robot import Robot, Link, Joint


class ProcessorFixedLinks(Processor):
    """
    Fixed links processor.
    When enabled, crawl all the links, and split them into sublinks containing all one part.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.use_fixed_links: bool = config.get("use_fixed_links", False)

    def process(self, robot: Robot):
        if self.use_fixed_links:
            new_links = []
            for link in robot.links:
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
