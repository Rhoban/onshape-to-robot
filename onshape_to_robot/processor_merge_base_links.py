from .processor import Processor
from .config import Config
from .robot import Robot, Link
from .message import info


class ProcessorMergeBaseLinks(Processor):
    """
    Merge links processor
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.merge_base_links: bool | list = config.get("merge_base_links", False)

    def process(self, robot: Robot):
        if self.merge_base_links:
            print(info(f"Using merge links ({self.merge_base_links})"))

            # Removing joints
            robot.joints = []

            # Creating a new link
            root_link = Link("root")
            root_link.fixed = True

            for link in robot.base_links:
                root_link.parts += link.parts
                root_link.frames = {**root_link.frames, **link.frames}
                robot.links.remove(link)

            robot.base_links = [root_link]
