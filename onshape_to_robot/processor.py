from .config import Config
from .robot import Robot


class Processor:
    def __init__(self, config: Config):
        self.config: Config = config
        pass

    def process(self, robot: Robot):
        pass
