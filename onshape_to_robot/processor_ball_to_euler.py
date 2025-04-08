import numpy as np
from .processor import Processor
from .config import Config
from .robot import Robot, Link, Joint
from .message import info
import fnmatch


class ProcessorBallToEuler(Processor):
    """
    Turn ball joints into euler roll/pitch/yaw joints.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # Check if it is enabled in configuration
        self.ball_to_euler: bool | list = config.get("ball_to_euler", False)
        self.ball_to_euler_order: bool | list = config.get(
            "ball_to_euler_order",
            "xyz",
            values_list=["xyz", "xzy", "zyx", "zxy", "yxz", "yzx"],
        )

    def should_replace(self, joint: Joint) -> bool:
        if self.ball_to_euler == True:
            return True
        elif isinstance(self.ball_to_euler, list):
            for entry in self.ball_to_euler:
                if fnmatch.fnmatch(entry, joint.name):
                    return True
        return False

    def process(self, robot: Robot):
        if self.ball_to_euler:
            print(info(f"Replacing balls to euler ({self.ball_to_euler})"))
            replaced_joints: list[Joint] = []

            # Searching for ball joints to replace
            for joint in robot.joints:
                if joint.joint_type == Joint.BALL and self.should_replace(joint):
                    parent = joint.parent
                    children = {}
                    for letter in self.ball_to_euler_order[:-1]:
                        body_name = f"{joint.name}_link_{letter}"
                        new_link = Link(body_name)
                        children[letter] = new_link
                        robot.links.append(new_link)
                    children[self.ball_to_euler_order[-1]] = joint.child

                    for letter in self.ball_to_euler_order:
                        if letter == "x":
                            axis = np.array([1.0, 0.0, 0.0])
                        elif letter == "y":
                            axis = np.array([0.0, 1.0, 0.0])
                        elif letter == "z":
                            axis = np.array([0.0, 0.0, 1.0])

                        new_joint = Joint(
                            name=f"{joint.name}_{letter}",
                            joint_type=Joint.REVOLUTE,
                            parent=parent,
                            child=children[letter],
                            T_world_joint=joint.T_world_joint,
                            axis=axis,
                        )
                        robot.joints.append(new_joint)
                        parent = children[letter]

                    replaced_joints.append(joint)

            # Removing replaced joints
            for joint in replaced_joints:
                robot.joints.remove(joint)
