from __future__ import annotations
from copy import deepcopy
import numpy as np
from .geometry import Shape, Mesh


class Part:
    """
    A part is a single component of a link.
    """

    def __init__(
        self,
        name: str,
        T_world_part: np.ndarray,
        mass: float,
        com: np.ndarray,
        inertia: np.ndarray,
        meshes: list[Mesh] = [],
        shapes: list[Shape] = [],
    ):
        self.name: str = name
        self.T_world_part: np.ndarray = T_world_part
        self.mass: float = mass
        self.com: np.ndarray = com
        self.inertia: np.ndarray = inertia
        self.meshes: list[Mesh] = deepcopy(meshes)
        self.shapes: list[Shape] = deepcopy(shapes)

    def prune_unused_geometry(self):
        """
        Remove meshes or shapes that are neither visual nor collision.
        """
        self.meshes = [mesh for mesh in self.meshes if (mesh.visual or mesh.collision)]
        self.shapes = [
            shape for shape in self.shapes if (shape.visual or shape.collision)
        ]


class Link:
    """
    A link of a robot.
    """

    def __init__(self, name: str):
        self.name = name
        self.parts: list[Part] = []
        self.frames: dict[str, np.ndarray] = {}
        self.fixed: bool = False

    def get_dynamics(self, T_world_frame: np.ndarray = np.eye(4)):
        """
        Returns the dynamics (mass, com, inertia) in a given frame.
        The CoM is expressed in the required frame.
        Inertia is expressed around the CoM, aligned with the required frame.
        """
        mass = 0
        com = np.zeros(3)
        inertia = np.zeros((3, 3))
        T_frame_world = np.linalg.inv(T_world_frame)

        for part in self.parts:
            T_frame_part = T_frame_world @ part.T_world_part
            com_frame = (T_frame_part @ [*part.com, 1])[:3]
            com += com_frame * part.mass
            mass += part.mass

        if mass > 1e-9:
            com /= mass

        for part in self.parts:
            T_frame_part = T_frame_world @ part.T_world_part
            com_frame = (T_frame_part @ [*part.com, 1])[:3]
            R = T_frame_part[:3, :3]
            q = (com_frame - com).reshape((3, 1))
            # See Modern Robotics, (8.26) & (8.27)
            inertia += (
                R @ part.inertia @ R.T + ((q.T @ q) * np.eye(3) - q @ q.T) * part.mass
            )

        return mass, com, inertia


class Relation:
    """
    Represents a relation (for example a gear) with a source joint
    """

    def __init__(self, source_joint: str, ratio: float):
        self.source_joint: str = source_joint
        self.ratio: float = ratio


class Joint:
    """
    A joint connects two links.
    """

    # Joint types
    FIXED = "fixed"
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    CONTINUOUS = "continuous"
    BALL = "ball"

    def __init__(
        self,
        name: str,
        joint_type: str,
        parent: Link,
        child: Link,
        T_world_joint: np.ndarray,
        properties: dict = {},
        limits: tuple[float, float] | None = None,
        axis: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ):
        self.name: str = name
        self.joint_type: str = joint_type
        self.properties: dict = properties
        self.parent: Link = parent
        self.child: Link = child
        self.limits: tuple[float, float] | None = limits
        self.axis: np.ndarray = axis
        self.T_world_joint: np.ndarray = T_world_joint
        self.relation: Relation | None = None


class Closure:
    """
    A kinematics closure
    """

    FIXED = "fixed"
    REVOLUTE = "revolute"
    BALL = "ball"
    SLIDER = "slider"

    def __init__(self, closure_type: str, frame1: str, frame2: str):
        self.closure_type: str = closure_type
        self.frame1: str = frame1
        self.frame2: str = frame2


class Robot:
    """
    Robot representation produced after requesting Onshape API, and before
    exporting (e.g URDF, MuJoCo).
    """

    def __init__(self, name: str):
        self.name: str = name
        self.links: list[Link] = []
        self.base_links: list[Link] = []
        self.joints: list[Joint] = []
        self.closures: list[Closure] = []

    def get_link(self, name: str):
        for link in self.links:
            if link.name == name:
                return link
        raise ValueError(f"Link {name} not found")

    def get_joint(self, name: str):
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError(f"Joint {name} not found")

    def get_link_joints(self, link: Link):
        return [joint for joint in self.joints if joint.parent == link]
