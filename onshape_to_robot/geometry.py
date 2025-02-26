import numpy as np


class Geometry:
    def __init__(
        self,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        self.color: np.ndarray = color
        self.visual: bool = visual
        self.collision: bool = collision

    def is_type(self, what: str):
        if what == "visual":
            return self.visual
        elif what == "collision":
            return self.collision
        return False


class Mesh(Geometry):
    def __init__(
        self,
        filename: str,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        super().__init__(color, visual, collision)
        self.filename: str = filename


class Shape(Geometry):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        super().__init__(color, visual, collision)
        self.T_part_shape: np.ndarray = T_part_shape


class Box(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        size: np.ndarray,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        super().__init__(T_part_shape, color, visual, collision)
        self.size: np.ndarray = size


class Cylinder(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        length: float,
        radius: float,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        super().__init__(T_part_shape, color, visual, collision)
        self.length: float = length
        self.radius: float = radius


class Sphere(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        radius: float,
        color: np.ndarray = np.array([0.5, 0.5, 0.5]),
        visual: bool = True,
        collision: bool = True,
    ):
        super().__init__(T_part_shape, color, visual, collision)
        self.radius: float = radius
