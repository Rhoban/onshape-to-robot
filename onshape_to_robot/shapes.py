import numpy as np


class Shape:
    def __init__(self, T_part_shape: np.ndarray):
        self.T_part_shape: np.ndarray = T_part_shape


class Box(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        size: np.ndarray,
    ):
        super().__init__(T_part_shape)
        self.size: np.ndarray = size


class Cylinder(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        length: float,
        radius: float,
    ):
        super().__init__(T_part_shape)
        self.length: float = length
        self.radius: float = radius


class Sphere(Shape):
    def __init__(
        self,
        T_part_shape: np.ndarray,
        radius: float,
    ):
        super().__init__(T_part_shape)
        self.radius: float = radius
