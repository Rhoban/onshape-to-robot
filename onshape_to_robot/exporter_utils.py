import math
import numpy as np
from xml.sax.saxutils import escape


def xml_escape(unescaped: str) -> str:
    """
    Escapes XML characters in a string so that it can be safely added to an XML file
    """
    return escape(unescaped, entities={"'": "&apos;", '"': "&quot;"})


def rotation_matrix_to_rpy(R):
    """
    Converts a rotation matrix to rpy Euler angles
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
