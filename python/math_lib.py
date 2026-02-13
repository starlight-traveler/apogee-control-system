# math_lib.py
# Author: Alex Kult
# Description: An reference library of functions for all ACS module code files
# Date: 5-19-2025
# Copyright Alpha Kappa Sigma

# --- Imports ---
import numpy as np


# --- Functions ---
def mag(vector):
    if len(vector) == 2:
        mag = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
    elif len(vector) == 3:
        mag = np.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    elif len(vector) == 4:
        mag = np.sqrt(
            vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2 + vector[3] ** 2
        )
    else:
        raise ValueError("Non-Vector Inputted")
    return mag


def coord_rot_q(quaternion):  # Input [w, x, y, z] quaternion coefficients
    w, x, y, z = quaternion
    rot_mat = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )
    return rot_mat


def quatern_prod(a, b):
    """
    Calculates the quaternion product of quaternion a and b.
    Expects inputs in format [w, x, y, z].
    """
    q1 = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    q2 = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    q3 = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
    q4 = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]

    return [q1, q2, q3, q4]
