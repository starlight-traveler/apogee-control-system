#convert.py
#Author: Alex Kult
#Description: Library of conversion functions used by flight.py to convert data from the body frame to global frame
#Date: 6-9-2025
#Copyright Alpha Kappa Sigma

# --- Imports ---
import numpy as np

# --- Functions ---
def quatern2euler(q): #Converts quaternion to euler angles
    # Separate Components
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]

    # Precompute elements of the rotation matrix
    R11 = 2 * w**2 - 1 + 2 * x**2
    R21 = 2 * (x * y - w * z)
    R31 = 2 * (x * z + w * y)
    R32 = 2 * (y * z - w * x)
    R33 = 2 * w**2 - 1 + 2 * z**2

    # Compute Euler angles
    phi = np.arctan2(R32, R33)
    theta = -np.arctan(R31 / np.sqrt(1 - R31**2))
    psi = np.arctan2(R21, R11)

    return psi, theta, phi #Yaw, pitch, roll

def euler2zenith(euler): #Converts euler angle to zenith angle
    theta = euler[1]
    phi = euler[2]
    zenith = np.arccos(np.cos(theta)*np.cos(phi))
    return zenith