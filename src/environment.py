# environment.py
# Author: Alex Kult
# Description: Environmental parameters to predict atmospheric conditions throughout flight
# Date: 6-9-2025
# Copyright Alpha Kappa Sigma

# --- Launch Conditions ---
temp_ground = 50  # Ground temperature [F]
wind_speed = 10  # Downrange wind speed [mph] (downrange speed is positive)
wind_direction = 270  # [deg] #*Use weather app
launch_direction = 260  # [deg] #*Use compass on phone

# --- Environmental Parameters ---
rough_len = 0.075  # [m] Roughness Length (~0.075 for harvested cropland)
grad_ht = 300  # [m] Gradient Height for open terrain and neutral stability
meas_ht = 10  # [m] Wind Speed Measurement Height (~10m for most weather stations)

# --- Imports ---
import constants as c
import numpy as np

# --- Wind Conditions ---
wind_ground = wind_speed * c.mph2ms
wind_vector = wind_ground * np.array(
    [np.cos(np.radians(wind_direction)), np.sin(np.radians(wind_direction))]
)
launch_vector = np.array(
    [np.cos(np.radians(launch_direction)), np.sin(np.radians(launch_direction))]
)
wind_downrange = np.dot(wind_vector, launch_vector)

# Gradient wind speed above planetary boundary layer
# NOTE: By the time the launch vehicle reaches the coast phase, it is above the planatary boundary layer (100-200m above ground in open terrain) so wind speed is "constant"
grad_speed = wind_downrange * np.log(grad_ht / rough_len) / np.log(meas_ht / rough_len)
grad_wind = np.array([0, grad_speed, 0])


# --- Temperature Conditions ---
def temp(alt):  # Temperature as a function of altitude
    temp_f = temp_ground - 0.00356 * alt * c.m2ft
    temp_k = c.F2K(temp_f)
    return temp_k
