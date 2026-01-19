# filter.py
# Author: Alex Kult
# Description: Library of filter functions used by flight.py including kalman filter and sensor fusion
# Date: 6-9-2025
# Copyright Alpha Kappa Sigma

# --- Imports ---
import numpy as np
from filterpy.kalman import KalmanFilter
import math_lib as mth


# --- Functions ---
def initialize_kalman_filters(
    sigma_accel_sensor_xy, sigma_accel_sensor_z, sigma_altimeter_sensor
):
    # --- Kalman Filter Setup for X and Y axes ---
    # State vector: [position, velocity, acceleration] (dim_x = 3)
    # Measurement: [acceleration] (dim_z = 1)
    H_accel_only = np.array([[0.0, 0.0, 1.0]])
    R_accel_only = np.array([[sigma_accel_sensor_xy**2]])

    kf_x = KalmanFilter(dim_x=3, dim_z=1)
    kf_y = KalmanFilter(dim_x=3, dim_z=1)

    # Set initial common parameters for XY filters
    for kf in [kf_x, kf_y]:
        kf.H = H_accel_only
        kf.R = R_accel_only
        # Initial State Estimate: [0, 0, 0] (position, velocity, acceleration)
        kf.x = np.array([[0.0], [0.0], [0.0]])
        # Initial Error Covariance: High uncertainty to allow quick convergence
        kf.P = np.array(
            [
                [1000.0, 0.0, 0.0],  # Position uncertainty
                [0.0, 1000.0, 0.0],  # Velocity uncertainty
                [0.0, 0.0, 10.0],
            ]
        )  # Acceleration uncertainty

    # --- Kalman Filter Setup for Z-axis (Acceleration + Altimeter) ---
    # State vector: [position, velocity, acceleration] (dim_x = 3)
    # Measurement: [acceleration, position] (dim_z = 2)
    kf_z = KalmanFilter(dim_x=3, dim_z=2)

    # Measurement Function Matrix (H) for Z-axis: Measures acceleration (index 2) and position (index 0)
    kf_z.H = np.array(
        [
            [0.0, 0.0, 1.0],  # Maps state[2] (acceleration) to measurement[0]
            [1.0, 0.0, 0.0],
        ]
    )  # Maps state[0] (position) to measurement[1]

    # Measurement Noise Covariance (R) for Z-axis: 2x2 matrix for accel and altimeter
    # Assuming no correlation between accelerometer and altimeter noise
    kf_z.R = np.array(
        [[sigma_accel_sensor_z**2, 0.0], [0.0, sigma_altimeter_sensor**2]]
    )

    # Initial State Estimate for Z-axis: [0, 0, 0] (position, velocity, acceleration)
    kf_z.x = np.array([[0.0], [0.0], [0.0]])
    # Initial Error Covariance for Z-axis: High uncertainty
    kf_z.P = np.array([[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 10.0]])

    last_time = None  # To store the previous timestamp for dt calculation

    return kf_x, kf_y, kf_z, last_time


def update_kalman_filters(
    kf_x,
    kf_y,
    kf_z,
    last_time,
    current_time,
    x_accel_raw,
    y_accel_raw,
    z_accel_raw,
    altimeter_raw,
    sigma_process_accel_xy,
    sigma_process_accel_z,
):
    if last_time is None:
        current_dt = 0.03  # Arbitrary small initial dt
    else:
        current_dt = current_time - last_time

    # Update F and Q matrices for the current dt for XY axes
    current_F_xy = np.array(
        [
            [1.0, current_dt, 0.5 * current_dt**2],
            [0.0, 1.0, current_dt],
            [0.0, 0.0, 1.0],
        ]
    )

    current_Q_xy = sigma_process_accel_xy**2 * np.array(
        [
            [0.25 * current_dt**4, 0.5 * current_dt**3, 0.5 * current_dt**2],
            [0.5 * current_dt**3, current_dt**2, current_dt],
            [0.5 * current_dt**2, current_dt, 1.0],
        ]
    )

    # Update F and Q matrices for the current dt for Z-axis
    current_F_z = np.array(
        [
            [1.0, current_dt, 0.5 * current_dt**2],
            [0.0, 1.0, current_dt],
            [0.0, 0.0, 1.0],
        ]
    )

    current_Q_z = sigma_process_accel_z**2 * np.array(
        [
            [0.25 * current_dt**4, 0.5 * current_dt**3, 0.5 * current_dt**2],
            [0.5 * current_dt**3, current_dt**2, current_dt],
            [0.5 * current_dt**2, current_dt, 1.0],
        ]
    )

    # Assign updated F and Q to each filter
    kf_x.F = current_F_xy
    kf_x.Q = current_Q_xy
    kf_y.F = current_F_xy
    kf_y.Q = current_Q_xy
    kf_z.F = current_F_z
    kf_z.Q = current_Q_z

    # --- Process X-axis ---
    kf_x.predict()
    kf_x.update(np.array([[x_accel_raw]]))

    # --- Process Y-axis ---
    kf_y.predict()
    kf_y.update(np.array([[y_accel_raw]]))

    # --- Process Z-axis (Acceleration + Altimeter) ---
    kf_z.predict()
    combined_z_measurement = np.array([[z_accel_raw], [altimeter_raw]])
    kf_z.update(combined_z_measurement)

    # Return current estimated states
    # [time, x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc]
    estimated_states = [
        current_time,
        kf_x.x[0, 0],
        kf_y.x[0, 0],
        kf_z.x[0, 0],
        kf_x.x[1, 0],
        kf_y.x[1, 0],
        kf_z.x[1, 0],
        kf_x.x[2, 0],
        kf_y.x[2, 0],
        kf_z.x[2, 0],
    ]

    return kf_x, kf_y, kf_z, current_time, estimated_states


def teasley_filter(
    quat, gyro, dt
):  # Use gyro to find quaternions. Implement more advanced fusion algorithm later
    # Gyroscope interpolation (angular velocity -> change in angular positon)
    omega = np.array([0, gyro[0], gyro[1], gyro[2]])
    dq = 0.5 * np.array(mth.quatern_prod(quat, omega))

    # Quaternion with corrected gyro
    q_new = np.array(quat) + dq * dt

    # Normalize
    q_norm = q_new / mth.mag(q_new)

    return q_norm
