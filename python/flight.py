# flight.py
# Author: Alex Kult
# Description: A state determination and control execution algorithm for the ACS module
# Date: 6-9-2025
# Copyright Alpha Kappa Sigma

# --- Inputs ---
apg_target = 1550  # [m]

# flight_log_file = r"Raw Flight Data/Full Scale Flight 1.csv"
# flight_log_file = r"Raw Flight Data/Full Scale Flight 2.csv"
flight_log_file = r"Raw Flight Data/Full Scale Flight 3.csv"
# flight_log_file = r"Raw Flight Data/Huntsville Flight.csv"

# --- Imports ---
import numpy as np
import apogee as apg
import filter as kal
import convert as con
import math_lib as mth
import constants as c
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import time as simtime

# --- Files ---
flight_log = np.loadtxt(flight_log_file, delimiter=",", dtype=float, skiprows=1)

# --- Initialization ---
num_points = len(flight_log)
filtered_flight_log = np.zeros(
    (num_points, 13)
)  # [time, x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc_filt, y_acc_filt, z_acc_filt, x_acc_rot, y_acc_rot, z_acc_rot]

status = "ground"
zenith = 0
state = np.zeros((3, 4))

alt_lst = []
apg_lst = []
t_apg_lst = []
sim_time_lst = []

start_time = simtime.time()

# Kalman Filter Initialization
sigma_process_accel_xy = 0.5
sigma_accel_sensor_xy = 0.5

# For Z-axis (most critical for altitude/velocity)
# If velocity/position is still too high or drifts, increase sigma_process_accel_z.
# If estimated Z-accel is too noisy, increase sigma_accel_sensor_z.
# If estimated Z-position is too noisy or not tracking altimeter well, decrease sigma_altimeter_sensor.
sigma_process_accel_z = 1.0
sigma_accel_sensor_z = 0.5
sigma_altimeter_sensor = 0.5

kf_x, kf_y, kf_z, last_time = kal.initialize_kalman_filters(
    sigma_accel_sensor_xy, sigma_accel_sensor_z, sigma_altimeter_sensor
)

# Loop through points in flight log
for idx in range(num_points):
    flight_data = flight_log[idx, :]

    # Sensor inputs
    alt_meas = flight_data[1] / c.m2ft
    acc_bno_meas = flight_data[4:7]
    acc_icm_meas = flight_data[17:20]
    quaternion = flight_data[13:17]
    gyro = flight_data[10:13]
    time = flight_data[0]

    # Determine acceleration sensor reading
    if status == "ground" or status == "burn":
        acc_meas = acc_icm_meas
    else:
        acc_meas = acc_bno_meas

        # Consistent sensor frame
        x, y, z = acc_meas[0], acc_meas[1], acc_meas[2]
        acc_meas[0] = -y
        acc_meas[1] = x
        acc_meas[2] = z

    # Prepare sensor fusion
    if status == "ground":
        quaternion_old = flight_log[idx - 1, 13:17]  # last quaternion

    if status == "burn" or status == "coast":
        # At high accelerations, the BNO085's orientation determination is unreliable.
        # For this reason, manual sensor fusion is used.
        # NOTE: Gyro drift starts to kick in pretty heavily after around 15-20 seconds.
        # Calculate quaternions manually using gyro (fusion)
        # Use more accurate fusion algorithm if we have enough processing power
        dt = time - last_time
        quaternion = kal.teasley_filter(quaternion_old, gyro, dt)
        quaternion_old = quaternion

    # Sensor frame to body frame
    x, y, z = acc_meas[0], acc_meas[1], acc_meas[2]
    acc_meas[0] = z
    acc_meas[1] = y
    acc_meas[2] = x

    # Calculate euler angles and zenith angle [rad]
    zenith_old = zenith
    euler = con.quatern2euler(quaternion)
    yaw, pitch, roll = euler
    zenith = con.euler2zenith(euler)

    # Body frame to global frame
    r = Rotation.from_euler("y", np.degrees(zenith) - 90, degrees=True)
    acc_i = r.apply(acc_meas)
    acc_i[2] -= c.g

    ax_meas, ay_meas, az_meas = acc_i

    # Use kalman filter on acceleration and altitude data
    kf_x, kf_y, kf_z, last_time, current_estimates = kal.update_kalman_filters(
        kf_x,
        kf_y,
        kf_z,
        last_time,
        time,
        ax_meas,
        ay_meas,
        az_meas,
        alt_meas,
        sigma_process_accel_xy,
        sigma_process_accel_z,
    )

    # Store filtered results
    filtered_flight_log[idx, :] = np.hstack((current_estimates, acc_i))
    pos_z = current_estimates[3]
    vel_z = current_estimates[6]
    acc_z = current_estimates[9]

    # Apogee Prediction
    if status == "coast":
        loop_start_time = simtime.time()

        # Downrange Conditions
        pos_horz = mth.mag(np.array(current_estimates[1:3]))
        vel_horz = mth.mag(np.array(current_estimates[4:6]))

        # Estimate angular velocty along pitch axis (improve later with gyro sensor readings)
        omega = (zenith - zenith_old) / dt

        # Initializing state matrix
        state[:2, 0] = [pos_z, pos_horz]
        state[:2, 1] = [vel_z, vel_horz]
        state[2, 2] = zenith
        state[2, 3] = omega

        # Predict apogee
        apogee = apg.apogee_pred(state)

        loop_end_time = simtime.time()
        loop_time = loop_end_time - loop_start_time

        # Append info to lists for plotting
        alt_lst.append(pos_z)
        apg_lst.append(apogee)
        t_apg_lst.append(time)
        sim_time_lst.append(loop_time)

    # State determination
    if acc_z > 5 and abs(pos_z) > 1 and status == "ground":
        status = "burn"
        t_burn = time
        print(f"Engine burn at t = {time:.4f} seconds.")
    elif acc_z < 0 and pos_z < apg_target and vel_z > 0 and status == "burn":
        status = "coast"
        t_burnout = time
        print(f"Engine burnout at t = {time:.4f} seconds.")
    elif acc_z < 0 and pos_z >= apg_target and status == "coast":
        status = "overshoot"
        print(f"Overshoot at t = {time:.4f} seconds.")
    elif acc_z < 0 and vel_z <= 0 and (status == "overshoot" or status == "coast"):
        status = "descent"
        apogee = pos_z
        t_apogee = time
        print(f"Apogee of {apogee:.4f} m reached at t = {t_apogee:.4f} seconds.")

# Calculating Simulation Time and Frequency Information
end_time = simtime.time()
tot_time = end_time - start_time
print(f"Total Simulation Time: {tot_time:.4f} seconds.")

t_sim_lst = [t_apg_lst[t] for t in range(len(sim_time_lst)) if sim_time_lst[t] != 0]
sim_time_lst = [val for val in sim_time_lst if val != 0]
hertz_lst = [1 / t if t != 0 else 0 for t in sim_time_lst]
print(f"Minimum Simulation Hertz: {np.min(hertz_lst):.4f} Hz.")

# --- Plotting ---
# Plotting Apogee Prediction Throughout Coast
plt.plot(t_apg_lst, alt_lst, label="Altitude")
plt.plot(t_apg_lst, apg_lst, label="Predicted Apogee")
plt.axhline(apogee, label="Apogee", color="g")
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")
plt.title("Apogee Prediction")
plt.legend()
plt.grid()
plt.show()

# Plotting Filtered Position, Velocity, and Acceleration Data (Global Frame)
plt.figure(figsize=(15, 15))
times = filtered_flight_log[:, 0]

# X-axis plots
plt.subplot(3, 3, 1)
plt.plot(times, filtered_flight_log[:, 1], label="Estimated X Position")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.ylabel("X Position [m]")
plt.title("X-axis Estimates")
plt.legend()
plt.grid()

plt.subplot(3, 3, 4)
plt.plot(times, filtered_flight_log[:, 4], label="Estimated X Velocity")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.ylabel("X Velocity [m/s]")
plt.legend()
plt.grid()

plt.subplot(3, 3, 7)
plt.plot(times, filtered_flight_log[:, 10], label="Raw X Acceleration", alpha=0.6)
plt.plot(times, filtered_flight_log[:, 7], label="Estimated X Acceleration")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.xlabel("Time [s]")
plt.ylabel("X Acceleration [m/s^2]")
plt.legend()
plt.grid()

# Y-axis plots
plt.subplot(3, 3, 2)
plt.plot(times, filtered_flight_log[:, 2], label="Estimated Y Position")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.ylabel("Y Position [m]")
plt.title("Y-axis Estimates")
plt.legend()
plt.grid()

plt.subplot(3, 3, 5)
plt.plot(times, filtered_flight_log[:, 5], label="Estimated Y Velocity")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.ylabel("Y Velocity [m/s]")
plt.legend()
plt.grid()

plt.subplot(3, 3, 8)
plt.plot(times, filtered_flight_log[:, 11], label="Raw Y Acceleration", alpha=0.6)
plt.plot(times, filtered_flight_log[:, 8], label="Estimated Y Acceleration")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.xlabel("Time [s]")
plt.ylabel("Y Acceleration [m/s^2]")
plt.legend()
plt.grid()

# Z-axis plots
plt.subplot(3, 3, 3)
plt.plot(times, flight_log[:, 1] / c.m2ft, label="Raw Altimeter", alpha=0.6)
plt.plot(times, filtered_flight_log[:, 3], label="Estimated Z Position")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.axhline(apogee, label="Apogee", color="g")
plt.axhline(apg_target, label="Apogee Target", color="y")
plt.ylabel("Z Position [m]")
plt.title("Z-axis Estimates")
plt.legend()
plt.grid()

plt.subplot(3, 3, 6)
plt.plot(times, filtered_flight_log[:, 6], label="Estimated Z Velocity")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.ylabel("Z Velocity [m/s]")
plt.legend()
plt.grid()

plt.subplot(3, 3, 9)
plt.plot(times, filtered_flight_log[:, 12], label="Raw Z Acceleration", alpha=0.6)
plt.plot(times, filtered_flight_log[:, 9], label="Estimated Z Acceleration")
plt.axvline(t_burn, label="Burn", color="k")
plt.axvline(t_burnout, label="Burnout", color="r")
plt.axvline(t_apogee, label="Apogee", color="g")
plt.xlabel("Time [s]")
plt.ylabel("Z Acceleration [m/s^2]")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

# Apogee Prediction Frequency Plot
plt.plot(t_sim_lst, hertz_lst)
plt.xlabel("Time [s]")
plt.ylabel("Hertz [s^(-1)]")
plt.title("Apogee Prediction Simulation Frequency")
plt.grid()
plt.show()
