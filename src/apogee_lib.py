# apogee_lib.py
# Author: Alex Kult
# Description: Library of functions used by apogee.py, an apogee prediction algorithm for the ACS module
# Date: 6-9-2025
# Copyright Alpha Kappa Sigma

# --- Inputs ---
cfd_csv_file = r"cfd.csv"

# --- Imports ---
import numpy as np
from scipy.interpolate import RegularGridInterpolator

# --- Initialization ---
# CFD Force Interpolation Initialization
cfd_data = np.loadtxt(cfd_csv_file, delimiter=",", dtype=float, skiprows=1)
acs_angles = np.unique(cfd_data[:, 0])
atk_angles = np.unique(cfd_data[:, 1])
mach_numbers = np.unique(cfd_data[:, 2])
axial_forces = np.full((len(acs_angles), len(atk_angles), len(mach_numbers)), np.nan)
normal_forces = np.full((len(acs_angles), len(atk_angles), len(mach_numbers)), np.nan)

for i in range(len(cfd_data)):
    acs_ang, atk_ang, mach = cfd_data[i, :3]
    axial_force = cfd_data[i, 3]
    normal_force = cfd_data[i, 4]

    i_idx = np.where(acs_angles == acs_ang)[0][0]
    j_idx = np.where(atk_angles == atk_ang)[0][0]
    k_idx = np.where(mach_numbers == mach)[0][0]

    axial_forces[i_idx, j_idx, k_idx] = axial_force
    normal_forces[i_idx, j_idx, k_idx] = normal_force

axial_interp = RegularGridInterpolator(
    (acs_angles, atk_angles, mach_numbers),
    axial_forces,
    bounds_error=False,
    fill_value=None,
)
normal_interp = RegularGridInterpolator(
    (acs_angles, atk_angles, mach_numbers),
    normal_forces,
    bounds_error=False,
    fill_value=None,
)


# --- Functions ---
def acceleration(acs_ang, state):
    # Disecting state matrix
    alt = state[0, 0]
    vel = state[:, 1]
    zenith = state[2, 2]

    # Gravitational Force
    grav_acc = np.array([-c.g, 0, 0])

    # Aerodynamic State
    temp_k = e.temp(alt)
    speed_of_sound = np.sqrt(c.gamma * c.R * temp_k)

    vel_rel = vel - e.grad_wind
    mach = mth.mag(vel_rel) / speed_of_sound

    # Calculate Aerodynamic Forces
    if mach >= 0.025:
        # Angle of Attack using zenith angle and velocity angle
        lift_state = True
        atk_ang = zenith - abs(np.arctan(vel_rel[1] / vel_rel[0]))
        if atk_ang < 0:
            lift_state = False
        atk_ang = abs(atk_ang)

        # Aerodynamic Forces and Moments
        aero_point = np.array([acs_ang, np.degrees(atk_ang), mach])
        axial_force_mag = axial_interp(aero_point)[0]
        normal_force_mag = normal_interp(aero_point)[0]

        axial_force = axial_force_mag * np.array([-np.cos(zenith), -np.sin(zenith), 0])
        normal_force = normal_force_mag * np.array([-np.sin(zenith), np.cos(zenith), 0])

        aero_mom = -normal_force_mag * v.cp_cg
        aero_mom *= 0.2  # Dampening Torque (From Teasley)

        ang_acc = aero_mom / v.mom_inertia

        # Opposite Direction of lift force and moment if angle of attack is negative
        if not lift_state:
            normal_force = -normal_force
            ang_acc = -ang_acc

        aero_acc = (axial_force + normal_force) / v.dry_mass

        acc = grav_acc + aero_acc
    else:
        acc = grav_acc
        ang_acc = 0

    return acc, ang_acc


# State Derivatives
def sys_drvs(state):
    vel = state[:, 1]
    ang_vel = state[2, 3]
    acc, ang_acc = acceleration(0, state)

    dx_dt = vel
    dv_dt = acc
    dang_dt = ang_vel
    dangvel_dt = ang_acc

    state_drv = np.zeros((3, 4))
    state_drv[:, 0] = dx_dt
    state_drv[:, 1] = dv_dt
    state_drv[2, 2] = dang_dt
    state_drv[2, 3] = dangvel_dt
    return state_drv


# Uses the RK4 method to numerically integrate state over time
def rk4_step(state, t_step):
    k1 = sys_drvs(state)
    k2 = sys_drvs(state + 0.5 * k1 * t_step)
    k3 = sys_drvs(state + 0.5 * k2 * t_step)
    k4 = sys_drvs(state + k3 * t_step)

    state_new = state + t_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return state_new
