# apogee.py
# Author: Alex Kult
# Description: An apogee prediction algorithm for the ACS module using flight state data
# Date: 6-9-2025
# Copyright Alpha Kappa Sigma

# --- Inputs ---
t_step = 0.1  # Trajectory prediction time step [s]

# --- Imports ---
import apogee_lib as apg


# --- Functions ---
def apogee_pred(state):
    alt_lst = [state[0, 0]]
    time = 0

    while state[0, 1] > 0:
        state = apg.rk4_step(state, t_step)
        time += t_step
        alt_lst.append(state[0, 0])

    apogee = alt_lst[-1]

    return apogee
