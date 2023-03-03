from dataclasses import dataclass
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime
from scipy.stats import gaussian_kde
from numba import jit, float64, boolean
import time

# CONSTANTS

# Physical properties
DIST_REAR_AXEL = 0.85 # m
DIST_FRONT_AXEL = 0.85 # m
WHEEL_BASE = DIST_REAR_AXEL + DIST_FRONT_AXEL

MASS = 15 # kg

CORNERING_STIFF_FRONT = 16.0 * 2.0  # N/rad
CORNERING_STIFF_REAR = 17.0 * 2.0  # N/rad

YAW_INERTIA = 22 # kg/(m^2)

# Input constraints
SPEED_MIN = 2 # m/s
SPEED_MAX = 10 # m/s

STEER_ANGLE_MAX = np.radians(37) # rad
STEER_ANGLE_DELTA = np.radians(1) # rad

ACCELERATION_MIN = -5 # m/(s^2)
ACCELERATION_MAX = 5 # m/(s^2)
 
STEER_ACC_MIN = -5
STEER_ACC_MAX = 5

DT = 0.01 # (s)

""""
A note on @jit decorators:
    - First position is signature - return(param, param, ...) - of method
        - Not required but required for pre-compliation in future if used
    - nopython forces jit to compile to pure byte code (significantly faster)
    - cache caches compiled functions to reduce overhead on first runs

"""

@jit("float64[:](float64[:], float64, float64)",
     nopython=True,
     cache=True)
def next_state(state, throttle, steering):

    """
    Return future state given inputs

    state: numpy array representing current state
    throttle: ???
    steering: delta in steering angle in radians
    """

    x, y, vel_x, vel_y, yaw_angle, steer_angle = state

    # Advect bike
    x = x + vel_x * np.cos(yaw_angle) * DT - vel_y * np.sin(yaw_angle) * DT
    y = y + vel_x * np.sin(yaw_angle) * DT + vel_y * np.cos(yaw_angle) * DT

    yaw = yaw_angle * steer_angle * DT
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi # Normalize yaw angle

    lat_force_front = -CORNERING_STIFF_FRONT * np.arctan(((vel_y + DIST_FRONT_AXEL * steer_angle) / vel_x - steering))
    lat_force_rear = -CORNERING_STIFF_REAR * np.arctan((vel_y - DIST_REAR_AXEL * steer_angle) / vel_x)

    # Aerodynamic and friction coefficients
    R_x = 0.01 * vel_x
    F_aero = 1.36 * vel_x ** 2
    F_load = F_aero + R_x

    vel_x = vel_x + (throttle - lat_force_front * np.sin(steering) / MASS - F_load / MASS + vel_y * steering) * DT
    vel_y = vel_y + (lat_force_rear / MASS + lat_force_front * np.cos(steering) / MASS - vel_x * steering) * DT

    steering = steering + (lat_force_front * DIST_FRONT_AXEL * np.cos(steering) - lat_force_rear * DIST_REAR_AXEL) / YAW_INERTIA * DT

    # Advect bike
    x = x + vel_x * np.cos(yaw_angle) * DT - vel_y * np.sin(yaw_angle) * DT
    y = y + vel_x * np.sin(yaw_angle) * DT + vel_y * np.cos(yaw_angle) * DT

    return np.array([x, y, vel_x, vel_y, yaw, steering])


@jit("boolean(float64[:])",
     nopython=True,
     cache=True)
def valid_state(state):

    """
    Return whether a state is allowed
    """

    vel_x, vel_y = state[2], state[3]
    steer_angle = state[5]

    speed = np.sqrt(vel_x ** 2 + vel_y ** 2)

    return (SPEED_MIN <= speed<= SPEED_MAX and
            -STEER_ANGLE_MAX <= steer_angle <= STEER_ANGLE_MAX)


@jit("float64[:,:](float64[:])",
     nopython=True,
     cache=True)
def get_possible_states(state):

    """
    Get all possible states
    """
    
    # TODO: should these arrays be initialized outside of function?
    possible_acc = np.arange(ACCELERATION_MIN, ACCELERATION_MAX, 0.1)
    possible_steer = np.arange(-STEER_ANGLE_DELTA, STEER_ANGLE_DELTA, np.radians(0.1)) 
    
    # TODO: should array be allocated outside of function?
    max_size = possible_acc.size * possible_steer.size
    possible_states = np.empty((max_size, 6), float)

    index = 0
    for acc in possible_acc:
        for steer in possible_steer:
                next = next_state(state, acc, steer)
                if (valid_state(next)):
                    possible_states[index] = next
                    index += 1

    # Only return valid states
    return possible_states[1:index]




def possible_states_performance(iter):

    """
    Print mean runtime and standard deviation of get_possible_states()
    """
    runs = np.empty(iter, float)

    # Performance appears to be same regardless of test_state
    test_state = np.array([1000, 1000, 3, 2, np.radians(90), np.radians(-20)])

    for i in range(0, iter):
        start = time.perf_counter()
        get_possible_states(test_state)
        end = time.perf_counter()
        runs[i] = end - start

    print("Mean: " + str(np.mean(runs)) + " STD: " + str(np.std(runs)))
    return


# Testing
if __name__ == "__main__":

    # Example state
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

    start = time.perf_counter()
    possible_states = get_possible_states(state)
    end = time.perf_counter()
    print("Elapsed (before compilation) = {}s".format((end - start)))

    start = time.perf_counter()
    possible_states = get_possible_states(state)
    end = time.perf_counter()
    print("Elapsed (after compilation) = {}s".format((end - start)))

    print("Number of states: " + str(possible_states.shape[0]))

    possible_states_performance(1000)
