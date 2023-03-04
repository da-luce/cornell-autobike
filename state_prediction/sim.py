import time
import numpy as np
from numba import jit
from visual import *
from constants import *

""""
A note on @jit decorators:
    - "signature string":   Defines the return type and parameter types for compilation.
                            Not required with just-in-time compilation, but is necessaryfor
                            for ahead-of-time compilation if used in future
    - nopython=True:        Forces jit to compile to pure byte code (significantly faster)
    - cache=True:           Caches compiled functions in a file to reduce overhead on first 
                            time runs

Read more: https://numba.pydata.org/numba-doc/latest/reference/types.html
"""

@jit("float64[::1](float64[::1], float64, float64)",
     nopython=True,
     cache=False)
def next_state_nonlinear(state, throttle, steering):

    """
    Nonlinear (dynamic) model of bicycle. Return future state given inputs. Physical model of bike.

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


@jit("boolean(float64[::1])",
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


@jit("float64[:,::1](float64[::1])",
     nopython=True,
     cache=False)
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
                next = next_state_nonlinear(state, acc, steer)
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

    mean = round(np.mean(runs), 5)
    std = round(np.std(runs), 5)

    print(f"PERFORMANCE\n----------\ntrials: {iter}\nmean: {mean}s\nSTD: {std}s\n")
    return


# Run jit functions to compile
def setup():
    print("Comiling functions...")
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])
    get_possible_states(state)
    print("Compilation complete")

# Testing
if __name__ == "__main__":
    
    # Compile functions
    setup()

    # Example state
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

    # Get array of all possible states achievable in DT from current state
    possible_states = get_possible_states(state)

    # Number of states
    print("Calculated " + str(possible_states.shape[0]) + " possible states\n")

    # Performance check
    possible_states_performance(50000)

    # Visualize states
    plot_states(possible_states)
    plot_bike(state)
    show_plot()
