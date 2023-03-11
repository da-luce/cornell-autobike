"""This model provides functionality for computing possible states/playable actions

This module uses numba in order to compile computation heavy tasks just in time:

A note on @jit decorators:
    - "signature string":   Defines the return type and parameter types for compilation.
                            Not required with just-in-time compilation, but is necessaryfor
                            for ahead-of-time compilation if used in future
    - nopython=True:        Forces jit to compile to pure byte code (significantly faster)
    - cache=True:           Caches compiled functions in a file to reduce overhead on first 
                            time runs (do not set True for functions that rely on outside variables)

Read more: https://numba.pydata.org/numba-doc/latest/reference/types.html
"""

import time
import numpy as np
from numba import jit
from visual import *
from constants import *



@jit("float64[::1](float64[::1], float64, float64)",
     nopython=True,
     cache=False)
def next_state_nonlinear(state, throttle, steering):

    """Nonlinear (dynamic) model of bicycle

    Return next state of bike given a current state and inputs.

    Args:
        state: 
            Six element np.array representing current state
        throttle:
            TODO Not sure about this one
        steering:
            Delta in steering angle in radians

    Returns:
        A six element np.array representing the next/update state
    """

    # Unpack state into vairables
    x, y, vel_x, vel_y, yaw_angle, steer_angle = state

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

    """Whether a state is valid

    Args:
        state:
            A six element np.array representing a bike state

    Returns:
        A boolean representing the validity of the state.
        True if the state is valid, False is the state is invalid
        or impossible
    """

    vel_x, vel_y = state[2], state[3]
    steer_angle = state[5]

    speed = np.sqrt(vel_x ** 2 + vel_y ** 2)

    return (SPEED_MIN <= speed<= SPEED_MAX and
            -STEER_ANGLE_MAX <= steer_angle <= STEER_ANGLE_MAX)



@jit("float64(float64, float64)",
     nopython=True,
     cache=True)
def round_to_multiple(number, multiple):

    """Rounds a float to the nearest mulitple of a number

    Args:
        number:
            The number to be rounded
        multiple:
            The multiple that the number is rounded to

    Returns:
        The rounded number. Example:

            round_to_multiple(5.5, 10) = 10
            round_to_multiple(6.3, 15) = 0
    """

    return multiple * round(number / multiple)



@jit("float64[::1](float64[::1])",
     nopython=True,
     cache=False)
def round_state(state):

    """Round a state to "fit" onto the state matrix

    Args:
        state:
            A six element np.array representing a state

    Returns:
        A rounded state that is an element of the state matix
    """

    # Allocate an empty array to store state
    rounded_state = np.empty((6,), float)

    # For each value in the state, round the value to the 
    # resolution defined in DIFFERENTIALS
    for i in np.arange(0,5,1):
        rounded_state[i] = round_to_multiple(state[i], DIFFERENTIALS[i])

    return rounded_state



@jit("float64[:,::1](float64[::1], float64, float64)",
     nopython=True,
     cache=False)
def get_possible_states(state, throttle_res, steering_res):

    """Get all possible states

    Return all possible states achievable within one timestep

    Args:
        state:
            A six element np.array representing the current state
        throttle_res:
            The difference between sequential throttle inputs
        steering_res:
            The difference between sequential steering inputs

    Return:
        A 2D np.array of all valid states achievable within one timestep
        given the input resolutions
    """

    possible_acc = np.arange(ACCELERATION_MIN, ACCELERATION_MAX, throttle_res)
    possible_steer = np.arange(-STEER_ANGLE_DELTA, STEER_ANGLE_DELTA, steering_res)

    # Allocate an array to store possible states
    max_size = possible_acc.size * possible_steer.size
    possible_states = np.empty((max_size, 6), float)

    index = 0

    for acc in possible_acc:
        for steer in possible_steer:

                next = next_state_nonlinear(state, acc, steer)

                # TODO: check for duplicate arrays
                if (valid_state(next)):
                    possible_states[index] = round_state(next)
                    index += 1

    # Only return valid states
    return possible_states[1:index]



def possible_states_performance(iter):

    """Print mean runtime and standard deviation of get_possible_states()

    Test performance of get_possible_states()

    Args:
        iter:
            The number of iterations of get_possible_states() to perform

    Returns:
        None
    """

    # Allocate array to store results of computations
    runs = np.empty(iter, float)

    # Performance appears to be same regardless of test_state
    # Thus, we use this dummy state as the initial state for all computations
    test_state = np.array([1000, 1000, 3, 2, np.radians(90), np.radians(-20)])

    for i in range(0, iter):

        start = time.perf_counter()
        get_possible_states(test_state, 0.1, 0.1)
        end = time.perf_counter()

        runs[i] = end - start

    # Mean and std of run times
    mean = round(np.mean(runs), 5)
    std = round(np.std(runs), 5)

    print(f"PERFORMANCE\n----------\ntrials: {iter}\nmean: {mean}s\nSTD: {std}s\n")
    return



def setup():

    """ Run and compile jit functions 
    """
    print("Comiling functions...")
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])
    get_possible_states(state, 0.5, 0.5)
    print("Compilation complete")



# TODO: improve this algorithim!
def optimize_possible_resolutions():

    """Optimize input resolutions for get_possible_states

    Find the largest resolution for throttle and steering which achieves
    the desired density of possible states. Currenty this algorithim is very bad.
    I do not like it. First, a "maximum number" of possible states is calculated
    by using a very large resolution. Next, possible combinations of input
    resolutions are calculated. The combination that achieves 95% coverage or more of
    all possible states and has the lowest resolution is choosen. Currenly, this algorithim
    relies on DIFFERENTIALS having a relatively small resolution.

    Args: None

    Returns:
        Two element np.array with optimized input resolutions:
        [throttle_res, steering_res]
    """

    # Set range of possible input resolutions
    throttle_resolutions = np.arange(0.001, 1, 0.001)
    steering_resolutions = np.arange(np.radians(0.001), np.radians(1), np.radians(0.001))

    # Create an array of all possible input combinations
    inputs = np.stack(np.meshgrid(throttle_resolutions, steering_resolutions), -1).reshape(-1, 2)

    # Add two empty column to inputs to store results
    column = np.empty((inputs.shape[0],2), dtype=float)

    results = np.hstack((inputs, column))

    # Dummy state
    # In theory, the values of this state do not really matter -``
    # As the differential resolution becomes large enough, the primary factor
    # Determining the number of possible states is the input resolution, and not
    # The value of the dummy state
    state = np.array([0, 0, 2, 0, np.radians(0), np.radians(0)])

    # Compute a theoreitcal max number of states using a ridiculusly fine resolution
    max_states = np.unique(get_possible_states(state, 0.001, np.radians(0.001))).size

    i = 0
    for i in results:
        throttle_res = i[0]
        steering_res = i[1]
        i[2] = np.unique(get_possible_states(state, throttle_res, steering_res)).size

        possible_acc = np.arange(ACCELERATION_MIN, ACCELERATION_MAX, throttle_res)
        possible_steer = np.arange(-STEER_ANGLE_DELTA, STEER_ANGLE_DELTA, steering_res)

        i[3] = possible_acc.size * possible_steer.size

    results = results[results[:,2] > 95/100 * max_states]
    results = results[results[:, 3].argsort()]

    print(f"Throttle: {results[0][0]}")
    print(f"Angle: {np.degrees(results[0][1])}")

    return np.array(results[0][0], results[0][1])



# Testing
if __name__ == "__main__":

    # Compile functions
    setup()

    # Example state
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

    # Get array of all possible states achievable in DT from current state
    possible_states = get_possible_states(state, 0.1, 0.1)

    # Number of states
    print("Calculated " + str(possible_states.shape[0]) + " possible states\n")

    # Performance check
    possible_states_performance(10000)

    # Print optimized states
    optimize_possible_resolutions()

    # Visualize states
    plot_states(possible_states)
    plot_bike(state)
    show_plot()
