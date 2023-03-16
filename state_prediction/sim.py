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
import visual as vis
import constants as cst



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

    yaw = yaw_angle * steer_angle * cst.DT
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi # Normalize yaw angle

    lat_force_front = -cst.CORNERING_STIFF_FRONT * np.arctan(((vel_y + cst.DIST_FRONT_AXEL * steer_angle) / vel_x - steering))
    lat_force_rear = -cst.CORNERING_STIFF_REAR * np.arctan((vel_y - cst.DIST_REAR_AXEL * steer_angle) / vel_x)

    # Aerodynamic and friction coefficients
    R_x = 0.01 * vel_x
    F_aero = 1.36 * vel_x ** 2
    F_load = F_aero + R_x

    vel_x = vel_x + (throttle - lat_force_front * np.sin(steering) / cst.MASS - F_load / cst.MASS + vel_y * steering) * cst.DT
    vel_y = vel_y + (lat_force_rear / cst.MASS + lat_force_front * np.cos(steering) / cst.MASS - vel_x * steering) * cst.DT

    steering = steering + (lat_force_front * cst.DIST_FRONT_AXEL * np.cos(steering) - lat_force_rear * cst.DIST_REAR_AXEL) / cst.YAW_INERTIA * cst.DT

    # Advect bike
    x = x + vel_x * np.cos(yaw_angle) * cst.DT - vel_y * np.sin(yaw_angle) * cst.DT
    y = y + vel_x * np.sin(yaw_angle) * cst.DT + vel_y * np.cos(yaw_angle) * cst.DT

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

    return (cst.SPEED_MIN <= speed<= cst.SPEED_MAX and
            -cst.STEER_ANGLE_MAX <= steer_angle <= cst.STEER_ANGLE_MAX)



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



@jit("float64[::1](float64[::1], float64[::1])",
     nopython=True,
     cache=False)
def round_state(state, differentials):

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
        rounded_state[i] = round_to_multiple(state[i], differentials[i])

    return rounded_state



@jit("float64[:,::1](float64[::1], float64[::1], float64, float64)",
     nopython=True,
     cache=False)
def get_possible_states(state, differentials, throttle_res, steering_res):

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

    possible_acc = np.arange(cst.ACCELERATION_MIN, cst.ACCELERATION_MAX, throttle_res)
    possible_steer = np.arange(-cst.STEER_ANGLE_DELTA, cst.STEER_ANGLE_DELTA, steering_res)

    # Allocate an array to store possible states indicies
    max_size = possible_acc.size * possible_steer.size
    possible_states = np.empty((max_size, 6), float)

    index = 0

    for acc in possible_acc:
        for steer in possible_steer:

                next = next_state_nonlinear(state, acc, steer)

                # TODO: check for duplicate arrays
                if (valid_state(next)):

                    # Round the state to fit within the state matrix
                    next = round_state(next, differentials)

                    possible_states[index] = (next)
                    index += 1

    # Only return valid states
    return possible_states[1:index]



@jit("float64[:,::1](float64[:,::1], float64[::1])",
     nopython=True,
     cache=False)
def get_possible_indicies(states, differentials):

    """Given an array of states, return the corresponding indicies within the state matrix

    Args:
        states:
            A two dimensional array of states
        differentials:
            One dimensional array representing differentials

    Return:
        Corresponding indicides of states within state matrix

    Usage:
        Used in conjunction with `get_possible_states()`

        Example usage:

        >>> current_state, differentials = ...
        >>> states = get_possible_states(current_state, differentials, ...)
        >>> indicies = get_possible_indicies(states, differentials)
    """

    return states / differentials



def possible_states_performance(iter, differentials):

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
        get_possible_states(test_state, differentials, 0.1, 0.1)
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
    differentials = np.array([1.0,1.0,1.0,1.0,1.0,1.0])
    get_possible_states(state, differentials, 0.5, 0.5)
    print("Compilation complete")



def performance(throttle_res, steering_res):

    throttle_size = np.arange(cst.ACCELERATION_MIN, cst.ACCELERATION_MAX, throttle_res).size
    steer_size = np.arange(-cst.STEER_ANGLE_DELTA, cst.STEER_ANGLE_DELTA, steering_res).size
    return throttle_size * steer_size



def optimize_input_matrix(res, differentials):

    """
    Return
    """

    state = np.array([0, 0, 2, 0, np.radians(0), np.radians(0)])

    throttle_range = np.linspace(0.01, 0.05, res)
    steering_range = np.flip(np.linspace(np.radians(0.01), np.radians(0.05), res))

    input_matrix = np.empty((res * res, 4))

    i = 0
    for throttle in throttle_range:
        for steering in steering_range:
            performance_m = performance(throttle, steering)
            possible_states = np.unique(get_possible_states(state, differentials, throttle, steering)).shape[0]
            input_matrix[i] = np.array([throttle, steering, possible_states, performance_m])
            i += 1

    percentile = 95
    index = np.int_((res * res) * (percentile / 100))
    # Sort by number of possible states
    """
    ind = np.argsort(input_matrix[:,2])
    input_matrix = input_matrix[ind]
    """

    print(input_matrix[:,0])
    #input_matrix = input_matrix[:-10]

    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    x = input_matrix[:,0]
    y = input_matrix[:,1]
    z = input_matrix[:,2]
    c = input_matrix[:,3]


    ax.scatter(x, y, z, c = c, cmap='hot', norm = "log")
    plt.show()



# Testing
if __name__ == "__main__":

    # Compile functions
    setup()

    # Example state
    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

    # Example differentials
    differentials = np.array([0.001, 0.001, 0.001, 0.001, np.radians(0.0001), np.radians(0.0001)])

    # Get array of all possible states achievable in DT from current state
    possible_states = get_possible_states(state, differentials, 0.03, 0.6)

    # Number of states
    print("Calculated " + str(possible_states.shape[0]) + " possible states\n")

    # Performance check
    possible_states_performance(10000, differentials)

    # Print optimized states
    optimize_input_matrix(100, differentials)

    # Visualize states
    vis.plot_states(possible_states)
    vis.plot_bike(state)
    vis.show_plot()
