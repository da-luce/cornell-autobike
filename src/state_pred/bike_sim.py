"""This module provides functionality for computing possible
   states/playable actions

This module uses numba in order to compile computation heavy tasks just in time:

A note on @jit decorators:
    - "signature string":   Defines the return type and parameter types for
                            compilation. Not required with just-in-time
                            compilation, but is necessary for for ahead-of-time
                            compilation if used in future
    - nopython=True:        Forces jit to compile to pure byte code
                            (significantly faster)
    - cache=True:           Caches compiled functions in a file to reduce
                            overhead on first  time runs (do not set True for
                            functions that rely on outside variables)

Read more: https://numba.pydata.org/numba-doc/latest/reference/types.html
"""

import time

import numpy as np
from numba import boolean, float64, jit

from src.state_pred import constants as cst
from src.state_pred import visual as vis


# pylint: disable=too-many-locals
@jit(float64[::1](float64[::1], float64[::1]), nopython=True, cache=False)
def next_state_dynamic(state, inputs):
    """Nonlinear (dynamic) model of bicycle

    Return next state of bike given a current state and inputs.

    Args:
        state:
            Six element np.array representing current state
        throttle:
            TODO: Not sure about this one
        steering:
            Delta in steering angle in radians

    Returns:
        A six element np.array representing the next/update state
    """

    # Unpack state into variables

    x, y, vel_x, vel_y, yaw_angle, steer_angle = state
    throttle, steering = inputs

    yaw = yaw_angle * steer_angle * cst.DT
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi  # Normalize yaw angle

    lat_force_front = -cst.CORNERING_STIFF_FRONT * np.arctan(
        (vel_y + cst.DIST_FRONT_AXEL * steer_angle) / (vel_x - steering)
    )

    lat_force_rear = -cst.CORNERING_STIFF_REAR * np.arctan(
        (vel_y - cst.DIST_REAR_AXEL * steer_angle) / vel_x
    )

    # Aerodynamic and friction coefficients
    r_x = 0.01 * vel_x
    f_aero = 1.36 * vel_x**2
    f_load = f_aero + r_x

    vel_x = (
        vel_x
        + (
            throttle
            - lat_force_front * np.sin(steering) / cst.MASS
            - f_load / cst.MASS
            + vel_y * steering
        )
        * cst.DT
    )

    vel_y = (
        vel_y
        + (
            lat_force_rear / cst.MASS
            + lat_force_front * np.cos(steering) / cst.MASS
            - vel_x * steering
        )
        * cst.DT
    )

    steering = (
        steering
        + (
            lat_force_front * cst.DIST_FRONT_AXEL * np.cos(steering)
            - lat_force_rear * cst.DIST_REAR_AXEL
        )
        / cst.YAW_INERTIA
        * cst.DT
    )

    # Advect bike
    x = x + vel_x * np.cos(yaw_angle) * cst.DT - vel_y * np.sin(yaw_angle) * cst.DT
    y = y + vel_x * np.sin(yaw_angle) * cst.DT + vel_y * np.cos(yaw_angle) * cst.DT

    return np.array([x, y, vel_x, vel_y, yaw, steering])


@jit(boolean(float64[::1]), nopython=True, cache=True)
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

    speed = np.sqrt(vel_x**2 + vel_y**2)

    return (
        cst.SPEED_MIN <= speed <= cst.SPEED_MAX
        and -cst.STEER_ANGLE_MAX <= steer_angle <= cst.STEER_ANGLE_MAX
    )


@jit(float64(float64, float64), nopython=True, cache=True)
def round_to_multiple(number, multiple):
    """Rounds a float to the nearest multiple of a number

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


@jit(float64[::1](float64[::1], float64[::1]), nopython=True, cache=False)
def round_state(state, differentials):
    """Round a state to "fit" onto the state matrix

    Args:
        state:
            A six element np.array representing a state

    Returns:
        A rounded state that is an element of the state matrix
    """

    # Allocate an empty array to store state
    rounded_state = np.empty((state.size,), float)

    # For each value in the state, round the value to the
    # resolution defined in DIFFERENTIALS
    for i in np.arange(0, state.size, 1):
        rounded_state[i] = round_to_multiple(state[i], differentials[i])

    return rounded_state


@jit(
    float64[:, ::1](float64[::1], float64[::1], float64[::1]),
    nopython=True,
    cache=False,
)
def get_possible_states(state, differentials, res):
    """Get all possible states

    Return all possible states achievable within one timestep

    Args:
        state:
            A six element np.array representing the current state
        differentials:
            A six element np.array with the state matrix differentials
        res:
            A two element np.array containing input resolutions
            (`determined with optimize_input_res()`)

    Return:
        A 2D np.array of all valid states achievable within one timestep
        given the input resolutions
    """

    throttle_res, steering_res = res

    possible_acc = np.arange(cst.ACCELERATION_MIN, cst.ACCELERATION_MAX, throttle_res)
    possible_steer = np.arange(
        -cst.STEER_ANGLE_DELTA, cst.STEER_ANGLE_DELTA, steering_res
    )

    # Allocate an array to store possible states indices
    max_size = possible_acc.size * possible_steer.size
    possible_states = np.empty((max_size, state.size), float)

    index = 0

    for acc in possible_acc:
        for steer in possible_steer:

            next_state = next_state_dynamic(state, np.array([acc, steer]))

            # TODO: check for duplicate arrays
            if valid_state(next_state):

                # Round the state to fit within the state matrix
                next_state = round_state(next_state, differentials)

                possible_states[index] = next_state
                index += 1

    # Only return valid states
    return possible_states[1:index]


@jit(
    float64[:, ::1](float64[::1], float64[::1], float64[::1]),
    nopython=True,
    cache=False,
)
def get_possible_indices(state, differentials, res):
    """
    Given an array of states, return the corresponding indices within the state matrix

    Args:
        states:
            A two dimensional array of states
        differentials:
            One dimensional array representing differentials
        res:
            A one dimensional array containing input resolutions

    Return:
        Corresponding indices of states within state matrix

    Usage:
        Used in conjunction with `get_possible_states()`

        Example usage (outside of package):

        >>> from state_pred import optimize_input_res, get_possible_indices
        >>> current_state, differentials = ...
        >>> res = sim.optimize_input_res()
        >>> indices = sim.get_possible_indices(current_state, differentials, res)
    """

    return get_possible_states(state, differentials, res) / differentials


def possible_states_performance(iters, differentials):
    """Print mean runtime and standard deviation of get_possible_states()
    on a certain set of input resolutions

    Test performance of get_possible_states()

    Args:
        iter:
            The number of iterations of get_possible_states() to perform

    Returns:
        None
    """

    # Allocate array to store results of computations
    runs = np.empty(iters, dtype=float)

    # Performance appears to be same regardless of test_state
    # Thus, we use this dummy state as the initial state for all computations
    example_state = np.array([1000, 1000, 3, 2, np.radians(90), np.radians(-20)])

    # Use optimized inputs
    res = optimize_input_res()

    for i in range(0, iters):

        start = time.perf_counter()
        get_possible_states(example_state, differentials, res)
        end = time.perf_counter()

        runs[i] = end - start

    # Mean and std of run times
    mean = round(np.mean(runs), 5)
    std = round(np.std(runs), 5)

    print(f"PERFORMANCE\n----------\ntrials: {iters}\nmean: {mean}s\nSTD: {std}s\n")
    return mean


def setup():
    """Run and compile jit functions"""

    state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])
    differentials = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    res = optimize_input_res()
    get_possible_states(state, differentials, res)


def performance(input_res):
    """
    Return the relative time complexity (total number of loops per call to get_possible_states())
    of certain input resolutions
    """

    throttle_res, steering_res = input_res

    throttle_size = np.arange(
        cst.ACCELERATION_MIN, cst.ACCELERATION_MAX, throttle_res
    ).size
    steer_size = np.arange(
        -cst.STEER_ANGLE_DELTA, cst.STEER_ANGLE_DELTA, steering_res
    ).size

    return throttle_size * steer_size


# Return optimized input resolutions
def optimize_input_res():

    # differentials, input_size, target_runtime, target_accuracy

    """
    differentials: Differentials
    max_runtime: maximum mean time for get_possible_states() to run in
    input_size: number of elements in input array
    """

    # TODO: determine how to actually optimize input resolutions
    return np.array([0.03, 0.06])


# Testing
if __name__ == "__main__":

    # Compile functions
    # setup()
    # FIXME: no longer needed as should always be running in __innit__.py

    # Example state
    test_state = np.array([0, 0, 1, -2, np.radians(-10), np.radians(-20)])

    # Example differentials
    test_differentials = np.array(
        [0.001, 0.001, 0.001, 0.001, np.radians(0.001), np.radians(0.001)]
    )

    # Example input resolutions
    test_res = optimize_input_res()

    # Get array of all possible states achievable in DT from current state
    test_possible_states = get_possible_states(test_state, test_differentials, test_res)

    # Number of states
    print("Calculated " + str(test_possible_states.shape[0]) + " possible states\n")

    # Performance check
    possible_states_performance(100, test_differentials)

    # Visualize states
    vis.plot_states(test_possible_states)
    vis.plot_bike(test_state)
    vis.show_plot()
