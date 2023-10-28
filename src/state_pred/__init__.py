"""This package provides functionality for determining possible states that the
bike may achieve within a time step.

Files and important functions:

    - constants.py: physical and simulation constants & constraints, including
                    differentials
    - sim.py:       contains methods for computing future states

        - get_possible_states()

    - visual.py:    provides utilities for visualizing possible states & bike
                    state in matplotlib

        - plot_states()
        - plot_bike()
        - show_plot()

States:

    States are reprsented as a np.array with 6 elements:
    np.array([x_position  (m), 
              y_position  (m),
              x_velocity  (m/s),
              y_velocity  (m/s),
              yaw_angle   (rad),
              steer_angle (rad)])

    Helpful video for understanding vehicle state: 
    https://www.youtube.com/watch?v=35lZlO6NrO0

Importing:

    # If only computing possible states
    from state_prediction import get_possible_states

    # If visualizing
    from state_prediction import get_possible_states, plot_states. plot_bike,
                                 show_plot
"""
import sys

sys.path.append("/usr/app/src/state_pred")
import constants
import sim
import visual
import time

# Since we are using Numba, we must run all functions for a first time in order
# to compile them. We can do this by running a "fake example"
print("Compiling functions...")
start = time.perf_counter()
sim.setup
end = time.perf_counter()
print(f"Compilation completed in {end - start}s")
