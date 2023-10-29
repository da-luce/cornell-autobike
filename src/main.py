from state_pred.sim import get_possible_states, optimize_input_res
import numpy as np

# Example state
state = np.array([0, 0, 2, 1, np.radians(10), np.radians(10)])

# Example differentials
differentials = np.array([0.001, 0.001, 0.001, 0.001,
                          np.radians(0.0001), np.radians(0.0001)])

# Example input resolutions
res = optimize_input_res()

# Get array of all possible states achievable in DT from current state
possible_states = get_possible_states(state, differentials, res)
