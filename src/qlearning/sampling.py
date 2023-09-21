from qAgent import QAgent
import numpy as np
import math
from state_prediction import get_possible_states

acceleration_power = 1  # m/s/s
braking_power = 1  # m/s/s
max_turning_rate = 30  # deg/s

#acc_res, turning_res, density of states that combo results in

def isReachable(currentState, futureState, timestep):

