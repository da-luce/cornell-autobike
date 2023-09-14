import numpy as np


def getPlayableActions(currentState, differentials, timestep):
    playableActions = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0],
                                [1, 0, 1, 0, 1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                [0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 1, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1, 0, 1, 0, 1],
                                [0, 0, 0, 0, 0, 0, 0, 1, 0]])

    return np.nonzero(playableActions[currentState])


def getStateMatrix():
    return np.zeros((9,)), (1,)
