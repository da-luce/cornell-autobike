def getPlayableActions(currentState, differentials, timestep):
    """Returns a list of states reachable from [currentState] after time [timestep]
    has elapsed. [currentState] is a list of 5 numbers: x coordinate, y coordinate, 
    x velocity, y velocity, and angle. [differentials] is also 5 numbers, but is the 
    differences between cells in the matrix in SI units. [timestep] is what states are 
    possible after [timestep] amount of time."""
