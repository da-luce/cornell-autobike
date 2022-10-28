def getPlayableActions(currentState, differentials, timestep):
    """Returns a list of states reachable from [currentState] after time [timestep]
    has elapsed. [currentState] is a list of 5 numbers: x coordinate, y coordinate, 
    x velocity, y velocity, and angle. [differentials] is also 5 numbers, but is the 
    differences between cells in the matrix in SI units. [timestep] is what states are 
    possible after [timestep] amount of time."""


def getStateMatrix():
    """Returns a tuple, the first element is a matrix with dimensions: x coordinate, 
    y coordinate, x velocity, y velocity, angle. The second element is the differences between
    each element of the matrix in SI units. This function should be determined before
    compile-time based on the occupancy grid resolution and other physical factors."""
