import numpy as np
import math 


def getPlayableActions(currentState, differentials, timestep):
    """Returns a list of states reachable from [currentState] after time [timestep]
    has elapsed. [currentState] is a list of 4 numbers: x coordinate, y coordinate, 
    speed, and angle. [differentials] is also 4 numbers, but is the 
    differences between cells in the matrix in SI units. [timestep] is what states are 
    possible after [timestep] amount of time."""
    acceleration_power = 1  # m/s/s
    braking_power = 1  # m/s/s
    max_turning_rate = 30  # deg/s 

    #calculating max velocity reachable
    max_vel = currentState[2] + acceleration_power*(timestep)

    #calculating min velocity reachable
    min_vel = currentState[2] - braking_power*(timestep) 

    #calculating max clockwise angle reachable
    clock_max_angle = (currentState[3]-(max_turning_rate*timestep))%360

    #calculating max counter-clockwise angle reachable
    counter_max_angle = (currentState[3]+(max_turning_rate*timestep))%360

    #min_x_coordinate
    init_x_coord = currentState[0]

    #min_y_coordinate
    init_y_coord = currentState[1]

    #calculating max displacement possible if continuing on same path with max acceleration
    max_S = currentState[2]*timestep + 0.5*acceleration_power*timestep*timestep

    #calculating max change in x coordinate
    max_x_coord = init_x_coord + (max_S * math.cos(currentState[3]))

    #calculating max change in y coordinate
    max_y_coord = init_y_coord + (max_S * math.sin(currentState[3]))

    angle_diff_check = counter_max_angle - (clock_max_angle-360) 

    x1 = clock_max_angle/differentials[3]

    while angle_diff_check>0:
        if angle_diff_check>counter_max_angle:
            x1-=1
            return [[[(x, y, z, x1) for x in range(init_x_coord/differentials[0], max_x_coord/differentials[0])] 
            for y in range(init_y_coord/differentials[1], max_y_coord/differentials[1])]
            for z in range(min_vel/differentials[2], max_vel/differentials[2])]
        else:
            x1+=1
            return [[[(x, y, z, x1) for x in range(init_x_coord/differentials[0], max_x_coord/differentials[0])] 
            for y in range(init_y_coord/differentials[1], max_y_coord/differentials[1])]
            for z in range(min_vel/differentials[2], max_vel/differentials[2])]

        angle_diff_check-=1

        




def getStateMatrix():
    """Returns a tuple, the first element is a matrix with dimensions: x coordinate, 
    y coordinate, speed, angle. The second element is the differences between
    each element of the matrix in SI units. This function should be determined before
    compile-time based on the occupancy grid resolution and other physical factors."""
    return np.zeros((1000, 1000, 10000, 10000, 360))
