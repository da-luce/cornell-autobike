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

    if (differentials[3]!=0): 
        x1 = clock_max_angle/differentials[3]
    else:
        x1 = clock_max_angle

    list_possible_states = []

    #print(init_x_coord/differentials[0])
    #print(max_x_coord/differentials[0])
    #print(init_y_coord/differentials[1])
    #print(max_y_coord/differentials[1])
    #print(min_vel/differentials[2])
    #print(max_vel/differentials[2])
    
    while angle_diff_check>0:
        if angle_diff_check>counter_max_angle:
            x1-=1
            for x in range(int(init_x_coord/differentials[0]+0.5), int(max_x_coord/differentials[0]+0.5)+1):
                #print("x is " + str(x))
                for y in range(int(init_y_coord/differentials[1]+0.5), int(max_y_coord/differentials[1]+0.5)+1):
                    #print("y is " + str(y))
                    for z in range(int(min_vel/differentials[2]+0.5), int(max_vel/differentials[2]+0.5)+1):
                        #print("z is " + str(z))
                        list_possible_states.append([x, y, z, x1])
            
        else:
            x1+=1
            for x in range(int(init_x_coord/differentials[0]+0.5), int(max_x_coord/differentials[0]+0.5)+1):
                #print("x is " + str(x))
                for y in range(int(init_y_coord/differentials[1]+0.5), int(max_y_coord/differentials[1]+0.5)+1):
                    #print("y is " + str(y))
                    for z in range(int(min_vel/differentials[2]+0.5), int(max_vel/differentials[2]+0.5)+1):
                        #print("z is " + str(z))
                        list_possible_states.append([x, y, z, x1])

        angle_diff_check-=1
  
    return list_possible_states


def getStateMatrix():
    """Returns a tuple, the first element is a matrix with dimensions: x coordinate, 
    y coordinate, speed, angle. The second element is the differences between
    each element of the matrix in SI units. This function should be determined before
    compile-time based on the occupancy grid resolution and other physical factors."""
    return (np.zeros((1000, 1000, 10000, 360), dtype='uint8'), np.array([1,1,1,1]))

#sample test cases from Aditya
#first test case
currStateNum1 = [0, 0, 1, 90]
step1 = 1
diff1 = getStateMatrix()
print(getPlayableActions(currStateNum1, diff1[1], step1))

#second test case
#currStateNum2 = [1, 1, 1, 90]
#step2 = 2
#diff2 = getStateMatrix()
#print(getPlayableActions(currStateNum2, diff2[1], step2))

#third test case
#currStateNum3 = [1, -1, 1, 0]
#step3 = 1
#diff3 = getStateMatrix()
#print(getPlayableActions(currStateNum3, diff3[1], step3))

#fourth test case
#currStateNum4 = [2, 6, 2, 0]
#step4 = 5
#diff4 = getStateMatrix()
#print(getPlayableActions(currStateNum4, diff4[1], step4))

#fifth test case
#currStateNum5 = [10, 5, 2, 180]
#step5 = 10
#diff5 = getStateMatrix()
#print(getPlayableActions(currStateNum5, diff5[1], step5))
