import numpy as np
import math
import time

timeDependent = False

if timeDependent == False:

    # Using the most recent reward function available.

    num_states = 1000
    num_actions = 5

    # Learning rate. In [0,1]. The higher the learning rate, the faster learning occurs
    alpha = 0.1

    # Discount factor. In [0,1]. The higher the discount factor, the more important future rewards are
    gamma = 0.9

    current_state_sample = 0
    q_matrix_sample = np.zeros((num_states, num_actions))
    end_state_sample = 1000
    current_action_sample = np.random.choice(num_actions)
    occupancy_grid_sample = np.random.random((num_states, num_actions))

    print(occupancy_grid_sample)

    # Updates reward function after every action
    def reward_function(current_state, end_state, action, occupancy_grid):

        reward = -(occupancy_grid[current_state][action])

        return reward

    # Temporal Difference learning
    def update_matrix(q_matrix, current_state, end_state, action, learning_rate, discount_factor, occupancy_grid):

        reward = reward_function(current_state, end_state, action, occupancy_grid)

        q_matrix[current_state][action] = q_matrix[current_state][action] + learning_rate * \
        (reward + discount_factor * np.max(q_matrix[current_state]) \
        - q_matrix[current_state][action])
        
        return q_matrix

    # Loop through the iterations
    def iterate(q_matrix, current_state, end_state, action, learning_rate, discount_factor, occupancy_grid, iterations):
        for i in range(iterations):
            update_matrix(q_matrix, current_state, end_state, action, learning_rate, discount_factor, occupancy_grid)
            current_state += 1
    
        return q_matrix

    # Reset our current q-matrix using the new data
    def reset_q_matrix(q_matrix, current_state, end_state, action, learning_rate, discount_factor, occupancy_grid, iterations):
        q_matrix = [[0 for i in range(len(q_matrix[0]))] for j in range(len(q_matrix))]
        iterate(q_matrix, current_state, end_state, action, learning_rate, discount_factor, occupancy_grid, iterations)


    # Sample q-matrix update with 1000 iterations
    iterations = 1000
    iterate(q_matrix_sample, current_state_sample, end_state_sample, 
            current_action_sample, alpha,gamma, occupancy_grid_sample, iterations)

    print(q_matrix_sample)
    
    reset_q_matrix(q_matrix_sample, current_state_sample, end_state_sample, 
            current_action_sample, alpha,gamma, occupancy_grid_sample, iterations)



"""
else:

    # Explicitly estimate the reward function in the future using the time dimension
    # that has been added to the state

    num_states = 20
    num_actions = 5
    q_matrix_sample = np.zeros((num_states, num_actions))

    # Learning rate. In [0,1]. The higher the learning rate, the faster learning occurs
    alpha = 0.1

    # Discount factor. In [0,1]. The higher the discount factor, the more important future rewards are
    gamma = 0.9

    future_time = time.time() + 60
    currentState = 0
    data = np.array([currentState, future_time])

    q_matrix_sample = np.zeros((num_states, num_actions))
    end_state_sample = np.random.rand(num_states, num_actions)
    currentAction = np.random.choice(num_actions)

    def reward_function(current_state, end_state, action):
        current_state = end_state
        

    # Temporal Difference learning
    def update_matrix(q_matrix, current_state, end_state, action, learning_rate, discount_factor):

        reward = reward_function(current_state, end_state, action)

        q_matrix[current_state][action] = q_matrix[current_state][action]
        + learning_rate * (reward + discount_factor * np.max(q_matrix[current_state, :]) - 
        q_matrix[current_state][action])

        return q_matrix


    # Estimate the reward function for future states (time >) before we have more information.
    # Make sure to not loop around

"""