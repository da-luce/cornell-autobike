import numpy as np
import math
import time

timeDependent = False

if timeDependent == False:

    # Using the most recent reward function available.

    num_states = 20
    num_actions = 5

    # Learning rate. In [0,1]. The higher the learning rate, the faster learning occurs
    alpha = 0.1

    # Discount factor. In [0,1]. The higher the discount factor, the more important future rewards are
    gamma = 0.9

    currentState = 0

    q_matrix_sample = np.zeros((num_states, num_actions))
    end_state_sample = 1000
    currentAction = np.random.choice(num_actions)

    # Updates reward function after every action
    def reward_function(current_state, end_state, action):

        reward = 0

        # Reward for an optimal action
        if action + current_state < 0:
            reward = -1
        else:
            reward = 1

        # Reward for reaching the end state
        if current_state + action == end_state:
            reward = 999

        return reward
    

    # Temporal Difference learning
    def update_matrix(q_matrix, current_state, end_state, action, learning_rate, discount_factor):

        reward = reward_function(current_state, end_state, action)

        q_matrix[current_state][action] = q_matrix[current_state][action] + learning_rate * \
        (reward + discount_factor * np.max(q_matrix[current_state]) \
        - q_matrix[current_state][action])
        
        return q_matrix

    # Loop through the iterations
    iterations = 1000
    for i in range(iterations):
        update_matrix(q_matrix_sample, currentState, end_state_sample, currentAction, alpha,
                      gamma)

        currentState += 1

    print(currentState)

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