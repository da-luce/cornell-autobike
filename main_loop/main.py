from pykalman import KalmanFilter
import numpy as np
import qlearning/bikeQAgent.py
import qlearning/qagent.py
import main_loop/check_waypoint.py
from smbus2 import SMBus

def main():
    # Initialize state
    current_state = [0, 0, 0, 0, 0, 0, 0]

    while(True):
        #get the waypoints somehow...
        curr_waypoint = []
        next_waypoint = []
        
        # Receiving data from GPS
        gps = get_gps()

        # Receiving data from vision
        vision = get_vision()

        #initialize qagent
        alpha = 0
        gamma = 0

	curr = np.array((curr_state[0], curr_state[1]))
        next_pt = np.array((next_waypoint[0], next_waypoint[1]))
        next_dist = np.linalg.norm(curr - next_pt)
        rewards = get_rewards(vision, next_dist)
        
        dt = 0
        qagent = BikeQAgent(alpha, gamma, rewards, dt)
        
        # Update state given GPS data
        current_state = update_state(gps)

        #check waypoint, return original or new qagent
        qagent = check_waypoint(current_state, curr_waypoint, rewards, qagent)

        #run qlearning
        iterations = 1000
        scale = 0.9
        qagent.alter_matrix(rewards, iterations, next_waypoint, scale)
        # Find optimal steering angle
        steering_angle = get_steering_angle(current_state)

        # Send data to controls
        output_data(steering_angle)

        # Put an end state

        # if end_state is reached:
        #     break

def calculate_moving_average(state, data):
    # take average of current state and gps data

    return

# Receives data from GPS
def get_gps():
    return

# Receives data from vision
def get_vision():
    return

# Updates state given GPS data
def update_state(state, gps):
    return state

# Returns optimal steering angle
def get_steering_angle(state):
    steering_angle = 0.0

    return steering_angle

# Sends steering angle data to controls
def output_data(steering_angle):   
    address = 80 #to change
    bus_number = 0 #to change
    with SMBus(bus_number) as bus:
        bus.write_i2c_block_data(address, 0, steering_angle)

if __name__ == "__main__":
    main()
