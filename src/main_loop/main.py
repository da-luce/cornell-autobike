from pykalman import KalmanFilter
import numpy as np

def main():
    # Initialize state
    current_state = [0, 0, 0, 0, 0, 0, 0]

    while(True):
        # Receiving data from GPS
        gps = get_gps()

        # Receiving data from vision
        vision = get_vision()

        # Update state given GPS data
        current_state = update_state(gps)

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
    return

if __name__ == "__main__":
    main()