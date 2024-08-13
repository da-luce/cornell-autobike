"""Module which runs the navigation algorithm as a whole."""

# import numpy as np
# from pykalman import KalmanFilter


def main():
    """
    Main loop
    """

    # Initialize state
    current_state = [0, 0, 0, 0, 0, 0, 0]

    while True:
        # Receiving data from GPS
        gps = get_gps()

        # Receiving data from vision
        vision = get_vision()
        print(vision)

        # Update state given GPS data
        current_state = update_state(current_state, gps)

        # Find optimal steering angle
        steering_angle = get_steering_angle(current_state)

        # Send data to controls
        output_data(steering_angle)

        # Put an end state

        # if end_state is reached:
        #     break


def calculate_moving_average(state, data):
    """
    take average of current state and gps data
    """
    print(state, data)
    return 0


def get_gps():
    """
    Receives data from GPS
    """
    return 0


def get_vision():
    """
    Receives data from vision
    """
    return 0


def update_state(state, gps):
    """
    Updates state given GPS data
    """
    print(state, gps)
    return state


def get_steering_angle(state):
    """
    Returns optimal steering angle
    """
    print(state)
    steering_angle = 0.0

    return steering_angle


def output_data(steering_angle):
    """
    Sends steering angle data to controls
    """
    print(steering_angle)
    return 0


if __name__ == "__main__":
    main()
