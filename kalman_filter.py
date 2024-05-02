from filterpy.kalman import KalmanFilter
import numpy as np
import pandas as pd

dim_x = 6 #size of the state vector (position, velocity, angle)
dim_z = 2 #size of the measurement vector (longitude, latitude)
t = 0 #time step TO DO: change this constant

filter = KalmanFilter(dim_x=dim_x, dim_z=dim_z)

#setting filter variables
filter.x = np.zeros((dim_x, 1))
filter.P *= 1 # TO DO: change constant
filter.R *= 1 #measurement noise matrix TO DO: change constant depending on data from GPS
filter.Q *= 1 #process noise matrix TO DO: change constant
filter.F = np.array([[1, 0, t, 0, 0, 0],
                     [0, 1, 0, t, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 1]])
filter.H *= 1 #measurement function TO DO: change measurement function

#predict/update loop
while True:
    #updating state estimate and state covariance matrix
    filter.predict()
    filter.update()

    state_prediction = filter.x #updated state
    prediction_error = filter.P #current state covariance matrix (error)
    # TO DO: do something with the new state_estimate

    condition = False
    if condition:
        break

    #output: predicted state, error on that prediction, iteratively update state & error
    # TO DO: figure out constants 