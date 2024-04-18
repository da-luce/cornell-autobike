
import numpy as np
import qlearning/bikeQAgent.py
import qlearning/qagent.py


#check how the data is stored as
def check_waypoint(curr_state, curr_waypoint, rewards, old_qagent):
	x_s = curr_state[0]
	y_s = curr_state[1]
	x_w = curr_waypoint[0]
	y_w = curr_waypoint[1]
	
	point1 = np.array((x_s, y_s))
        point2 = np.array((x_w, y_w))

        dist = np.linalg.norm(point1 - point2)
        
	tolerance = 0.5 #to change
	alpha = 0 #to change
	gamma = 0 #to change
	dt = 0 # to change
	

	if (dist <= tolerance):
                qAgent = BikeQAgent(alpha, gamma, rewards, dt)
                return qAgent
        else:
                return old_qagent
