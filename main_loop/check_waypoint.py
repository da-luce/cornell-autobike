
import numpy as np
import qlearning/bikeQAgent.py
import qlearning/qagent.py


#check how the data is stored as
def check_waypoint(curr_state, vision, curr_waypoint, next_waypoint):
	x_s = curr_state[0]
	y_s = curr_state[1]
	x_w = curr_waypoint[0]
	y_w = curr_waypoint[1]
	x_n = next_waypoint[0]
	y_n = next_waypoint[1]
	
	point1 = np.array((x_s, y_s))
        point2 = np.array((x_w, y_w))
        point3 = np.array((x_n, y_n))
        dist = np.linalg.norm(point1 - point2)
        
	tolerance = 0.5 #to change
	alpha = 0 #to change
	gamma = 0 #to change
	dt = 0 # to change
	

	if (dist <= tolerance):
                next_dist = np.linalg.norm(point1 - point3)
                rewards = get_rewards(vision, next_dist) 
                qAgent = BikeQAgent(alpha, gamma, rewards, dt)
