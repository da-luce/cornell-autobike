from src.state_pred.bike_sim import get_possible_states, optimize_input_res
import numpy as np
import math
import matplotlib.pyplot as plt

acceleration_power = 1  # m/s/s
braking_power = 1  # m/s/s
max_turning_rate = 30  # deg/s

#potential range of ACC-RES: 0.01 to 5.00
#potential range of TURNING-RES: 0.1 to 100.0

#plot in 2d matrix with arrays of acc and turning
#acc_res, turning_res, density of states that combo results in

# init_acc_res = 0.01
# init_turning_res = 0.1

acc_ult = 0.01
turning_ult = 0.1

#states are reprsented as a np.array with 6 elements:  
    #np.array([x_position  (m), 
              #y_position  (m),
              #x_velocity  (m/s),
              #y_velocity  (m/s),
              #yaw_angle   (rad),
              #steer_angle (rad)])

curr1 = np.array([0, 0, 5, 5, 0.5, 0.5])

differentials = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

high_res = np.array([acc_ult, turning_ult])

#use very high res to get close to 100% of states to be used as the benchmark 
#for checking sampling density
total_samples = get_possible_states(curr1, differentials, high_res)

#sample_size = len(temp)
total_samples_size = len(total_samples)
#print(total_samples_size)

acc_res_size = 1000
turn_res_size = 1000

densities_shape = (acc_res_size, turn_res_size)
#densities = np.empty(densities_shape)
densities = []
#res_shape = (acc_res_size, turn_res_size) #500 rows for acc_res, 1000 columns for turn_res

acc_res = []
turn_res = []

#res = np.empty(res_shape)

pos_i = 0
pos_j = 0
i = 0.1

while (i<=100):
  if i>0.1:
    pos_i+=1
  j=0.01
  while (j<=10):
    if j>0.01:
      pos_j+=1
    acc_res.append(j)
    turn_res.append(i)
    temp = (get_possible_states(curr1, differentials, (np.array([j, i]))))
    sample_size = len(temp)
    #print(sample_size)
    sample_density = (sample_size/total_samples_size)*100
    #print(sample_density)
    #densities[pos_i][pos_j] = sample_density
    densities.append(sample_density)
    j+=0.01

  i+=0.1

final_densities = np.array(densities)
acc_res = np.array(acc_res)
turn_res = np.array(turn_res)

# print(len(acc_res))
# print(len(turn_res))
# final_len = len(final_densities)
# print(final_len)


# densities_pos=0
# for x in res:
#   temp = (get_possible_states(curr1, differentials, x))
#   sample_size = len(temp)
#   densities[densities_pos] = sample-density
#   densities_pos+=1

#for i in final_densities:
  #print(i)

#use diff values of res to check the output
#temp = get_possible_states(curr, diff, res)


#sample_density = (sample_size/total_samples_size)*100

plt.figure()
ax = plt.axes(projection="3d")
ax.scatter(acc_res, turn_res, final_densities)

plt.plot(acc_res, turn_res, final_densities)

ax.set_xlabel("acceleration res")
ax.set_ylabel("turning res")
ax.set_zlabel("sampling density")
ax.view_init(60,35)

plt.savefig('sampling_plot.png')
#plt.show()
