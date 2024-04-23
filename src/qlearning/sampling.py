from src.state_pred.bike_sim import get_possible_states, optimize_input_res
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

acceleration_power = 1  # m/s^2
braking_power = 1  # -m/s^2
max_turning_rate = 0.5181  # rad/s

#potential range of ACC-RES: 0.01 to 5.00
#potential range of TURNING-RES: 0.01 to 100.0

#plot in 2d matrix with arrays of acc and turning
#acc_res, turning_res, density of states that combo results in

# init_acc_res = 0.01
# init_turning_res = 0.1

acc_ult = 0.01
turning_ult = 0.01

#states are reprsented as a np.array with 6 elements:  
    #np.array([x_position  (m), 
              #y_position  (m),
              #x_velocity  (m/s),
              #y_velocity  (m/s),
              #yaw_angle   (rad),
              #steer_angle (rad)])

curr1 = np.array([10, 10, 5, 5, 2.1, 0.67])

differentials = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

high_res = np.array([acc_ult, turning_ult])

#use very high res to get close to 100% of states to be used as the benchmark 
#for checking sampling density
total_samples = get_possible_states(curr1, differentials, high_res)

#sample_size = len(temp)
total_samples_size = len(total_samples)
#print(total_samples_size)

acc_res_size = 160
turn_res_size = 160

densities = np.zeros((acc_res_size, turn_res_size))
acc_res = np.zeros(acc_res_size)
turn_res = np.zeros(turn_res_size)

vari=0.01
varj=0.01
for i in range(turn_res_size): #primary nested loop responsible for density computation
  for j in range(acc_res_size):
    #print(vari)
    #print(varj)
    acc_res[j] = varj
    turn_res[i] = vari

    temp = (get_possible_states(curr1, differentials, (np.array([varj, vari]))))
    sample_size = len(temp)
    sample_density = (sample_size/total_samples_size)*100
    #print(sample_density)
    densities[i, j] = sample_density

    varj+=0.01
  varj=0.01
  vari+=0.01

#print(densities)
#print(densities.shape)

#print(len(acc_res))
# print(len(turn_res))
# final_len = len(densities)


# Assuming acc_res and turn_res are 1D arrays and densities is a 2D array

# Create a meshgrid for acc_res and turn_res
# acc_res_grid, turn_res_grid = np.meshgrid(np.log(acc_res), np.log(turn_res))
acc_res_grid, turn_res_grid = np.meshgrid((acc_res), (turn_res))

densities = np.where(densities > 1e-10, densities, -10)
np.log(densities, out=densities, where=densities > 0)


#alternate way to transform densities into log given below; seems to have some errors in computation

# safe = np.clip(densities, 1e-10, None)
# densities = np.where(densities > 1e-10, np.log(safe), 0)
    
# Example usage

# Creating a figure for 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Surface plot
surf = ax.plot_surface(acc_res_grid, turn_res_grid, densities, cmap='viridis')

# Labels and title
ax.set_xlabel('Acceleration Res')
ax.set_ylabel('Turning Res')
ax.set_zlabel('Sampling Density')

# View angle
ax.view_init(60, 35)

# Color bar
fig.colorbar(surf, shrink=0.5, aspect=5)

# Save the plot
plt.savefig('sampling_density_surface_plot.png')




# Creating a figure
fig, ax = plt.subplots()

# Contour plot
contour = ax.contourf(acc_res_grid, turn_res_grid, densities, cmap='viridis')

# Labels
ax.set_xlabel('Acceleration Res')
ax.set_ylabel('Turning Res')

# Color bar
fig.colorbar(contour)

# Save the plot
plt.savefig('sampling_density_contour_plot.png')





# plt.figure()
# ax = plt.axes(projection="3d")

# ax.scatter(acc_res, turn_res, densities)
# plt.plot(acc_res, turn_res, densities)
# ax.set_xlabel("acceleration res")
# ax.set_ylabel("turning res")
# ax.set_zlabel("sampling density")
# ax.view_init(60,35)
# plt.savefig('sampling_plot5.png')

# plt.figure()
# ax = plt.axes(projection="3d")
# ax.scatter(acc_res, turn_res, densities)

# plt.plot(acc_res, turn_res, densities)

# ax.set_xlabel("acceleration res")
# ax.set_ylabel("turning res")
# ax.set_zlabel("sampling density")
# ax.view_init(60,35)

# plt.savefig('sampling_plot5.png')


# plt.show()







# acc_res = []
# turn_res = []

# pos_i = 0
# pos_j = 0

# i = 0.1
# while (i<=100):
#   if i>0.1:
#     pos_i+=1
#   j=0.01
#   while (j<=10):
#     if j>0.01:
#       pos_j+=1
#     acc_res.append(j)
#     turn_res.append(i)
#     temp = (get_possible_states(curr1, differentials, (np.array([j, i]))))
#     sample_size = len(temp)
#     #print(sample_size)
#     sample_density = (sample_size/total_samples_size)*100
#     #print(sample_density)
#     densities[pos_i, pos_j] = sample_density
#     #densities.append(sample_density)
#     j+=0.01

#   i+=0.1


# final_densities = np.array(densities).reshape(len(acc_res), -1)
# print(final_densities.shape)