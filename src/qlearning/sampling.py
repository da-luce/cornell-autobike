from qAgent import QAgent
import numpy as np
import math
import matplotlib.pyplot as plt
from state_prediction import get_possible_states

acceleration_power = 1  # m/s/s
braking_power = 1  # m/s/s
max_turning_rate = 30  # deg/s

#acc_res, turning_res, density of states that combo results in
acc_res = 0
turning_res = 0
acc_ult = 0
turning_ult = 0

res = [acc_res, turning_res]
high_res = [acc_ult, turning_ult]

#use diff values of res to check the output
temp = get_possible_states(curr, diff, res)

#use very high res to get close to 100% of states to be used as the benchmark 
#for checking sampling density
total_samples = get_possible_states(curr, diff, high_res)

sample_size = len(temp)
total_samples_size = len(total_samples)

sample_density = (sample_size/total_samples_size)*100

plt.figure()
ax = plt.axes(projection="3d")
ax.scatter(acc_res, turning_res, sample_density)

#plt.plot(acc_res, turning_res, sample_density)

ax.set_xlabel("acceleration res")
ax.set_ylabel("turning res")
ax.set_zlabel("sampling density")
ax.view_init(60,35)
plt.show()

