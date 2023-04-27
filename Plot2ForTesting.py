import matplotlib.pyplot as plt
import HardToWriteFunctions as func

#x,y,speed,angle

currState = [0,0,1,90]
step1 = 1
diff1 = func.getStateMatrix()
testingstates = func.getPlayableActions(currState, diff1[1], step1)

x =[]

y = []

speed = []

angle = []


for arr in testingstates:
    x.append(arr[0])
    y.append(arr[1])
    speed.append(arr[2])
    angle.append(arr[3])

#plt.scatter(x, y, c ="blue", label="y vs x", sizes=[20]) 
#plt.scatter(x, speed, c="green", label="speed vs x", sizes=[15]) 
plt.scatter(x, angle, c="orange", label="angle vs x", sizes=[0.5]) 
 
# To show the plot
plt.legend()
plt.show()

