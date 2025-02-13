import rosbag
import os
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt 
bag = rosbag.Bag('./packages/my_package/tick.bag')
left = []
right = []
_vehicle_name = os.environ['VEHICLE_NAME']   
radius = rospy.get_param(f'/{_vehicle_name}/kinematics_node/radius', 100)
DISTANCE_PER_TICK = np.pi*2*radius/135
for topic, msg, t in bag.read_messages(topics=[f"/{_vehicle_name}/left_wheel_encoder_node/tick"]):
    left.append(msg.data)

for topic, msg, t in bag.read_messages(topics=[f"/{_vehicle_name}/right_wheel_encoder_node/tick"]):
    right.append(msg.data)
bag.close()

ctl = 0
ctr = 0
ptl = left[0]
ptr = right[0]
ntheta = 0
xt = 0
yt = 0
xs = []
ys = []

print(left)
print(right)

for i in range(1, min(len(left), len(right))):
    ctl = left[i]
    ctr = left[i]

    dl = (ctl - ptl) * DISTANCE_PER_TICK
    dr = (ctr - ptr) * DISTANCE_PER_TICK

    d = (dl+dr)/2
    thetha = (dl-dr)/0.16
    
    xt += d*math.cos(ntheta + thetha/2)
    yt += d*math.sin(ntheta + thetha/2)
    ntheta += thetha

    xs.append(xt)
    ys.append(yt)

    ptl = left[i]
    ptr = right[i]


# plt.plot(xs,ys)
# plt.show()

# print(xs)
# print(ys)






