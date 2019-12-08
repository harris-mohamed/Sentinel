import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import RANSAC
import PARSER
import numpy as np

# Log file locations 
#single_scan = './Logs/Static-sweep0_11-26-19.log'
#single_scan = './Logs/Static-sweep1_11-26-19.log'
single_scan = './Logs/Static-sweep_11-26-19.log'
#single_scan = './Logs/Dynamic-sweep_11-26-19.log'
#single_scan = './Logs/Sweep2_11-22-19.log'
x = np.array([[0.0],[0.0],[0.0]])
dt1 = 0
dt2 = 0 #In reality, these would be grabbed from the Arduino.

res = PARSER.parser(single_scan)

# These are only included so that RANSAC can run its sampling algorithm
#angle_increment = np.radians(res[index]['Angular Increment']) #for when the angle increment value is returned correctly by the parser
angle_increment = np.radians(270/811)
message_count = res[-1]['Message Count']
start_angle = np.radians(res[-1]['Start Angle'])
end_angle = start_angle + (message_count-1)*angle_increment
x = RANSAC.UpdatePosition(x, dt1, dt2)
scan = RANSAC.ConvertToCartesian(res, x)
#for plotting the points later
xs = []
ys = []
zs = []
for point in scan.values():
    xs.append(point[1])
    ys.append(point[2])
    zs.append(point[3])

start = time.time()
(Landmark_LSRPS, LSRP_list) = RANSAC.RANSAC(scan, start_angle, end_angle)
end = time.time()
print("RANSAC took "+ str(end-start) + " seconds to process "+ str(message_count) +" points ")

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(xs, ys, zs, s=3, marker='x', color='r')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
RANSAC.plotLSRPs(ax, LSRP_list, ymax=7000)
ax.view_init(0, 0)
plt.show()
