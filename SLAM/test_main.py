#Created by: Nicholas O'Brien
#Project Sentinel; main script
#Created: December 7th, 2019
#Last Edit: December 8th, 2019
import sys

sys.path.append("..\\")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import RANSAC.RANSAC as RANSAC
import PARSER.PARSER as PARSER
import EKF.EKF as EKF
import numpy as np

# Log file locations 
#single_scan = '../../../Logs/Static-sweep0_11-26-19.log'
#single_scan = '../../../Logs/Static-sweep1_11-26-19.log'
single_scan = '../../../Logs/Static-sweep_11-26-19.log'
#single_scan = '../../../Logs/dynamic_12-26-19_2252.log'
#single_scan = '../../../Logs/Dynamic-sweep_11-26-19.log'
#single_scan = '../../../Logs/Sweep2_11-22-19.log'
x = [[0.0], [0.0], [0.0], [6.0], [3.0], [1.0]]
dx_sum = np.zeros((3,1))
P = np.zeros((6,6))
Landmark_Positions = {1:3}
dt1 = 0.0
dt2 = 0.0 #In reality, these would be grabbed from the Arduino.

res = PARSER.parser(single_scan)
for i in range(0, len(res)):
    print(res[i]['Angular Increment'])
# These are only included so that RANSAC can run its sampling algorithm
#angle_increment = np.radians(res[index]['Angular Increment']) #for when the angle increment value is returned correctly by the parser
angle_increment = np.radians(270/811)
message_count = res[-1]['Message Count']
start_angle = np.radians(res[-1]['Start Angle'])
end_angle = start_angle + (message_count-1)*angle_increment
(x, dx_sum) = EKF.UpdatePosition(x, dx_sum, dt1, dt2)
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
(Landmarks_New, LSRP_list, Unassociated_Points) = RANSAC.RANSAC(scan, start_angle, end_angle)
end = time.time()
print("RANSAC took "+ str(end-start) + " seconds to process "+ str(message_count) +" points. Now associating new landmarks with current landmarks...")

Landmark_Pairs = RANSAC.PairLandmarks(Landmarks_New, Landmark_Positions, x, P)
(x, P) = EKF.EKF(x, dx_sum, P, Landmark_Positions, Landmarks_New, Landmark_Pairs)
fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(xs, ys, zs, s=3, marker='x', color='r')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
RANSAC.plotLSRPs(ax, LSRP_list, ymax=7000)
ax.view_init(0, 0)
plt.show()
