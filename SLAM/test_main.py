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
##single_scan = '../../../Logs/Static-sweep_11-26-19.log'
single_scan = '../../../Logs/dynamic_12-26-19_2252.log'
#single_scan = '../../../Logs/Dynamic-sweep_11-26-19.log'
#single_scan = '../../../Logs/Sweep2_11-22-19.log'
x = [[0.0], [0.0], [0.0], [6.0], [3.0], [1.0]]
dx_sum = np.zeros((3,1))
P = np.zeros((6,6))
Landmark_Positions = {1:3}
dt1 = 0.0
dt2 = 0.0 #In reality, these would be grabbed from the Arduino.

res = PARSER.parser(single_scan)
i=0
lenres = len(res)
print("Successfully parsed the scan! Now removing empty messages...")
while i<lenres:
# These are only included so that RANSAC can run its sampling algorithm
#angle_increment = np.radians(res[index]['Angular Increment']) #for when the angle increment value is returned correctly by the parser
#
#This for loop will simulate receiving a continuous stream of scans from the LIDAR
    if res[i]['Angular Increment']=='': #sometimes, the dynamic scan returns blank dictionary. this removes it
        del res[i]
        lenres = len(res)
    else:
        i += 1
##angle_increment = np.radians(270/811)
##message_count = res[-1]['Message Count']
##start_angle = np.radians(res[-1]['Start Angle'])
##end_angle = start_angle + (message_count-1)*angle_increment
print("The scan has been cleaned, now updating odometry...")
(x, dx_sum) = EKF.UpdatePosition(x, dx_sum, dt1, dt2)
scan = RANSAC.ConvertToCartesian(res, x)
lenscan = len(scan)
print("All "+str(lenscan)+" points have been moved to a new dictionary, now running RANSAC")
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
print("RANSAC took "+ str(end-start) + " seconds to process "+ str(len(scan))+" out of "+str(lenscan) +" points, extracting "+str(len(LSRP_list))+" Landmarks. Now associating new landmarks with current landmarks...")

Landmark_Pairs = RANSAC.PairLandmarks(Landmarks_New, Landmark_Positions, x, P)
(x, P) = EKF.EKF(x, dx_sum, P, Landmark_Positions, Landmarks_New, Landmark_Pairs)
print("Plotting Points and Landmarks...")
fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(xs, ys, zs, s=1, marker='o', color='r')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
##ax.set_xlim3d(-2000, 2000)
##ax.set_ylim3d(-2000, 2000)
##ax.set_zlim3d(-2000, 2000)
RANSAC.plotLSRPs(ax, LSRP_list, ymax=7000)
ax.view_init(45, 45)
plt.show()
