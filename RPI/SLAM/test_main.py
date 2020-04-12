#Created by: Nicholas O'Brien
#Project Sentinel; main script
#Created: December 7th, 2019
#Last Edit: December 8th, 2019
import sys
import ast
sys.path.append("..\PARSER")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import RANSAC.RANSAC as RANSAC
##import PARSER
import EKF.EKF as EKF
import numpy as np
import quick
##import ANGLEPARSE
X = 10 #mm, the maximum distance a point must be from an LSRP in RANSAC to be considered in tolerance.
C = 5000 #Consensus for RANSAC, number of points that must pass the tolerance check of the LSRP
N = 10 #Max number of trials in RANSAC before ending
S = 1000 #Number of points to sample for RANSAC
S_LIM = 100000 #mm, half the length of a side of the cube to draw around the randomly sampled point in RANSAC
DATABASE = "Arc_test1_2"

res = quick.readFromAWS(DATABASE)
LSRP_list = []
# Log file locations 
##sample_logs = '../../../sample_logs/'
#single_scan = '../../sample_logs/Static-sweep0_11-26-19.log'
#single_scan = '../../sample_logs/Static-sweep1_11-26-19.log'
##single_scan = '../../sample_logs/Static-sweep_11-26-19.log'
##single_scan = '../../sample_logs/dynamic_12-26-19_2252.log'
#single_scan = '../../sample_logs/Dynamic-sweep_11-26-19.log'
#single_scan = '../../sample_logs/Sweep2_11-22-19.log'
##lidar_4 = '../../sample_logs/attempt-4-lidar.log'
##angle_4 = '../../sample_logs/attempt-4.log'
##lidar_5 = '../../sample_logs/attempt-5-lidar.log'
##angle_5 = '../../sample_logs/attempt-5.log'
##lidar_6 = '../../sample_logs/attempt-6-lidar.log'
##angle_6 = '../../sample_logs/attempt-6.log'

##parsed_log_file = '2020-1-9_22-28-2-DIAGNOSTIC_RUN.txt'
##parsed_log_file = '2020-1-9_22-42-41-NICK1.txt'
##parsed_log_file = '2020-1-9_22-44-47-NICK2.txt'
##parsed_log_file = '2020-1-9_22-53-29-LIGHTSOUT.txt'
##parsed_log_file = '2020-1-9_22-53-55-LIGHTSOUTNICK.txt'
##parsed_log_file = '2020-1-9_22-54-27-MOVINGNICK.txt'
##parsed_log_file = '2020-1-9_22-57-23-BALLBOX.txt'
##parsed_log_file = '2020-1-9_22-59-1-BALLRAISEDBOX.txt'
##parsed_log_file = '2020-1-9_22-58-33-SNOWMAN.txt'


x = [[0.0], [0.0], [0.0], [6.0], [3.0], [1.0]]
dx_sum = np.zeros((3,1))
P = np.zeros((6,6))
Landmark_Positions = {1:3}
dt1 = 0.0
dt2 = 0.0 #In reality, these would be grabbed from the Arduino.


##with open(parsed_log_file, 'r') as file:
##    stringdicts = file.readlines()
##    res = []
##    for stringdict in stringdicts:
##        res.append(ast.literal_eval(stringdict))

i=0
lenres = len(res)
print("Successfully parsed the scan! Now removing empty messages and sorting scans by frames...")
while i<lenres:
# These are only included so that RANSAC can run its sampling algorithm
#angle_increment = np.radians(res[index]['Angular Increment']) #for when the angle increment value is returned correctly by the parser
#
#This for loop will simulate receiving a continuous stream of scans from the LIDAR
    if res[i]['Angular Increment']=='' or res[i]['Quantity']!=len(res[i]['Measurement']): #sometimes, the dynamic scan returns blank dictionary. this removes it
        del res[i]
        lenres = len(res)
    else:
        i += 1
        
frames = []
frame = []
for i in range(0, len(res), 1):
    frame.append(res[i])
    if i==0: continue
    
    current_motor_angle = res[i]['Motor encoder']
    previous_motor_angle = res[i-1]['Motor encoder']
    if current_motor_angle<100 and previous_motor_angle>200: #this line is supposed to check that the angle has rolled over.
        del frame[-1]
        frames.append(frame)
        frame = [res[i]]
    if i==len(res)-1:
        frames.append(frame)
print("The scan has been cleaned, now updating odometry...")
##for frame in frames:
##	print("NEW FRAME, LENGTH = "+str(len(frame)))
##	for scan in frame:
##		print(scan['Motor encoder'])
for index in range(0, len(frames), 1):
    frame = frames[index]
    if index>0: break
    frame = frames[1]    
    (x, dx_sum) = EKF.UpdatePosition(x, dx_sum, dt1, dt2)
    scan = RANSAC.ConvertToCartesian(frame, x)
    lenscan = len(scan)
    print("All "+str(lenscan)+" points have been moved to a new dictionary, now running RANSAC on frame "+str(index))
    #for plotting the points later
    xs = []
    ys = []
    zs = []
    for point in scan.values():
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
##    xfs = []
##    yfs = []
##    zfs = []
##    for point in scanfiltered.values():
##        xfs.append(point[0])
##        yfs.append(point[1])
##        zfs.append(point[2])
    start = time.time()
    (Landmarks_New, LSRP_list, Unassociated_Points) = RANSAC.RANSAC(scan, X, C, N, S, S_LIM)
    end = time.time()
    print("RANSAC took "+ str(end-start) + " seconds to process "+ str(len(scan))+" out of "+str(lenscan) +" points, extracting "+str(len(LSRP_list))+" Landmarks. Now associating new landmarks with current landmarks...")
##
##    Landmark_Pairs = RANSAC.PairLandmarks(Landmarks_New, Landmark_Positions, x, P)
##    (x, P) = EKF.EKF(x, dx_sum, P, Landmark_Positions, Landmarks_New, Landmark_Pairs)
##    start = time.time()
##    (Landmarks_New_filter, LSRP_list_filter, Unassociated_Points_filter) = RANSAC.RANSAC(scanfiltered)
##    end = time.time()
##    print("RANSAC took "+ str(end-start) + " seconds to process "+ str(len(scan))+" out of "+str(lenscan) +" points, extracting "+str(len(LSRP_list))+" Landmarks. Now associating new landmarks with current landmarks...")
##
##    Landmark_Pairs_filter = RANSAC.PairLandmarks(Landmarks_New_filter, Landmark_Positions, x, P)
##    (x, P) = EKF.EKF(x, dx_sum, P, Landmark_Positions, Landmarks_New_filter, Landmark_Pairs_filter)
##
    # print("Plotting Points and Landmarks...")
    # fig = plt.figure()
    # plt.clf()
    # ax = Axes3D(fig)
    # ax.scatter(xs, ys, zs, s=1, marker='o', color='r')
##    ax.scatter(xfs, yfs, zfs, s=1, marker='x', color='b')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_xlim3d(-3000, 3000)
    # ax.set_ylim3d(-3000, 3000)
##    ax.set_zlim3d(-2000, 2000)
    # RANSAC.plotLSRPs(ax, LSRP_list, ymax=7000)
    # ax.view_init(45, -90)
    # plt.show(False)
