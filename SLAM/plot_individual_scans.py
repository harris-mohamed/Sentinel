#Created by: Nicholas O'Brien
#Project Sentinel; plot_individual_scans
#Created: January 11th, 2020

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
import scipy.ndimage as nim
from matplotlib import animation

sample_logs = '../../sample_logs/'
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

parsed_log_file = '2020-1-9_22-28-2-DIAGNOSTIC_RUN.txt'
##parsed_log_file = '2020-1-9_22-42-41-NICK1.txt'
##parsed_log_file = '2020-1-9_22-44-47-NICK2.txt'
##parsed_log_file = '2020-1-9_22-53-29-LIGHTSOUT.txt'
##parsed_log_file = '2020-1-9_22-53-55-LIGHTSOUTNICK.txt'
##parsed_log_file = '2020-1-9_22-54-27-MOVINGNICK.txt'
##parsed_log_file = '2020-1-9_22-57-23-BALLBOX.txt'
##parsed_log_file = '2020-1-9_22-59-1-BALLRAISEDBOX.txt'
##parsed_log_file = '2020-1-9_22-58-33-SNOWMAN.txt'

parsed_log_file = sample_logs + parsed_log_file

with open(parsed_log_file, 'r') as file:
    stringdicts = file.readlines()
    res = []
    for stringdict in stringdicts:
        res.append(ast.literal_eval(stringdict))
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

fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')
line, = ax.plot([],[],markersize=1, marker='o',linewidth=0)
ax.set_ylim([0, 8000])

def init():
    line.set_data([], [])
    return line,

def animate(i):
	theta = res[i]['Motor encoder']
	message_count = res[i]['Quantity']
	angle_increment = np.radians(res[i]['Angular Increment'])
	start_angle = np.radians(res[i]['Start Angle'])
##	range_data = nim.median_filter(res[i]['Measurement'], size=15)
	range_data = res[i]['Measurement']
	ts = []
	rs = []
	for index in range(len(range_data)):
		ts.append(start_angle+angle_increment*index)
		rs.append(range_data[index])
	line.set_data(ts, rs)
	plt.text(-np.pi/3, 6000, "Motor Angle: "+str(theta)+" Degrees")
	return line,
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=len(res), interval=200, blit=True)
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

anim.save('no_filter.mp4', writer=writer)
plt.show()
