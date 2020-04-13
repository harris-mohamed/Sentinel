from __future__ import print_function # Python 2/3 compatibility
import boto3
import json
import decimal
import sys
import os
import ast
from boto3.dynamodb.conditions import Key, Attr
from subprocess import call
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

sys.path.append("..\PARSER")

import quick
import RANSAC.RANSAC as ransac
import KALMAN as kalman

X = 0.005 #unitless, the maximum distance a point must be from an LSRP in RANSAC to be considered in tolerance.
C = 500 #Consensus for RANSAC, number of points that must pass the tolerance check of the LSRP
N = 30 #Max number of trials in RANSAC before ending
S = 10 #Number of points to sample for RANSAC
S_LIM = 0.05 #unitless, half the length of a side of the cube to draw around the randomly sampled point in RANSAC

LSRP_list = []
##
##dynamodb = boto3.resource('dynamodb', region_name='us-east-2', endpoint_url='http://dynamodb.us-east-2.amazonaws.com')
##table = dynamodb.Table('SENTINEL')

scan_kalman = quick.readFromAWS('hallway_scan_3-8-2020')

for scan in scan_kalman:
    scan['euler'] = kalman.Gravity([[float(scan['Ax'])], [float(scan['Ay'])], [float(scan['Az'])]]).__repr__()

test_coordinates = ransac.ConvertToCartesianEulerAngles(scan_kalman)
(scaled_coordinates, factor) = ransac.NormalizeCartesian(test_coordinates)


(Landmarks_New, LSRP_list, Unassociated_Points, Associated_Points) = ransac.RANSAC(scaled_coordinates, X, C, N, S, S_LIM)
print("RANSAC found "+str(len(LSRP_list))+" LSRP's!")
print(LSRP_list)

## Assemble lists of the unassociated point coordinates for visualizations
xun = []
yun = []
zun = []
for point in Unassociated_Points.values():
    xun.append(point[0])
    yun.append(point[1])
    zun.append(point[2])

## Assemble lists of the associated point coordinates for visualizations
xa = []
ya = []
za = []
for point in Associated_Points.values():
    xa.append(point[0])
    ya.append(point[1])
    za.append(point[2])

print("Plotting Points and Landmarks...")
fig = plt.figure()
plt.clf()
ax = Axes3D(fig)
ax.scatter(xun, yun, zun, s=1, marker='o', color='r')
ax.scatter(xa, ya, za, s=1, marker='x', color='b')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1, 1)
ax.set_zlim3d(0, 1)
ransac.plotLSRPs(ax, LSRP_list)
ax.view_init(45, -90)
plt.show()
