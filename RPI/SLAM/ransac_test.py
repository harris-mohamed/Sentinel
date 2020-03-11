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

sys.path.append("../RPI/SLAM")

import RANSAC.RANSAC as ransac
import KALMAN as kalman

def readFromAWS(name):
        """ Grabs all scans associated with a certain name

            Args:
                Name we are interested in 
            Return:
                A list of dictionaries 
        """

        output = []

        response = table.query(KeyConditionExpression=Key('Name').eq(name))

        for scan in response['Items']:
            output.append(scan)

        return output

X = 5 #mm, the maximum distance a point must be from an LSRP in RANSAC to be considered in tolerance.
C = 50 #Consensus for RANSAC, number of points that must pass the tolerance check of the LSRP
N = 30 #Max number of trials in RANSAC before ending
S = 30 #Number of points to sample for RANSAC
S_LIM = 300 #mm, half the length of a side of the cube to draw around the randomly sampled point in RANSAC

LSRP_list = []

dynamodb = boto3.resource('dynamodb', region_name='us-east-2', endpoint_url='http://dynamodb.us-east-2.amazonaws.com')
table = dynamodb.Table('SENTINEL')

scan_kalman = readFromAWS('hallway_scan_3-8-2020')

for scan in scan_kalman:
    scan['euler'] = kalman.Gravity([[float(scan['Ax'])], [float(scan['Ay'])], [float(scan['Az'])]]).__repr__()

test_coordinates = ransac.ConvertToCartesianEulerAngles(scan_kalman)

xs = []
ys = []
zs = []
for point in test_coordinates.values():
    xs.append(point[0])
    ys.append(point[1])
    zs.append(point[2])

(Landmarks_New, LSRP_list, Unassociated_Points) = ransac.RANSAC(test_coordinates, X, C, N, S, S_LIM)
print(LSRP_list)

print("Plotting Points and Landmarks...")
fig = plt.figure()
plt.clf()
ax = Axes3D(fig)
ax.scatter(xs, ys, zs, s=1, marker='o', color='r')
#    ax.scatter(xfs, yfs, zfs, s=1, marker='x', color='b')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim3d(-3000, 3000)
ax.set_ylim3d(-3000, 3000)
#    ax.set_zlim3d(-2000, 2000)
ransac.plotLSRPs(ax, LSRP_list, ymax=7000)
ax.view_init(45, -90)
plt.show()