# My attempt at RANSAC 
# Author: Harris Mohamed 

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
import heapq

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import RANSAC.RANSAC as ransac
import KALMAN as kalman

dynamodb = boto3.resource('dynamodb', region_name='us-east-2', endpoint_url='http://dynamodb.us-east-2.amazonaws.com')
table = dynamodb.Table('SENTINEL')

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

# Lines to handle python class
if __name__=="__main__":
    output = readFromAWS('hallway_scan_3-8-2020')

scan = 'arc_test1_2'

curr_scan = readFromAWS(scan)

for data in curr_scan:
    data['euler'] = kalman.Gravity([[float(data['Ax'])], [float(data['Ay'])], [float(data['Az'])]]).__repr__()

curr_coordinates = ransac.ConvertToCartesianEulerAngles(curr_scan) 

actual_coordinates = []

for key, value in curr_coordinates.items():
    hold = []
    for coordinate in value:
        hold.append(coordinate[0])
    actual_coordinates.append(hold)

# Helper function to find distance between points 
def distPoints(p0, p1):
    return (np.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2))

# Actual RANSAC stuff begins below

POINTS_TO_EXPLORE = 10  # Number of points to explore 

# actual_points = []
# actual_x = []
# actual_y = []
# actual_z = []

points_dict = {}
nearest_neighbors = {}
count = 0
# Loop through all points, compute normal vector for each point 
for coordinate in actual_coordinates:
    # Explore the nearest n neighbors. Doing this without any optimizations as of now.
    print(count)
    if count == 811:
        break
    count = count + 1
    actual_points = []
    actual_x = []
    actual_y = []
    actual_z = []
    n_points = []
    heapq.heapify(n_points)
    mod_list = actual_coordinates.copy()
    mod_list.remove(coordinate)
    for point in mod_list:
        heapq.heappush(n_points, (distPoints(coordinate, point), point))
    
    # Find the nearest 10 points
    for i in range(POINTS_TO_EXPLORE):
        curr_point = heapq.heappop(n_points)[1]
        # print(curr_point)
        actual_points.append(curr_point)
        actual_x.append(curr_point[0])
        actual_y.append(curr_point[1])
        actual_z.append(curr_point[2])
    
    # Find the plane classifying the 10 points 
    tmp_A = []
    tmp_B = []
    for i in range(POINTS_TO_EXPLORE):
        tmp_A.append([actual_x[i], actual_y[i], 1])
        tmp_B.append(actual_z[i])
    b = np.matrix(tmp_B).T 
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b

    nearest_neighbors[str(coordinate)] = actual_points
    points_dict[str(coordinate)] = fit

ANGLE_LOW_THRESHOLD = 80
# Now loop through the dictionary and average nearby normal vectors


    # print(type(fit))
    # print(fit)
    # errors = b - A * fit
    # residual = np.linalg.norm(errors)

    # plt.figure()
    # ax = plt.subplot(111, projection='3d')
    # ax.scatter(actual_x, actual_y, actual_z, color='b') 

    # xlim = ax.get_xlim()
    # ylim = ax.get_ylim()
    # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
    #                   np.arange(ylim[0], ylim[1]))
    # Z = np.zeros(X.shape)
    # for r in range(X.shape[0]):
    #     for c in range(X.shape[1]):
    #         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    # ax.plot_wireframe(X,Y,Z, color='k')

    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.show()

    # break
# # plt.figure()
# # ax = plt.subplot(111, projection='3d')
# # ax.scatter(actual_x, actual_y, actual_z, color='b') 

# xlim = ax.get_xlim()
# ylim = ax.get_ylim()
# X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
#                   np.arange(ylim[0], ylim[1]))
# Z = np.zeros(X.shape)
# for r in range(X.shape[0]):
#     for c in range(X.shape[1]):
#         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
# ax.plot_wireframe(X,Y,Z, color='k')

# # ax.set_xlabel('x')
# # ax.set_ylabel('y')
# # ax.set_zlabel('z')
# # plt.show()
    
