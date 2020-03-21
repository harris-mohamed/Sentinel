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
from ast import literal_eval

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
# Loop through all points, compute normal vector for each cluster of points 
for coordinate in actual_coordinates:
    # Explore the nearest n neighbors. Doing this without any optimizations as of now.
    # if count == 100:
    #     break
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

        actual_coordinates.remove(curr_point)
    
    # Find the plane classifying the 10 points 
    tmp_A = []
    tmp_B = []
    for i in range(POINTS_TO_EXPLORE):
        tmp_A.append([actual_x[i], actual_y[i], 1])
        tmp_B.append(actual_z[i])
    b = np.matrix(tmp_B).T 
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b
    # print("Fit: ", fit)
    # print("a: ", fit[0,0], "b: ", fit[1,0], "c: ", fit[2,0])
    mag = np.sqrt(fit[0,0]**2 + fit[1,0]**2 + fit[2,0]**2)
    nearest_neighbors[str(coordinate)] = actual_points
    normalized = []
    normalized.append((fit[0,0] / mag))
    normalized.append((fit[1,0] / mag))
    normalized.append((fit[2,0] / mag))
    # points_dict[str(coordinate)] = normalized
    points_dict[str(normalized)] = coordinate
    # print("Normalized: ", normalized)

ANGLE_THRESHOLD = 80.0
num_features = 0
features = {}
# Now loop through the dictionary and average normal vectors
for key, value in points_dict.items():
    found = False
    old_key = ''
    copy_buffer = [] 
    avg_normal = []
    for k, v in features.items():
        current_normal_vector = literal_eval(key)
        compare_normal_vector = literal_eval(k)
        angle = np.degrees(np.arccos(current_normal_vector[0]*compare_normal_vector[0] + current_normal_vector[1]*compare_normal_vector[1] + current_normal_vector[2]*compare_normal_vector[2]))

        if angle <= ANGLE_THRESHOLD:
            # Compute the average of the normal vectors 
            avg_x = (current_normal_vector[0] + compare_normal_vector[0]) / 2
            avg_y = (current_normal_vector[1] + compare_normal_vector[1]) / 2
            avg_z = (current_normal_vector[2] + compare_normal_vector[2]) / 2
            avg_normal.append(avg_x)
            avg_normal.append(avg_y)
            avg_normal.append(avg_z)

            copy_buffer = v.copy()
            copy_buffer.append(avg_normal)
            old_key = k
            found = True 
            break 
    
    if found == True:
        features[str(avg_normal)] = copy_buffer
        del features[old_key]
        found = False 
    else:
        features[key] = [value]
        num_features = num_features + 1

print(num_features)

# plt.figure()
# ax = plt.subplot(111, projection='3d')
# Plot the results 
for key, value in features.items():
    print(key)

    # for coordinate in value:
    #     current_x.append(coordinate[0])
    #     current_y.append(coordinate[1])
    #     current_z.append(coordinate[2])
    
 
    # ax.scatter(current_x, current_y, current_z, color='b')

    # xlim = ax.get_xlim()
    # ylim = ax.get_ylim()
    # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
    #                   np.arange(ylim[0], ylim[1]))
    # Z = np.zeros(X.shape)
    # for r in range(X.shape[0]):
    #     for c in range(X.shape[1]):
            # Z[r,c] = curr_vector[0] * X[r,c] + curr_vector[1] * Y[r,c] + curr_vector[2]
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
    
