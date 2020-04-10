# My attempt at point cloud clustering 
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
import random

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

# Point cloud clustering begins here 
CLUSTER_POINTS = 1000
clusters = {}
normal_vectors = {}
center_points = []

# Populate both the dictionaries
# clusters is a dict where the key is the center point and the value is the list of points associated with the cluster 
# normal_vectors is a dict where the key is the center point and the value is the normal vector associated with the cluster 
while len(actual_coordinates) != 0:

    # Pick a random point to find the cluster of 
    random_point_idx = random.randrange(0, len(actual_coordinates))
    random_point = actual_coordinates[random_point_idx]
    actual_coordinates.remove(random_point)

    # Find the closest points to make the cluster with 
    closest_points_buffer= []
    heapq.heapify(closest_points_buffer)
     
    # This step is expensive, but oh well
    for point in actual_coordinates:
        heapq.heappush(closest_points_buffer, (distPoints(random_point, point), point))
    
    # Can't form a cluster with less than 3 points, just discard it
    if (len(actual_coordinates) < 3):
        break

    # If we don't have enough points left, we will use what is left for the last cluster
    if (len(actual_coordinates) < CLUSTER_POINTS):
        CLUSTER_POINTS = len(actual_coordinates)

    cluster_buffer = []
    cluster_buffer.append(random_point)

    # Populate the cluster buffer array and find the center point at the same time(ish)
    x_val = 0
    y_val = 0
    z_val = 0
    for i in range(CLUSTER_POINTS):
        curr_min = heapq.heappop(closest_points_buffer)[1]
        x_val = x_val + curr_min[0]
        y_val = y_val + curr_min[1]
        z_val = z_val + curr_min[2]
        cluster_buffer.append(curr_min)
        actual_coordinates.remove(curr_min)
    
    # Find the actual center point 
    x_avg = (x_val + cluster_buffer[0][0]) / (CLUSTER_POINTS + 1)
    y_avg = (y_val + cluster_buffer[0][1]) / (CLUSTER_POINTS + 1)
    z_avg = (z_val + cluster_buffer[0][2]) / (CLUSTER_POINTS + 1) 
    center_point = [x_avg, y_avg, z_avg]

    # Find the point that is the closest to the center point 
    min_dist = 999999999
    min_point = cluster_buffer[0]
    for i in range(CLUSTER_POINTS + 1):
        dist_calc = distPoints(cluster_buffer[i], center_point)
        if (dist_calc < min_dist):
            min_dist = dist_calc
            min_point = cluster_buffer[i]
    
    # Now find the normal vector associated with each cluster 
    x_coor = []
    y_coor = []
    z_coor = []
    tmp_A = []
    tmp_B = []

    for i in range(CLUSTER_POINTS + 1):
        curr_point = cluster_buffer[i]
        x_coor.append(curr_point[0])
        y_coor.append(curr_point[1])
        z_coor.append(curr_point[2])
    
    for i in range(CLUSTER_POINTS + 1):
        tmp_A.append([x_coor[i], y_coor[i], 1])
        tmp_B.append(z_coor[i])
    b = np.matrix(tmp_B).T 
    A = np.matrix(tmp_A)
    orig_output = (A.T * A).I * A.T * b
    normal_vec = [orig_output[0,0], orig_output[1,0], -1, orig_output[2,0]]

    # normal_vec is of form [a, b, c]
    # But the plane is of form Ax + By - z = -C
    # Normalize the normal vector 
    magnitude = np.sqrt(normal_vec[0]**2 + normal_vec[1]**2 + normal_vec[2]**2)
    normalized = [(normal_vec[0] / magnitude), (normal_vec[1] / magnitude), (normal_vec[2] / magnitude), normal_vec[3]]

    # Store the cluster 
    clusters[str(center_point)] = cluster_buffer
    normal_vectors[str(center_point)] = normalized
    center_points.append(center_point)

    # print("The cluster contains: ")
    # for i in cluster_buffer:
    #     print(str(i))
    
    # print(" ")
    # print("The center point of the cluster is: ")
    # print(str(center_point))

    # print(" ")
    # print("The center point I found is: ")
    # print(str(min_point))

    # print(" ")
    # print("The plane I found is: ")
    # print(str(orig_output[0,0]) + "x + " + str(orig_output[1,0]) + "y + 1 = -" + str(orig_output[2,0]))

    # print(" ")
    # print("The normal vector representing the cluster is: ")
    # print(str(normal_vec))

    # print(" ")
    # print("The normalized vector is: ")
    # print(str(normalized))
    # break
    
features = {}

current_center_point_idx = random.randrange(0, len(center_points))
current_center_point = center_points[current_center_point_idx]
current_cluster_points = clusters[str(current_center_point)]
current_normal = normal_vectors[str(current_center_point)]

# Set the first feature to the randomly selected cluster 
features[str(current_normal)] = current_cluster_points
center_points.remove(current_center_point)

while len(center_points) != 0:
    # Find the closest cluster to the current one 
    min_point = center_points[0]
    min_dist = 999999999
    for point in center_points:
        dist_calc = distPoints(point, current_center_point)
        if (dist_calc < min_dist):
            min_dist = dist_calc
            min_point = point 
    
    closest_cluster = clusters[str(min_point)]
    closest_normal = normal_vectors[str(min_point)]
    center_points.remove(min_point)

    # MIGHT NEED TO ADD LOGIC TO ENSURE THEY ARE NEIGHBORS. For now, I will proceed with the assumption that they are neighbors.
    # To carry out the above comment, check the angle between the vector that connects the points and the average normal vector

    FOUND = False 
    # Check which of the existing features the closest normal matches up with 
    for key, value in features.items():
        feature_normal = literal_eval(key)

        # Find the angle between the normal vectors of the feature and the closest cluster
        angle = np.degrees(np.arccos(feature_normal[0]*closest_normal[0] + feature_normal[1]*closest_normal[1] + feature_normal[2]*closest_normal[2]))

        ANGLE_THRESHOLD = 70

        # If the conditions are met, it is combined with the current feature 
        if (angle < ANGLE_THRESHOLD):
            avg_normal = [((feature_normal[0] + closest_normal[0]) / 2), ((feature_normal[1] + closest_normal[1]) / 2), ((feature_normal[2] + closest_normal[2]) / 2), ((feature_normal[3] + closest_normal[3]) / 2)]
            features[str(avg_normal)] = features[str(feature_normal)]
            features[str(avg_normal)].append(clusters[str(min_point)])
            del features[str(feature_normal)]
            FOUND = True 
            break 

    # If FOUND has not been changed, we have found a new feature 
    if FOUND == False:
        features[str(closest_normal)] = clusters[str(min_point)]
    
    # No matter what, update the current center point
    current_center_point = min_point

print("Number of features found: " + str(len(features)))

for key, value in features.items():
    print("Features found: " + key)
    # print("Coordinates found: " + str(value))
# random_cl_point_idx = random.randrange(0, len(center_points))
# random_cl_point = center_points[random_cl_point_idx]
# cluster_points = clusters[str(random_cl_point)]
# cluster_normal = normal_vectors[str(random_cl_point)]
# features[str(cluster_normal)] = cluster_points
# center_points.remove(random_cl_point)

# # Loop through the points
# while len(center_points) != 0:

#     # Find the closest cluster to the one we are looking at
#     min_point = center_points[0]
#     min_dist = 9999999
#     for point in center_points:
#         dist_calc = distPoints(point, random_cl_point)
#         if (dist_calc < min_dist):
#             min_dist = dist_calc
#             min_point = point 
    
#     center_points.remove(min_point)

#     ANGLE_THRESHOLD = 45 
#     # If the vector between the center points is perpendicular to the
#     for key, value in features.items():
#         # First check the angle between the normal vectors 
#         normal_vec_feature = literal_eval(key)
#         normal_vec_min = normal_vectors[str(min_point)]
#         angle = np.degrees(np.arccos(normal_vec_feature[0]*normal_vec_min[0] + normal_vec_feature[1]*normal_vec_min[1] + normal_vec_feature[2]*normal_vec_min[2]))

#         # If the if statement is satisfied, then the vectors are part of the same feature
#         if (angle < ANGLE_THRESHOLD) or (angle > 180 - ANGLE_THRESHOLD):
#             avg_normal = [((normal_vec_feature[0] + normal_vec_min[0]) / 2), ((normal_vec_feature[1] + normal_vec_min[1]) / 2), ((normal_vec_feature[2] + normal_vec_min[2]) / 2)]
#             hold = features[str(normal_vec_feature)]
#             features[str(avg_normal)] = hold.append(clusters[str()])
#         # Otherwise, it's a new feature 
#         else:

#     # Check the angle between the normal vectors 
#     normal_vec_random_cl = normal_vectors[str(random_cl_point)] 
#     normal_vec_min = normal_vectors[str(min_point)]

    
#     angle = np.degrees(np.arccos(normal_vec_random_cl[0]*normal_vec_min[0] + normal_vec_random_cl[1]*normal_vec_min[1] + normal_vec_random_cl[2]*normal_vec_min[2]))

    
