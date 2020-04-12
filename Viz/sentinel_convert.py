# Clean utility functions to convert an AWS dataset into the proper format to be visualized
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

sys.path.append("../RPI/SLAM")

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

# coordinate_directory is the directory where the .txt files for the coordinates will live
# It should not be tracked on git, make sure this directory is NOT within the git folders
coordinate_directory = '../../sentinel_coordinate/'

# las_directory is the directory where the las files will live
# It should not be tracked on git, make sure this directory is NOT within the git folder
las_directory = '../../sentinel_las/'

# point_cloud_directory is the directory where the point cloud files will live
# It should not be tracked on git, make sure this directory is NOT within the git folder
point_cloud_directory = '../../sentinel_pointclouds/'

# Populate this with the scans we want from AWS. Verify using dynamoDB's query function
scan_list = ['arc_test1_2']

for data_scan in scan_list: 
    # Grab the scan from AWS
    curr_scan = readFromAWS(data_scan)
    data_scan += '_redux1'
    # # Default acceleration orientation 
    # if count == 0:
    # Update the orientation of each scan using the accelerometer
    for data in curr_scan:
        data['euler'] = kalman.Gravity([[float(data['Ax'])], [float(data['Ay'])], [float(data['Az'])]]).__repr__()
    # else:
    #     data_scan += '_euler'
    # Get the coordinates
    curr_coordinates = ransac.ConvertToCartesianEulerAngles(curr_scan) 

    # Define the file paths
    coordinate_filename = coordinate_directory + data_scan + ".txt"
    las_filepath = las_directory + data_scan + ".las"
    point_cloud_filename = point_cloud_directory + data_scan

    with open(coordinate_filename, "w") as file:
        for key, value in curr_coordinates.items():
            for coordinate in value:
                file.write(str(float(coordinate)) + " ")
            
            # Add RGB values for the color black 
            file.write("0 0 0")
            file.write("\n")
    
    # Add a blue point at (0, 0, 0)
    with open(coordinate_filename, "a") as file:
        file.write("0 0 0 0 119 190")
    

    # Make the LAS file for the current file
    os.system("./txt2las64.exe -parse xyzRGB -set_scale 0.01 0.01 0.01 -i " + coordinate_filename + " -o " + las_filepath)

    # Make a new directory for the point cloud
    os.mkdir(point_cloud_filename)

    # Generate the point cloud for the current LAS file
    os.system("./PotreeConverter.exe " + las_filepath + " -o " + point_cloud_filename)


    

