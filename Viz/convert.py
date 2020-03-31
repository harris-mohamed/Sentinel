# Utility functions to convert an AWS dataset into the proper format to be visualized
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

# Idk what these lines do 
if __name__=="__main__":
    output = readFromAWS('hallway_scan_3-8-2020')


scan_kalman = readFromAWS('hallway_scan_3-8-2020')

for scan in scan_kalman:
    scan['euler'] = kalman.Gravity([[float(scan['Ax'])], [float(scan['Ay'])], [float(scan['Az'])]]).__repr__()

test_coordinates = ransac.ConvertToCartesianEulerAngles(scan_kalman)

# kalman_coordinates = ransac.ConvertToCartesianEulerAngles(scan_kalman)
# standard_coordinates = ransac.ConvertToCartesian(scan_kalman)

# for key, value in standard_coordinates.items():
#     print(value)

with open("kalman.txt", "w") as file:
    for key, value in test_coordinates.items():
        for coordinate in value:
            file.write(str(float(coordinate)) + " ")
        file.write("\n")

# with open("standard.txt", "w") as file:
#     for key, value in standard_coordinates.items():
#         for coordinate in value:
#             file.write(str(float(coordinate)) + " ")
#         file.write("\n")

kalman_file_name = "kalman.txt"
# standard_file_name = "standard"

os.system("./txt2las64.exe -parse xyz -set_scale 0.01 0.01 0.01 -i " + kalman_file_name + " -o lidar.las")
os.system("./PotreeConverter.exe lidar.las -o accel_test")

# Below is working conversion data
# runs = ['2020-1-9_22-28-2-DIAGNOSTIC_RUN.txt', 
#         '2020-1-9_22-42-41-NICK1.txt', 
#         '2020-1-9_22-44-47-NICK2.txt',
#         '2020-1-9_22-53-29-LIGHTSOUT.txt',
#         '2020-1-9_22-53-55-LIGHTSOUTNICK.txt',
#         '2020-1-9_22-54-27-MOVINGNICK.txt',
#         '2020-1-9_22-57-23-BALLBOX.txt',
#         '2020-1-9_22-58-1-BALLRAISEDBOX.txt',
#         '2020-1-9_22-58-33-SNOWMAN.txt']

# for run in runs:
#     with open(run, 'r') as file:
#         stringdicts = file.readlines()
#         res = [] 
#         for stringdict in stringdicts:
#             res.append(ast.literal_eval((stringdict)))
        
#         curr_conv = ransac.ConvertToCartesian(res)
#         new_name = run[:-4]
#         file_name = str(new_name) + '_conv.txt'
#         with open(str(new_name) + '_conv.txt', 'w') as f:
#             for key, value in curr_conv.items():
#                 for coordinate in value:
#                     f.write(str(float(coordinate)) + " ")
#                 f.write("\n")
        
#         loc = str(new_name + '_las')
#         os.mkdir(loc)

#         os.system("./txt2las64.exe -parse xyz -set_scale 0.01 0.01 0.01 -i " + file_name + " -o " + loc + "/" + new_name + ".las")
#         os.system("./PotreeConverter.exe " + loc + "/" + new_name + ".las" + "-o " + loc)
