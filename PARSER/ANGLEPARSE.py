# Project Sentinel: TiM781 Log File Parser w/Angle correlation
# Harris M 
# January 4, 2020 

# LIBRARIES 
import re
import numpy as np
import sys
#This try-except statement is for running the test_main.py file, for some reason, it fails to import PARSER correctly from this module.
try: import PARSER.PARSER as lidar_parser
except: import PARSER as lidar_parser

# EXTERNAL PATHS
sys.path.append('../SLAM/RANSAC')

# EXTERNAL LIBRARIES
import RANSAC as ransac

# Function: Angle parser 
# Description: Goes through the PUTTY log files and returns a dict
def angle_parser(file):
    init_parse = lidar_parser.load_file(file)
    init_parse = init_parse[1:]

    for index in range(len(init_parse)):
        curr = init_parse[index]
        init_parse[index] = curr.strip('\n') 
    
    return init_parse

# Function: Angle/LIDAR merger
# Description: Appropriately truncates and then merges angle data and a LIDAR scan
def merge(angles, lidar):
    angles = angles[1:]
    start_angle = angles[0]
    start_angle = start_angle.split(':')
    start_a = start_angle[0]
    lidar_init = lidar[0]
    start_l = float(lidar_init['Time since start-up'])

    for message in lidar:
        curr_lidar_time = float(message['Time since start-up']) - start_l
        curr_min = 99999
        min_angle = 99999
        for angle in angles: 
            curr_angle = angle.split(':')
            curr_angle_time = (float(curr_angle[0]) - float(start_a)) * (10**(-3))
            calc = abs(curr_lidar_time - curr_angle_time)
            if (calc < curr_min):
                curr_min = calc
                min_angle = curr_angle[1]
            
        message['Motor encoder'] = float(min_angle)
        
    return lidar
if __name__=="__main__":
    # Move the logs outside the version controlled files, then updates the paths below
    lidar_4 = '../../sample_logs/attempt-4-lidar.log'
    angle_4 = '../../sample_logs/attempt-4.log'
    lidar_5 = '../../sample_logs/attempt-5-lidar.log'
    angle_5 = '../../sample_logs/attempt-5.log'
    lidar_6 = '../../sample_logs/attempt-6-lidar.log'
    angle_6 = '../../sample_logs/attempt-6.log'

    lidar_p4 = lidar_parser.parser(lidar_4)
    lidar_p5 = lidar_parser.parser(lidar_5)
    lidar_p6 = lidar_parser.parser(lidar_6)
    merge_4 = merge(angle_parser(angle_4), lidar_p4)

    coor = []
    coor.append(ransac.ConvertToCartesian(merge_4))

    # merge_5 = merge(angle_parser(angle_5), lidar_p5)
    # merge_6 = merge(angle_parser(angle_6), lidar_p6)
