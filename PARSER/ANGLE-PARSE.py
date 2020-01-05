# Project Sentinel: TiM781 Log File Parser w/Angle correlation
# Harris M 
# January 4, 2020 

# LIBRARIES 
import re
import numpy as np
import PARSER as lidar_parser  

lidar_4 = '../../sample_logs/attempt-4-lidar.log'
angle_4 = '../../sample_logs/attempt-4.log'
lidar_5 = '../../sample_logs/attempt-5-lidar.log'
angle_5 = '../../sample_logs/attempt-5.log'
lidar_6 = '../../sample_logs/attempt-6-lidar.log'
angle_6 = '../../sample_logs/attempt-6.log'

lidar_p4 = lidar_parser.parser(lidar_4)
lidar_p5 = lidar_parser.parser(lidar_5)
lidar_p6 = lidar_parser.parser(lidar_6)

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
    if len(angles) > len(lidar):
        angles = angles[:len(lidar)]
    elif len(lidar) > len(angles):
        lidar = lidar[:len(angles)]

    for message in range(len(lidar)):
        curr = lidar[message]
        curr['Motor encoder'] = angles[message]
    
    return lidar
    
merge_4 = merge(angle_parser(angle_4), lidar_p4)
merge_5 = merge(angle_parser(angle_5), lidar_p5)
merge_6 = merge(angle_parser(angle_6), lidar_p6)
