# Project Sentinel: TiM781 Log File Parser w/Angle correlation
# Harris M 
# January 4, 2020 

# LIBRARIES 
import re
import numpy as np
import PARSER as lidar_parser  

lidar_4 = '../LOGS/attempt-4-lidar.log'
angle_4 = '../LOGS/attempt-4.log'
lidar_5 = '../LOGS/attempt-5-lidar.log'
angle_5 = '../LOGS/attempt-5.log'
lidar_6 = '../LOGS/attempt-6-lidar.log'
angle_6 = '../LOGS/attempt-6.log'

lidar_p4 = lidar_parser.parser(lidar_4)