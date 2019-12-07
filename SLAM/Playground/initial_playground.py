# Project Sentinel: TiM781 Log File Parser 
# Harris M 
# October 28, 2019 

# LIBRARIES 
import numpy as np 

# Global array/variable instantiation
input_points = np.array([[1, 3]])

# Function: generate_input
# Description: Generates a specified number of input points (x, y, z)
def generate_input(count):
    np.reshape(input_points, [count, 3])

# SiliconRecruiting@fb.com

generate_input(10)