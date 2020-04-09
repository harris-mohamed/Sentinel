# Project Sentinel: A file to hold all of the constants and utility functions 
# Harris M 
# February 12, 2020

# Libraries 
import serial 

# Port constants
PORT = 2112 
IP_ADDRESS = '169.254.100.100'
REQUEST_SINGLE_SCAN = b'\x02sRN LMDscandata\x03'  
REQUEST_CONT_SCAN = b'\x02sEN LMDscandata 1\x03'
STOP_CONT_SCAN = b'\x02sEN LMDscandata 0\x03'

# Sensor constants
MICROSECOND = 10**(-6)
ANGLE_STOP = 10**(4)
SCALE_FACTOR = {'3F800000': '1x',
                '40000000': '2x' }
            
# RPI CONSTANTS 
PWR_MGMT_1 = 0x6B 
SMPLRT_DIV = 0x19 
CONFIG = 0x1A 
GYRO_CONFIG = 0x1B 
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_BAUD = 115200

# ACCELEROMETER CONSTANTS 
ACCEL_CONSTANT = 16384.0
GYRO_CONSTANT = 131.0
ACCEL_ADDRESS = 0x68
ACCEL_BUS_ADDRESS = 3
QK_VAL = 1.0
RK_VAL = 1000.0

# AWS CONSTANTS  
DB = 'dynamodb'
REGION_NAME = 'us-east-2'
ENDPOINT_URL = 'http://dynamodb.us-east-2.amazonaws.com'
TABLE_NAME = 'SENTINEL'

# SERIAL BUS CONSTANTS 
startSentinel = '<'
endSentinel = '>'
