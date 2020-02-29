# Project Sentinel: TiM781 - Finishing the live parser, once and for all.
# Harris M 
# January 21, 2020 

# LIBRARIES 
import sys
import socket
import time
from datetime import datetime 
import struct

# CONNECTION CONSTANTS
PORT = 2112
IP_ADDRESS = '169.254.100.100'

# SCAN CONSTANTS 
REQUEST_SINGLE_SCAN = b'\x02sRN LMDscandata\x03'  
REQUEST_CONT_SCAN = b'\x02sEN LMDscandata 1\x03'
STOP_CONT_SCAN = b'\x02sEN LMDscandata 0\x03'
LOG_IN_CLIENT = b'\x02sMN SetAccessMode 03 F4724744\x03'
LOG_OUT = b'\x02sMN Run\x03'
READ_FOR_ANGLE_FREQ = b'\x02sRN LMPscancfg\x03'

# CONSTANTS NOT TESTED 
LOAD_FACTORY_DEFAULTS = b'\x02sMN mSCloadfacdef\x03'
REBOOT_TEST = b'\x02sMN mSCreboot\x03'
SAVE_PARAMETERS_PERMANENT = b'\x02sMN mEEwriteall\x03'

# SENSOR CONSTANTS 
microsecond = 10**(-6)
angle_step = 10**(4)
scale_factor = {'3F800000': '1x',
                '40000000': '2x' }

# Function: type converter
# Description: Converts a number to specified base
def type_conv(num, base):
    initial = int(num, 16)
    if base == 's8':
        conv = (initial + 2**7) % 2**8 - 2**7
    elif base == 'u8':
        conv = initial % 2**8 
    elif base == 's16':
        conv = (initial + 2**15) % 2**16 - 2**15
    elif base == 'u16':
        conv = initial % 2**16 
    elif base == 's32':
        conv = (initial + 2**31) % 2**32 - 2**31
    elif base == 'u32':
        conv = initial % 2**32
    elif base == 's64':
        conv = (initial + 2**63) % 2**64 - 2**63
    elif base == 'u64':
        conv = initial % 2**64
    else:
        return initial
    
    return conv

# Function: log_in 
# Description: Logs into the sensor 
def log_in():
    sock = connect()
    sock.send(LOG_IN_CLIENT)
    scan = message_collect(sock)
    
    if (len(scan) != 3):
        print("Something went wrong.")
    elif (scan[2] == '1'):
        print("Log in successful")
    elif (scan[2] == '0'):
        print("Log in not successful")

# Function: log_out
# Description: Logs out from the sensor
def log_out():
    sock = connect()
    sock.send(LOG_OUT)
    scan = message_collect(sock)

    if (len(scan) != 3) or (scan[1] != 'Run'):
        print("Something went wrong with the save parameters command")
    else: 
        if (scan[2] == 0):
            print("There was some error in logging out and running.")
        elif (scan[2] == 1):
            print("Successful log out and run.")

# Function: connect
# Description: Connects to the sensor
def connect():
    print("Connecting to TiM781...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((IP_ADDRESS, PORT))
    print("Connected to Tim781.")
    return sock

# Function: message_collect
# Description: Grabs a single telegram from the sensor
def message_collect(s):
    scan = []  
    curr = ''
    while 1:
        msg_orig = s.recv(1)
        msg = msg_orig.decode('utf-8')
    
        if msg_orig == b'\x03': 
            scan.append(curr)
            break
        elif msg == ' ':
            scan.append(curr)
            curr = ''
        else:
            curr += msg
    return scan

# Function: read_freq_angle
# Description: Reads the existing frequency and angle settings
def read_freq_angle():
    sock = connect()
    sock.send(READ_FOR_ANGLE_FREQ)
    scan = message_collect(sock)
    
    if (len(scan) != 7) or (scan[1] != 'LMPscancfg'):
        print("Something went wrong with the read")
    else:
        scan_frequency = type_conv(scan[2], 'u32')
        sector_count = type_conv(scan[3], 'u16')
        angle_resolution = type_conv(scan[4], 'u32')
        start_angle = type_conv(scan[5], 's32')
        stop_angle = type_conv(scan[6], 'u32')

        print("Scan frequency is: ", scan_frequency / 100, "hz")
        print("Number of sectors: ", sector_count)
        print("Angular resolution: ", angle_resolution / 10000, "degrees")
        print("Start angle: ", start_angle / 10000, "degrees")
        print("Stop angle: ", stop_angle / 10000, "degrees")

# Function: load_factory_defaults
# Description: Loads the factory defaults into the sensor (WARNING: UNTESTED)
def load_factory_defaults():
    sock = connect()
    sock.send(LOAD_FACTORY_DEFAULTS)
    scan = message_collect(sock)

    if (len(scan) != 2) or (scan[1] != 'mSCloadfacdef'):
        print("Something went wrong with the factory reset.")
    else:
        print("Factory reset successful.")
    
# Function: reboot_device
# Description: Reboots the device
def reboot_device():
    sock = connect()
    sock.send(REBOOT_TEST)
    scan = message_collect(sock)

    if (len(scan) != 2) or (scan[1] != 'mSCreboot'):
        print("Something went wrong with the reboot.")
    else:
        print("Sensor reboot successful.")

# Function: save_param_permanent
# Description: Save parameters permanently
def save_param_permanent():
    sock = connect()
    sock.send(SAVE_PARAMETERS_PERMANENT)
    scan = message_collect(sock)
    
    if (len(scan) != 3) or (scan[1] != 'mEEwriteall'):
        print("Something went wrong with the save parameters command")
    else: 
        print("The parameters were written successfully.")

# Function: custom_message
# Description: Writes some custom message
def custom_message():
    start = '\x02'
    end = '\x03'
    message = input("Enter the custom message here:")
    custom = start + message + end 
    custom = str.encode(custom)
    sock = connect()
    sock.send(custom)
    scan = message_collect(sock)

    print("Output: ", scan)

