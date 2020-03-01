# Project Sentinel: The official Sentinel class
# Harris M 
# February 5, 2020

from __future__ import print_function 
from datetime import datetime 
from boto3.dynamodb.conditions import Key, Attr
import boto3
import json 
import decimal

from time import sleep 
from datetime import datetime
import sys
import socket
import time
import struct

import smbus 
import serial 
import sentinel_reference as s

sys.path.append("../SLAM")

import RANSAC as ransac
import KALMAN as kalman

class SENTINEL:
    """ Function declarations for the SENTINEL class """
    
    bus = smbus.SMBus(3)

    def accel_init(self):
        """Instantiates the MPU-6050 module 

            Args:
                None
            Return:
                None
        """
        self.bus.write_byte_data(s.ACCEL_ADDRESS, s.SMPLRT_DIV, 7)
        self.bus.write_byte_data(s.ACCEL_ADDRESS, s.PWR_MGMT_1, 1)
        self.bus.write_byte_data(s.ACCEL_ADDRESS, s.CONFIG, 0)
        self.bus.write_byte_data(s.ACCEL_ADDRESS, s.GYRO_CONFIG, 24)
        self.bus.write_byte_data(s.ACCEL_ADDRESS, s.INT_ENABLE, 1)

    def singleScan(self):
        """Takes a single scan

        Args:
            None
        Return:
            The single scan
        """
        return s.single_parse() 

    def contScan(self, count):
        """Takes a specified number of consecutive scans

        Args:
            The number of consecutive scans to take
        Return:
            A list of dictionaries, each dictionary contains a parsed scan 
        """ 
        return s.live_parse(count)

    def singleScanPretty(self):
        """Takes a single scan and prints it for debugging

        Args:
            None
        Return:
            The single scan
        """
        current_scan = s.single_parse()
        print(current_scan)
        return current_scan 

    def singleScanWithUpload(self):
        """Takes a single scan and uploads it to AWS

        Args:
            None
        Return:
            The single scan
        """

        current_scan = s.single_parse() 
        s.uploadToAws(current_scan)
        return current_scan

    def custom_message(self):
        """Sends a custom message to the LIDAR sensor

        Args:
            None
        Return:
            None
        """

        start = '\x02'
        end = '\x03'
        message = input("Enter the custom message here:")
        custom = start + message + end 
        custom = str.encode(custom)
        sock = connect()
        sock.send(custom)
        scan = message_collect(sock)

        print("Output: ", scan)

        
    def read_freq_angle(self):
        """Reads and prints current angle and frequency settings 

        Args:
            None
        Return:
            None
        """
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

    def load_factory_defaults(self):
        """Loads factory defaults into LIDAR sensor

        Args:
            None
        Return:
            None
        """
        sock = connect()
        sock.send(LOAD_FACTORY_DEFAULTS)
        scan = message_collect(sock)

        if (len(scan) != 2) or (scan[1] != 'mSCloadfacdef'):
            print("Something went wrong with the factory reset.")
        else:
            print("Factory reset successful.")
        
    def reboot_device(self):
        """Reboots LIDAR sensor

        Args:
            None
        Return:
            None
        """
        sock = connect()
        sock.send(REBOOT_TEST)
        scan = message_collect(sock)

        if (len(scan) != 2) or (scan[1] != 'mSCreboot'):
            print("Something went wrong with the reboot.")
        else:
            print("Sensor reboot successful.")

    def save_param_permanent(self):
        """Saves parameters to the LIDAR sensor

        Args:
            None
        Return:
            None
        """

        sock = connect()
        sock.send(SAVE_PARAMETERS_PERMANENT)
        scan = message_collect(sock)
        
        if (len(scan) != 3) or (scan[1] != 'mEEwriteall'):
            print("Something went wrong with the save parameters command")
        else: 
            print("The parameters were written successfully.")

    # def beginArduinoComm():
    #     """Starts communicating with the Arduino

    #         Args:
    #             None
    #         Return:
    #             None
    #     """
        
    def read_raw_data(self, addr):
        """Instantiates the MPU-6050 module 

            Args:
                None
            Return:
                None
        """
        high = self.bus.read_byte_data(s.ACCEL_ADDRESS, addr)
        low = self.bus.read_byte_data(s.ACCEL_ADDRESS, addr + 1)

        value = ((high << 8) | low)

        if (value > 32768):
            value = value - 65536
        
        return value         

    def manualControl(self): 
        """Enters manual control mode

        Args:
            None
        Return:
            None
        """
        print("Entering manual control mode...")
        print("Enter WASD for direction control, or q to quit")

        while 1: 
            curr = input()
            if (curr == 'W'):
                ser.write('W')
            elif (curr == 'A'):
                ser.write('A')
            elif (curr == 'S'):
                ser.write('S')
            elif (curr == 'D'):
                ser.write('D')
            elif (curr == 'q'):
                break 

    def __init__(self):
        """Initializes neccesary constants and communication busses

        Args:
            None
        Return:
            None
        """
        dynamodb = boto3.resource(s.DB, region_name=s.REGION_NAME, endpoint_url=s.ENDPOINT_URL)
        table = dynamodb.Table(s.TABLE_NAME)
        s.setupSerial()
        self.accel_init()
        (Ax, Ay, Az) = s.accel_read()
        x = kalman.Gravity([[Ax], [Ay], [Az]])

sentinel = SENTINEL()
