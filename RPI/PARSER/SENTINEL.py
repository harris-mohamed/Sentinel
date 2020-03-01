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
import numpy as np

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

    def type_conv(self, num, base):
        """Convert a string to an integer with the proper representation

            Args:
                num (string): String representation of number we wish to convert
                base (string): Specifies the base we would like to convert to. Format is 'xy', 
                            where x designates sign (u for unsigned, s for signed) and y is 
                            number of bits the number is.
            Return:
                The converted number.
        """
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


    def telegram_parse(self, scan):
        """ Converts a raw scan into a neat dictionary  

            Args:
                scan (array): Raw scan data 
            Return:
                Dictionary of parsed values
        """
        telegram = {'Version Number': '',
                    'Device Number': '',
                    'Serial Number': '',
                    'Device Status': '',
                    'Telegram Counter': '',
                    'Scan Counter': '',
                    'Time since start-up': '',
                    'Time of transmission': '',
                    'Scan Frequency': '',
                    'Measurement Frequency': '',
                    'Amount of Encoder': '',
                    '16-bit Channels': '',
                    'Scale Factor': '',
                    'Scale Factor Offset': '',
                    'Start Angle': '',
                    'Angular Increment': '', 
                    'Quantity': '',
                    'Motor encoder': '',
                    'Timestamp': '',
                    'Ax': '',
                    'Ay': '',
                    'Az': '',
                    'Gx': '',
                    'Gy': '',
                    'Gz': '',
                    'Measurement': [] }

        if (scan[1] != 'LMDscandata'):
            print("There is something wrong with the scan data")
        else:
            telegram['Version Number'] = type_conv(scan[2], 'u16')
            telegram['Device Number'] = type_conv(scan[3], 'u16')
            telegram['Serial Number'] = type_conv(scan[4], 'u32')
            telegram['Device Status'] = type_conv(scan[6], 'u8')
            telegram['Telegram Counter'] = type_conv(scan[7], 'u16')
            telegram['Scan Counter'] = type_conv(scan[8], 'u16')
            telegram['Time since start-up'] = type_conv(scan[9], 'u32') * microsecond
            telegram['Time of transmission'] = type_conv(scan[10], 'u32') * microsecond
            telegram['Scan Frequency'] = type_conv(scan[16], 'u32') 
            telegram['Measurement Frequency'] = type_conv(scan[17], 'u32') 
            telegram['Amount of Encoder'] = type_conv(scan[18], 'u32')
            telegram['16-bit Channels'] = type_conv(scan[19], 'u32')
            telegram['Scale Factor'] = scale_factor[scan[21]]
            telegram['Scale Factor Offset'] = type_conv(scan[22], 'u32')
            telegram['Start Angle'] = type_conv(scan[23], 's32') / angle_step
            telegram['Angular Increment'] = type_conv(scan[24], 'u16') / angle_step
            telegram['Quantity'] = type_conv(scan[25], 'u16')

            for message in range(26, 26 + telegram['Quantity']):
                telegram['Measurement'].append(type_conv(scan[message], 'u16'))
            
            telegram['Timestamp'] = scan[-1]
            
            Ax, Ay, Az, Gx, Gy, Gz = accel_read()
            telegram['Ax'] = Ax
            telegram['Ay'] = Ay
            telegram['Az'] = Az
            telegram['Gx'] = Gx
            telegram['Gy'] = Gy
            telegram['Gz'] = Gz

        return telegram 

    def live_parse(self, count):
        """ Scans as many consecutive scans as necessary  

            Args:
                scan (array): Raw scan data 
            Return:
                Dictionary of parsed values
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((IP_ADDRESS, PORT))
        sock.send(REQUEST_CONT_SCAN)

        scans = []
        while 1:
            scan = []
            curr = ''
            while 1:
                msg_orig = sock.recv(1)
                msg = msg_orig.decode('utf-8')

                if msg_orig == b'\x03': 
                    scan.append(curr)
                    break
                elif msg == ' ':
                    scan.append(curr)
                    curr = ''
                else:
                    curr += msg
            
            if (len(scan) < 4):
                continue

            curr = datetime.now()
            nice_timestamp = str(curr.year) + "-" + str(curr.month) + "-" + str(curr.day) + "_" + str(curr.hour) + "-" + str(curr.minute) + "-" + str(curr.second)

            scan.append(nice_timestamp)

            initial_parse = telegram_parse(scan)
            scans.append(initial_parse)
            
            if len(scans) == count:
                sock.send(STOP_CONT_SCAN)
                break

    def single_parse(self):
        """ Parses a single scan from the sensor

            Args:
                None 
            Return:
                Dictionary of parsed values
        """
        insideTelegram = False 
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((IP_ADDRESS, PORT))
        sock.send(REQUEST_SINGLE_SCAN)

        scan = []
        curr = ''
        while 1:
            msg_orig = sock.recv(1)
            msg = msg_orig.decode('utf-8')
            if msg_orig == b'\x02':
                insideTelegram = True 
            elif msg_orig == b'\x03':
                insideTelegram = False 
                break
            elif msg == ' ':
                scan.append(curr)
                curr = ''
            else:
                curr += msg
        
        curr = datetime.now()
        nice_timestamp = str(curr.year) + "-" + str(curr.month) + "-" + str(curr.day) + "_" + str(curr.hour) + "-" + str(curr.minute) + "-" + str(curr.second)

        scan.append(nice_timestamp)

        initial_parse = telegram_parse(scan)
        read_serial = ser.readline()
        return initial_parse

    def uploadToAWS(self, telegram):

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

    def accel_read(self):
        """Reads data from the MPU-6050 module  

            Args:
                None
            Return:
                Acceleration and Gyroscope data in 
                all 3 axes. Acceleration is in m/s, 
                gyro is in degrees/s
        """
        acc_x = self.read_raw_data(s.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(s.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(s.ACCEL_ZOUT_H)

        gyro_x = self.read_raw_data(s.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(s.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(s.GYRO_ZOUT_H)

        Ax = acc_x / s.ACCEL_CONSTANT
        Ay = acc_y / s.ACCEL_CONSTANT
        Az = acc_z / s.ACCEL_CONSTANT

        Gx = gyro_x / s.GYRO_CONSTANT
        Gy = gyro_y / s.GYRO_CONSTANT
        Gz = gyro_z / s.GYRO_CONSTANT
        
        return Ax, Ay, Az, Gx, Gy, Gz

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
        self.A = self.accel_read()
        self.x = kalman.Gravity([[self.A[0]], [self.A[1]], [self.A[2]]])

sentinel = SENTINEL()
count = 0
prevTime = time.time() 
actualTime = time.time()
P = np.eye(3)

Qk = np.diag([100, 100, 100])
Rk = np.diag([1, 1, 1])

while True:
    arduinoReply = s.recvLikeArduino()
    A = sentinel.accel_read()
    kalman.Predict(sentinel.x , P, [[A[3]], [A[4]], [A[5]]], time.time() - actualTime, Qk)
    actualTime = time.time()

    print(sentinel.singleScanPretty())

    if not (arduinoReply == 'XXX'):
        yeet = arduinoReply.split(" ")
        yeet = yeet[-2]
        curr = sentinel.single_parse()
        curr['Motor encoder'] = yeet
        print(curr)


    if time.time() - prevTime > 1.0:
        s.sendToArduino('g')
        prevTime = time.time()
        count += 1
