# Project Sentinel: The official Sentinel class
# Harris M 
# March 2, 2020

# AWS library imports 
from __future__ import print_function 
from datetime import datetime 
from boto3.dynamodb.conditions import Key, Attr
import boto3
import json 
import decimal

# Standard library imports
from time import sleep 
from datetime import datetime
import sys
import socket
import time
import struct
from numpy import array
import numpy as np

# RPI specific imports
import smbus 
import serial 
import sentinel_reference as s

sys.path.append("../SLAM")

import RANSAC.RANSAC as ransac
import KALMAN as kalman

class SENTINEL:
    """ Function declarations for the SENTINEL class """
      
    def __init__(self):
        """Initializes neccesary constants and communication busses

        Args:
            None
        Return:
            None
        """
        self.dynamodb = boto3.resource(s.DB, region_name=s.REGION_NAME, endpoint_url=s.ENDPOINT_URL)
        self.table = self.dynamodb.Table(s.TABLE_NAME)
        self.dataStarted = False 
        self.dataBuf = ""
        self.messageComplete = False
        self.setupSerial()
        self.bus = smbus.SMBus(3)
        self.accel_init()
        self.A = self.accel_read()
        self.x = kalman.Gravity([[self.A[0]], [self.A[1]], [self.A[2]]])

    def setupSerial(self, baudRate=115200, serialPortName=s.ARDUINO_PORT):
        """Instantiate serial port with Arduino

            Args:
                baudRate (int): Baud rate for serial connection
                serialPortName (string): Specifies which serial port to connect to
            Return:
                None
        """
        # global serialPort
        self.serialPort = serial.Serial(port=serialPortName, baudrate=baudRate, timeout=0, rtscts=True)
        print("Serial port " + serialPortName + " opened Baudrate " + str(baudRate))
        self.waitForArduino()

    def waitForArduino(self):
        """Waits for Arduino to connect to Raspberry Pi

            Args:
                None
            Return:
                None
        """
        print("Waiting for Arduino to reset")
        msg = ""
        while msg.find("Arduino is ready") == -1:
            msg = self.recvLikeArduino() 
            if not (msg == 'XXX'):
                print(msg)

    def recvLikeArduino(self):
        """Handles received serial messages on the serial port

            Args:
                None
            Return:
                Either the data buffer, or XXX to indicate failure
        """

        if self.serialPort.inWaiting() > 0 and self.messageComplete == False:
            x = self.serialPort.read().decode("utf-8") 
        
            if self.dataStarted == True:
                if x != s.endSentinel:
                    self.dataBuf = self.dataBuf + x 
                else:
                    self.dataStarted = False 
                    self.messageComplete = True 
            elif x == s.startSentinel:
                self.dataBuf = ''
                self.dataStarted = True 

        if (self.messageComplete == True):
            self.messageComplete = False 
            return self.dataBuf 
        else: 
            return "XXX"

    def sendToArduino(self, stringToSend):
        """Sends a properly encoded string to the Arduino

            Args:
                The string we want to send
            Return:
                None
        """

        stringWithMarkers = (s.startSentinel)
        stringWithMarkers += stringToSend 
        stringWithMarkers += (s.endSentinel)

        self.serialPort.write(stringWithMarkers.encode('utf-8'))

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

    def singleScanPretty(self):
        """Takes a single scan and prints it for debugging

        Args:
            None
        Return:
            The single scan
        """
        current_scan = self.single_parse()
        print(current_scan)
        return current_scan 

    def singleScanWithUpload(self):
        """Takes a single scan and uploads it to AWS

        Args:
            None
        Return:
            The single scan
        """

        current_scan = self.single_parse()
        self.uploadToAws(current_scan)
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
            telegram['Version Number'] = self.type_conv(scan[2], 'u16')
            telegram['Device Number'] = self.type_conv(scan[3], 'u16')
            telegram['Serial Number'] = self.type_conv(scan[4], 'u32')
            telegram['Device Status'] = self.type_conv(scan[6], 'u8')
            telegram['Telegram Counter'] = self.type_conv(scan[7], 'u16')
            telegram['Scan Counter'] = self.type_conv(scan[8], 'u16')
            telegram['Time since start-up'] = self.type_conv(scan[9], 'u32') * s.MICROSECOND
            telegram['Time of transmission'] = self.type_conv(scan[10], 'u32') * s.MICROSECOND
            telegram['Scan Frequency'] = self.type_conv(scan[16], 'u32') 
            telegram['Measurement Frequency'] = self.type_conv(scan[17], 'u32') 
            telegram['Amount of Encoder'] = self.type_conv(scan[18], 'u32')
            telegram['16-bit Channels'] = self.type_conv(scan[19], 'u32')
            telegram['Scale Factor'] = s.SCALE_FACTOR[scan[21]]
            telegram['Scale Factor Offset'] = self.type_conv(scan[22], 'u32')
            telegram['Start Angle'] = self.type_conv(scan[23], 's32') / s.ANGLE_STOP
            telegram['Angular Increment'] = self.type_conv(scan[24], 'u16') / s.ANGLE_STOP
            telegram['Quantity'] = self.type_conv(scan[25], 'u16')

            for message in range(26, 26 + telegram['Quantity']):
                telegram['Measurement'].append(self.type_conv(scan[message], 'u16'))
            
            telegram['Timestamp'] = scan[-1]
            
            Ax, Ay, Az, Gx, Gy, Gz = self.accel_read()
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
        sock.connect((s.IP_ADDRESS, s.PORT))
        sock.send(s.REQUEST_CONT_SCAN)

        self.P = np.eye(3)
        self.Qk = np.diag([s.QK_VAL, s.QK_VAL, s.QK_VAL])
        self.Rk = np.diag([s.RK_VAL, s.RK_VAL, s.RK_VAL])

        self.sendToArduino('g') #Tell Arduino to start spinning

        time.sleep(20)
        actualTime = time.time()
        
        scans = []
        while 1:
            scan = []
            curr = ''
            while 1:
                startTime = time.time()
                msg_orig = sock.recv(1)
                msg = msg_orig.decode('utf-8')  
                actualTime = time.time()

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
            
            currentTrack = (time.time() - startTime) 
            startTime = time.time()

            curr = datetime.now()
            nice_timestamp = str(curr.year) + "-" + str(curr.month) + "-" + str(curr.day) + "_" + str(curr.hour) + "-" + str(curr.minute) + "-" + str(curr.second)

            scan.append(nice_timestamp)

            initial_parse = self.telegram_parse(scan)

            actualTime = time.time()
            self.A = self.accel_read()
            self.x, self.P = kalman.Predict(self.x , self.P, [[np.deg2rad(self.A[3])], [np.deg2rad(self.A[4])], [np.deg2rad(self.A[5])]], time.time() - actualTime, self.Qk) #This line should read the gyroscope while the motor is spinning
            self.x, self.P = kalman.Correct(self.x , self.P, [[self.A[0]], [self.A[1]], [self.A[2]]], self.Rk) #When the scan is about to be taken, this line should be executed.
            initial_parse['Rk'] = self.Rk
            initial_parse['Qk'] = self.Qk 
            initial_parse['P'] = self.P
            initial_parse['euler'] = self.x
            initial_parse['Motor encoder'] = 0

            scans.append(initial_parse)
            currentTrack = (time.time() - startTime)

            print(len(scans)) 
            if len(scans) == count:
                sock.send(s.STOP_CONT_SCAN)
                self.sendToArduino('s') #Tell Arduino to stop
                scan_name = input("What is the scan name?\n")
                for scan in scans:
                    sentinel.uploadToAWS(scan, scan_name)

                # Update the list of scan names
                currentNameHold = self.readFromAWS('NAMES')
                currentNames = currentNameHold[0]['list']
                self.deleteFromAWS('NAMES', '-1')
                currentNames.append(scan_name)
                break

        return(scans)
    
    def replaceAWSName(self, names):
        """ Specialized upload to AWS function that updates names 

            Args:
                names: Array of strings to update names with
            Return:
                None 
        """
        response = self.table.put_item(
            Item={
                'Name': 'NAMES',
                'Count': '-1', 
                'list': str(names)
            }
        )

    def deleteFromAWS(self, name, value):
        """ Deletes an entry from the database

            Args:
                name: Name of the entry to delete
                value: Value of the entry to delete 
            Return:
                None 
        """
        response = self.table.delete_item(
            Key = {
                'Name': name, 
                'Count': value
            }
        )

    def single_parse(self):
        """ Parses a single scan from the sensor

            Args:
                None 
            Return:
                Dictionary of parsed values
        """
        insideTelegram = False 
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((s.IP_ADDRESS, s.PORT))
        sock.send(s.REQUEST_SINGLE_SCAN)

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

        initial_parse = self.telegram_parse(scan)
        return initial_parse

    def uploadToAWS(self, telegram, name):
        """ Uploads a scan to the AWS database

            Args:
                A parsed telegram from the sensor 
            Return:
                None
        """
        if name == "":
            name = str("Unnamed_" + telegram['Timestamp'])
        
        response = self.table.put_item(
            Item={
                # 'Time-stamp': str(telegram['Timestamp']),
                'Name': name,
                'Count': str(telegram['Telegram Counter']),
                'Version Number': str(telegram['Version Number']),
                'Device Number': str(telegram['Device Number']),
                'Serial Number': str(telegram['Serial Number']),
                'Device Status': str(telegram['Device Status']),
                # 'Telegram Counter': str(telegram['Telegram Counter']),
                'Scan Counter': str(telegram['Scan Counter']),
                'Time since start-up': str(telegram['Time since start-up']),
                'Time of transmission': str(telegram['Time of transmission']),
                'Scan Frequency': str(telegram['Scan Frequency']),
                'Measurement Frequency': str(telegram['Measurement Frequency']),
                'Amount of Encoder': str(telegram['Amount of Encoder']),
                '16-bit Channels': str(telegram['16-bit Channels']),
                'Scale Factor': str(telegram['Scale Factor']),
                'Scale Factor Offset': str(telegram['Scale Factor Offset']),
                'Start Angle': str(telegram['Start Angle']),
                'Angular Increment': str(telegram['Angular Increment']),
                'Quantity': str(telegram['Quantity']),
                'Motor encoder': telegram['Motor encoder'],
                'Timestamp': str(telegram['Timestamp']),
                'Rk': (telegram['Rk']).__repr__(),
                'Qk': (telegram['Qk']).__repr__(), 
                'P':  (telegram['P']).__repr__(),
                'euler': (telegram['euler']).__repr__(),
                'Ax': str(telegram['Ax']),
                'Ay': str(telegram['Ay']),
                'Az': str(telegram['Az']),
                'Gx': str(telegram['Gx']),
                'Gy': str(telegram['Gy']),
                'Gz': str(telegram['Gz']),
                'Measurement': str(telegram['Measurement'])
            }
        )

    def readFromAWS(self, name):
        """ Grabs all scans associated with a certain name

            Args:
                Name we are interested in 
            Return:
                A list of dictionaries 
        """

        output = []

        response = self.table.query(KeyConditionExpression=Key('Name').eq(name))

        for scan in response['Items']:
            output.append(scan)

        return output 

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
            scan_frequency = self.type_conv(scan[2], 'u32')
            sector_count = self.type_conv(scan[3], 'u16')
            angle_resolution = self.type_conv(scan[4], 'u32')
            start_angle = self.type_conv(scan[5], 's32')
            stop_angle = self.type_conv(scan[6], 'u32')

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
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((s.IP_ADDRESS, s.PORT))
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
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((s.IP_ADDRESS, s.PORT))
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

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((s.IP_ADDRESS, s.PORT))
        sock.send(s.SAVE_PARAMETERS_PERMANENT)
        scan = message_collect(sock)
        
        if (len(scan) != 3) or (scan[1] != 'mEEwriteall'):
            print("Something went wrong with the save parameters command")
        else: 
            print("The parameters were written successfully.")

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

    def stopAndGo(self, count):
        """Runs the stop and go algorithm of Sentinel

        Args:
            count: How many scans we want
        Return:
            None
        """

        scans = []
        actualTime = time.time() 

        self.P = np.eye(3)
        self.Qk = np.diag([s.QK_VAL, s.QK_VAL, s.QK_VAL])
        self.Rk = np.diag([1, 1, 1])
        self.counter = 0
        self.sendToArduino('g')
        
        while True:
            self.A = self.accel_read()
            self.x, self.P = kalman.Predict(self.x , self.P, [[np.deg2rad(self.A[3])], [np.deg2rad(self.A[4])], [np.deg2rad(self.A[5])]], time.time() - actualTime, self.Qk) #This line should read the gyroscope while the motor is spinning
            actualTime = time.time()
            
            arduinoReply = self.recvLikeArduino()
            parse = arduinoReply.split(" ")

            if len(parse) != 1 and parse[0] == 'D':
                time.sleep(0.3)
                actualTime = time.time()
                self.counter = self.counter + 1 
                scan = self.single_parse()
                scan['Motor encoder'] = parse[1]
                # theta_motor = (np.pi * (float(scan['Motor encoder']))) / 180.00 #This line needs to be the value from the motor encoder that the arduino sends to the RPi.
                self.x, self.P = kalman.Correct(self.x , self.P, [[self.A[0]], [self.A[1]], [self.A[2]]], self.Rk) #When the scan is about to be taken, this line should be executed.
                scan['Rk'] = self.Rk
                scan['Qk'] = self.Qk 
                scan['P'] = self.P
                scan['euler'] = self.x
                scans.append(scan)
                self.sendToArduino('g')
                print("Took a scan!")
                if (self.counter == count):
                    # Update the list of scan names
                    # currentNameHold = self.readFromAWS('NAMES')
                    # currentNames = currentNameHold[0]['list']
                    # self.deleteFromAWS('NAMES', '-1')
                    # currentNames.append(scan_name)
                    # self.replaceAWSName(scan_name)
                    scan_name = input("Enter the name for the scan: ")
                    self.replaceAWSName(scan_name)
                    # Upload the AWS scans here 
                    for scan in scans: 
                        self.uploadToAWS(scan, scan_name)
                    break


    def contbutnot(self, count):
        """Runs the stop and go algorithm of Sentinel

        Args:
            count: How many scans we want
        Return:
            None
        """

        scans = []
        actualTime = time.time() 
        OuterloopTime = time.time()
        dt = 0.5
        stopcount = 10 #After every "stopcount" scans, stop the motor to correct for drift using kalman.gravity. Maybe this could change according to the count given.
        self.P = np.eye(3)
        self.Qk = np.diag([s.QK_VAL, s.QK_VAL, s.QK_VAL])
        self.Rk = np.diag([s.RK_VAL, s.RK_VAL, s.RK_VAL])
        self.counter = 0
        self.subcounter = 0
        self.sendToArduino('g')
        
        while True:
            self.A = self.accel_read()
            self.x = kalman.GyroIntegrate(self.x ,[[np.deg2rad(self.A[3])], [np.deg2rad(self.A[4])], [np.deg2rad(self.A[5])]], time.time() - OuterloopTime) #This line should read the gyroscope while the motor is spinning
            OuterloopTime = time.time()
            
            # arduinoReply = self.recvLikeArduino()
            # parse = arduinoReply.split(" ")
            if time.time() - actualTime > dt:
                # time.sleep(0.3)
                actualTime = time.time()
                self.counter = self.counter + 1
                self.subcounter = self.subcounter + 1
                scan = self.single_parse()
                print("Took a scan!")
                scan['Motor encoder'] = 0
                # theta_motor = (np.pi * (float(scan['Motor encoder']))) / 180.00 #This line needs to be the value from the motor encoder that the arduino sends to the RPi.
                scan['Rk'] = self.Rk
                scan['Qk'] = self.Qk 
                scan['P'] = self.P

                #self.sendToArduino('g')
                #print("Took a scan!")
              #  if self.subcounter >= stopcount:
              #      print("Stopping Arduino!")
              #      self.sendToArduino('s')
              #      time.sleep(1.0)
              #      self.A = self.accel_read()
              #      self.x = kalman.Gravity([[self.A[0]], [self.A[1]], [self.A[2]]])
              #      self.sendToArduino('g')
              #      print("Telling Arduino to go!")
              #      self.subcounter = 0

                #I placed the if statement before to correct the orientation when necessary.    
                scan['euler'] = self.x
                scans.append(scan)
                if (self.counter == count):
                    # Update the list of scan names
                    # currentNameHold = self.readFromAWS('NAMES')
                    # currentNames = currentNameHold[0]['list']
                    # self.deleteFromAWS('NAMES', '-1')
                    # currentNames.append(scan_name)
                    # self.replaceAWSName(scan_name)
                    self.sendToArduino('s')
                    print("About to ask for input")
                    scan_name = input("Enter the name for the scan: ")
                    self.replaceAWSName(scan_name)
                    # Upload the AWS scans here 
                    for scan in scans: 
                        self.uploadToAWS(scan, scan_name)
                    break

sentinel = SENTINEL()
# scans = sentinel.live_parse(60)
# scans = sentinel.stopAndGo(20)
scans = sentinel.contbutnot(40)

