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

""" Function declarations for the SENTINEL class """

bus = smbus.SMBus(3)
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((s.IP_ADDRESS, s.PORT)) 

global startSentinel, endSentinel, serialPort, dataStarted, dataBuf, messageComplete

global serialPort 

def accel_init():
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

def singleScan():
    """Takes a single scan

    Args:
        None
    Return:
        The single scan
    """
    return s.single_parse() 

def contScan(count):
    """Takes a specified number of consecutive scans

    Args:
        The number of consecutive scans to take
    Return:
        A list of dictionaries, each dictionary contains a parsed scan 
    """ 
    return s.live_parse(count)

def singleScanPretty():
    """Takes a single scan and prints it for debugging

    Args:
        None
    Return:
        The single scan
    """
    current_scan = s.single_parse()
    print(current_scan)
    return current_scan 

def setupSerial(baudRate=115200, serialPortName=s.ARDUINO_PORT):
    """Instantiate serial port with Arduino

        Args:
            baudRate (int): Baud rate for serial connection
            serialPortName (string): Specifies which serial port to connect to
        Return:
            None
    """
    global serialPort
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)
    print("Serial port " + serialPortName + " opened Baudrate " + str(baudRate))
    self.waitForArduino()


def waitForArduino():
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

def recvLikeArduino():
    """Handles received serial messages on the serial port

        Args:
            None
        Return:
            Either the data buffer, or XXX to indicate failure
    """
    global startSentinel, endSentinel, serialPort, dataStarted, dataBuf, messageComplete
    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") 

        if dataStarted == True:
            if x != endSentinel:
                dataBuf = dataBuf + x 
            else:
                dataStarted = False 
                messageComplete = True 
        elif x == startSentinel:
            dataBuf = ''
            dataStarted = True 

    if (messageComplete == True):
        messageComplete = False 
        return dataBuf 
    else: 
        return "XXX"

def sendToArduino(stringToSend):
    """Sends a properly encoded string to the Arduino

        Args:
            The string we want to send
        Return:
            None
    """
    global startSentinel, endSentinel, serialPort 

    stringWithMarkers = (startSentinel)
    stringWithMarkers += stringToSend 
    stringWithMarkers += (endSentinel)

def singleScanWithUpload():
    """Takes a single scan and uploads it to AWS

    Args:
        None
    Return:
        The single scan
    """

    current_scan = s.single_parse() 
    s.uploadToAws(current_scan)
    return current_scan

def type_conv(num, base):
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


def telegram_parse(scan):
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
        telegram['Time since start-up'] = type_conv(scan[9], 'u32') * s.MICROSECOND
        telegram['Time of transmission'] = type_conv(scan[10], 'u32') * s.MICROSECOND
        telegram['Scan Frequency'] = type_conv(scan[16], 'u32') 
        telegram['Measurement Frequency'] = type_conv(scan[17], 'u32') 
        telegram['Amount of Encoder'] = type_conv(scan[18], 'u32')
        telegram['16-bit Channels'] = type_conv(scan[19], 'u32')
        telegram['Scale Factor'] = s.SCALE_FACTOR[scan[21]]
        telegram['Scale Factor Offset'] = type_conv(scan[22], 'u32')
        telegram['Start Angle'] = type_conv(scan[23], 's32') / s.ANGLE_STOP
        telegram['Angular Increment'] = type_conv(scan[24], 'u16') / s.ANGLE_STOP
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

def live_parse(count):
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

def single_parse():
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
    # read_serial = ser.readline()
    return initial_parse

def uploadToAWS(telegram):

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

    
def read_freq_angle():
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

def load_factory_defaults():
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
    
def reboot_device():
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

def save_param_permanent():
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
    
def read_raw_data(addr):
    """Instantiates the MPU-6050 module 

        Args:
            None
        Return:
            None
    """
    high = bus.read_byte_data(ACCEL_ADDRESS, addr)
    low = bus.read_byte_data(ACCEL_ADDRESS, addr + 1)

    value = ((high << 8) | low)

    if (value > 32768):
        value = value - 65536
    
    return value         

def accel_read():
    """Reads data from the MPU-6050 module  

        Args:
            None
        Return:
            Acceleration and Gyroscope data in 
            all 3 axes. Acceleration is in m/s, 
            gyro is in degrees/s
    """
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    Ax = acc_x / ACCEL_CONSTANT
    Ay = acc_y / ACCEL_CONSTANT
    Az = acc_z / ACCEL_CONSTANT

    Gx = gyro_x / GYRO_CONSTANT
    Gy = gyro_y / GYRO_CONSTANT
    Gz = gyro_z / GYRO_CONSTANT
    
    return Ax, Ay, Az, Gx, Gy, Gz

def manualControl(): 
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


dynamodb = boto3.resource(s.DB, region_name=s.REGION_NAME, endpoint_url=s.ENDPOINT_URL)
table = dynamodb.Table(s.TABLE_NAME)
setupSerial()
accel_init()
A = accel_read()
x = kalman.Gravity([[A[0]], [A[1]], [A[2]]])

sentinel = SENTINEL()
count = 0
prevTime = time.time() 
actualTime = time.time()
P = np.eye(3)

Qk = np.diag([100, 100, 100])
Rk = np.diag([1, 1, 1])

while True:
    arduinoReply = recvLikeArduino()
    A = accel_read()
    kalman.Predict(x , P, [[A[3]], [A[4]], [A[5]]], time.time() - actualTime, Qk)
    actualTime = time.time()
    print(arduinoReply)
    # print(sentinel.single_parse())

    if not (arduinoReply == 'XXX'):
        yeet = arduinoReply.split(" ")
        yeet = yeet[-2]
        curr = single_parse()
        curr['Motor encoder'] = yeet
        print(curr)


    if time.time() - prevTime > 1.0:
        sendToArduino('g')
        prevTime = time.time()
        count += 1
