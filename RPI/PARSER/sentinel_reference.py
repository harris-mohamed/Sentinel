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

# AWS CONSTANTS  
DB = 'dynamodb'
REGION_NAME = 'us-east-2'
ENDPOINT_URL = 'http://dynamodb.us-east-2.amazonaws.com'
TABLE_NAME = 'Sentinel'

# SERIAL BUS CONSTANTS 
startSentinel = '<'
endSentinel = '>'
dataStarted = False 
dataBuf = ""
messageComplete = False 


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

def uploadToAWS(telegram):
    """ Uploads a scan to the AWS database

        Args:
            A parsed telegram from the sensor 
        Return:
            None
    """
    response = table.put_item(
        Item={
            'Time-stamp': str(telegram['Timestamp']),
            'Version Number': str(telegram['Version Number']),
            'Device Number': str(telegram['Device Number']),
            'Serial Number': str(telegram['Serial Number']),
            'Device Status': str(telegram['Device Status']),
            'Telegram Counter': str(telegram['Telegram Counter']),
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
            #'Motor encoder': str(telegram['Motor encoder']),
            'Timestamp': str(telegram['Timestamp']),
            'Ax': str(telegram['Ax']),
            'Ay': str(telegram['Ay']),
            'Az': str(telegram['Az']),
            'Gx': str(telegram['Gx']),
            'Gy': str(telegram['Gy']),
            'Gz': str(telegram['Gz']),
            'Measurement': str(telegram['Measurement'])
        }
    )