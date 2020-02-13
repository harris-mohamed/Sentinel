# Project Sentinel: The official Sentinel class
# Harris M 
# February 5, 2020

from time import sleep 
from datetime import datetime
import sys
import socket
import time
import struct

from __future__ import print_function 
from datetime import datetime 
from boto3.dynamodb.conditions import Key, Attr
import boto3
import json 
import decimal

import smbus 
import serial 
import sentinel_reference as ref

sys.path.append('../../SLAM/RANSAC')

import RANSAC as ransac

Class SENTINEL:
    def __init__(self):
        bus = smbus.SMBus(3)
        dynamodb = boto3.resource(ref.DB, region_name=ref.REGION_NAME, endpoint_url=ref.ENDPOINT_URL)
        table = dynamodb.Table(ref.TABLE_NAME)
        ser = serial.Serial(ref.ARDUINO_PORT, ref.ARDUINO_BAUD)


