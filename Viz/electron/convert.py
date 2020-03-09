# Utility functions to convert an AWS dataset into the proper format to be visualized
# Author: Harris Mohamed 

from __future__ import print_function # Python 2/3 compatibility
import boto3
import json
import decimal
from boto3.dynamodb.conditions import Key, Attr

dynamodb = boto3.resource('dynamodb', region_name='us-east-2', endpoint_url='http://dynamodb.us-east-2.amazonaws.com')
table = dynamodb.Table('SENTINEL')

def readFromAWS(name):
        """ Grabs all scans associated with a certain name

            Args:
                Name we are interested in 
            Return:
                A list of dictionaries 
        """

        output = []

        response = table.query(KeyConditionExpression=Key('Name').eq(name))

        for scan in response['Items']:
            output.append(scan)

        return output

# Idk what these lines do 
if __name__=="__main__":
    output = readFromAWS('hallway_scan_3-8-2020')
    print(output)

# scan_kalman = readFromAWS('hallway_scan_3-8-2020')

