# Libraries
from __future__ import print_function
from datetime import datetime
import boto3
import json
from boto3.dynamodb.conditions import Key, Attr
import decimal

# Database connection
dynamodb = boto3.resource('dynamodb', region_name='us-east-2',
                          endpoint_url="http://dynamodb.us-east-2.amazonaws.com")
table = dynamodb.Table('Sentinel')

# Some defaults for testing 
now = datetime.now()
sampleCustomer = "Harris"
sampleTimestamp = str(now)
# print(sampleTimestamp)

def createItem():
    response = table.put_item(
        Item={
        'Time-stamp': sampleTimestamp,
    }
    )


def readItem():
    try:
        response = table.get_item(
            Key={
                'Customer': sampleCustomer,
                'Timestamp': sampleTimestamp
            }
        )
    except ClientError as e:
        print(e.response['Error']['Message'])
    else:
        item = response['Item']
        print(item)


def updateItem():
    response = table.update_item(
        Key={
            'Customer': sampleCustomer,
            'Timestamp': sampleTimestamp
        },
        UpdateExpression="set Frequencies = :r",
        ExpressionAttributeValues={
            ':r': {10, 11, 12, 13}
        }
    )

def deleteItem():
    try:
        response = table.delete_item(
            Key={
                'Customer': sampleCustomer,
                'Timestamp': sampleTimestamp
            }
        )
    except ClientError as e:
        if e.response['Error']['Code'] == "ConditionalCheckFailedException":
            print(e.response['Error']['Message'])
        else:
            raise

    else:
        print("Delete succeeded!")

def conditionalQuery():
    response = table.query(
        ProjectionExpression="#c, #t, Frequencies",
        ExpressionAttributeNames={ "#c": "Customer" }, # Expression Attribute Names for Projection Expression only.
        KeyConditionExpression=Key('Customer') & Key('Frequencies').between('081519191030', '081519191430'))
    for i in response[u'Items']:
        print(json.dumps(i, cls=DecimalEncoder))
    
createItem()