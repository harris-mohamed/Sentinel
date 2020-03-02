from __future__ import print_function # Python 2/3 compatibility
import boto3
import json
import decimal
from boto3.dynamodb.conditions import Key, Attr

dynamodb = boto3.resource('dynamodb', region_name='us-east-2', endpoint_url='http://dynamodb.us-east-2.amazonaws.com')
table = dynamodb.Table('SENTINEL')

response = table.put_item(
        Item={
        'Name': 'Scan 1',
        'Count': '0'
    }
    )



# response = table.query(
#     KeyConditionExpression=Key('Time-stamp').eq('Yeet')
# )


# for i in response['Items']:
#     print(type(i))