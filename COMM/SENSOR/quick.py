import sys 
import ast
import json 

with open("2020-1-9_19-41-58.txt", "r") as file:
    x = file.readlines()
     
test = ast.literal_eval(x[0])
y = test['Measurement']

print(type(test['Measurement'][0]))
