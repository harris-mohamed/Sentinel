#!/bin/bash

# Arduino flash script 
# This script aliases the rather lengthy commands we need to program the Arduino 
# Author: Harris Mohamed 

arduino --board arduino:avr:mega:cpu=atmega2560 --port /dev/ttyACM0 --upload $1 