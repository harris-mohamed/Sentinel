import serial 
import time 

startMarker = '<'
endMarker = '>'
dataStarted = False 
dataBuf = ""
messageComplete = False 

def setupSerial(baudRate, serialPortName):
    global serialPort 
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)
    print("Serial port " + serialPortName + " opened Baudrate " + str(baudRate))
    waitForArduino()

def sendToArduino(stringToSend):
    global startMarker, endMarker, serialPort 

    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend 
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8'))

def recvLikeArduino():
    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete
    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") 

        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x 
            else:
                dataStarted = False 
                messageComplete = True 
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True 

    if (messageComplete == True):
        messageComplete = False 
        return dataBuf 
    else: 
        return "XXX"

def waitForArduino():
    print("Waiting for Arduino to reset")
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino() 
        if not (msg == 'XXX'):
            print(msg)

setupSerial(115200, "/dev/ttyACM0")
count = 0
prevTime = time.time() 
while True:
    arduinoReply = recvLikeArduino() 
    if not (arduinoReply == 'XXX'):
        print("Time %s Reply %s" %(time.time(), arduinoReply))

    if time.time() - prevTime > 1.0:
        # sendToArduino("this is a test " + str(count))
        sendToArduino("g")
        prevTime = time.time()
        count += 1
