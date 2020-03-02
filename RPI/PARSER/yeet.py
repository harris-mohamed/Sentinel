# This script enables serial communication between the Raspberry Pi and the Arduino.
# It is running on the Raspberry Pi

setupSerial(115200, "/dev/ttyACM0") # Helper functions instantiates Serial bus 
count = 0
prevTime = time.time()      
while True:
    arduinoReply = recvLikeArduino()   # This helper function returns XXX if the Raspberry Pi is not done parsing, and returns an array of strings if it is done parsing    
    if not (arduinoReply == 'XXX'): 
        print ("Time %s  Reply %s" %(time.time(), arduinoReply))    
        
    if time.time() - prevTime > 1.0:
        sendToArduino("this is a test " + str(count))    # Helper function sends a string to the Arduino
        prevTime = time.time()
        count += 1