#include<Wire.h> 

#define SLAVE_ADDRESS 0x04 

int number = 0;

void setup() {
  pinMode(13, OUTPUT); 
  Serial.begin(9600); 
  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(receiveDataCallback);
  Wire.onRequest(sendDataCallback);

  Serial.println("Initialization complete!");
}

void loop() {
  delay(100);
}

void receiveDataCallback(int byteCount){
  while(Wire.available()){
    number = Wire.read();
    Serial.print("Received: ");
    Serial.println(number);
    }
  }

void sendDataCallback(){
  number++;
  Wire.write(number); 
  }
