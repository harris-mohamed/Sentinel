/* ----------------------------------------------------------
       SENTINEL - MOTOR ENCODER SCRIPT w/I2C output

       Harris M

       January 9th, 2020
   ---------------------------------------------------------- */

/* ----------------------------------------------------------
       LIBRARIES
   ---------------------------------------------------------- */
#include<Encoder.h>
#include<Wire.h> 

/* ----------------------------------------------------------
       PIN
   ---------------------------------------------------------- */
#define SLAVE_ADDRESS 0x04 
#define LIVE 13

/* ----------------------------------------------------------
       GLOBAL ARRAYS/Variables
   ---------------------------------------------------------- */
long oldPosition  = -999;
double deg;
const double rev = 360.00;
const double PPR = 1080.00;  // This is a constant given by the manufacturer. Amazon says this should be 540, but our own testing revealed it is 1080.
int number = 0;

/* ----------------------------------------------------------
       ENCODER INSTANTIATION
   ---------------------------------------------------------- */
Encoder myEnc(2, 3);    // Pins 2 and 3 are interrupt pins 

void setup() {
  pinMode(LIVE, OUTPUT); 
  
  /* ----- UART INIT ----- */
  digitalWrite(LIVE, LOW);
  Serial.begin(9600, SERIAL_8N1);
  digitalWrite(LIVE, HIGH);
  delay(1000);

  /* ----- I2C INIT ----- */  
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
    if (number == 1) {
      sendDataCallback();  
    }
    }
  }

void sendDataCallback(){
  Wire.write(11); 
  }
