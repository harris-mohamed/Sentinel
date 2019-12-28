/* ----------------------------------------------------------
       SENTINEL - MOTOR ENCODER SCRIPT

       Harris M

       December 28, 2019
   ---------------------------------------------------------- */

/* ----------------------------------------------------------
       LIBRARIES
   ---------------------------------------------------------- */
#include <Encoder.h>

/* ----------------------------------------------------------
       PIN
   ---------------------------------------------------------- */
#define LIVE 7

/* ----------------------------------------------------------
       GLOBAL ARRAYS/Variables
   ---------------------------------------------------------- */
long oldPosition  = -999;

/* ----------------------------------------------------------
       ENCODER INSTANTIATION
   ---------------------------------------------------------- */
Encoder myEnc(2, 3);    // Pins 2 and 3 are interrupt pins 

void setup() {
    pinMode(LIVE, OUTPUT);
    
    /* ----- UART INIT ----- */
    digitalWrite(LIVE, LOW);
    Serial.begin(115200, SERIAL_8N1);
    Serial.setTimeout(UART_TIMEOUT);
    digitalWrite(LIVE, HIGH);
    delay(1000);
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
}

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Raw: ");
    Serial.print(newPosition);
    Serial.print(" Revolutions: ");
    Serial.print(newPosition/540.00);
    Serial.print(" Degrees: ");
    Serial.println((newPosition/540.00)*180.00);
  }
}