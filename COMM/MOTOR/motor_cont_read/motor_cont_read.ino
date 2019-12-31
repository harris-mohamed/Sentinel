/* ----------------------------------------------------------
       SENTINEL - MOTOR ENCODER SCRIPT w/Circular Correction

       Harris M

       December 30, 2019
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
double rev_count;
double deg;
const double rev = 360.00;
const double PPR = 1080.00;  // This is a constant given by the manufacturer. Amazon says this should be 540, but our own testing revealed it is 1080.

/* ----------------------------------------------------------
       ENCODER INSTANTIATION
   ---------------------------------------------------------- */
Encoder myEnc(2, 3);    // Pins 2 and 3 are interrupt pins 

void setup() {
    pinMode(LIVE, OUTPUT);
    
    /* ----- UART INIT ----- */
    digitalWrite(LIVE, LOW);
    Serial.begin(115200, SERIAL_8N1);
    digitalWrite(LIVE, HIGH);
    delay(1000);
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
}

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    rev_count = newPosition / 1080.00;
    deg = rev_count * rev;

    if (deg > rev){
      deg -= 360;
    }

  Serial.print("Revolutions: ");
  Serial.print(rev_count);
  Serial.print(" Degrees: ");
  Serial.println(deg);
  }
  
}
