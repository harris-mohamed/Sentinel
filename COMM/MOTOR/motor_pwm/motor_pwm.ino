
int ledPin = 13;    // LED connected to digital pin 9
int forwardPin = 6; 
int reversePin = 5;

void setup() {
  // nothing happens in setup
}

void loop() {
   analogWrite(forwardPin, 0);
   analogWrite(reversePin, 130);
//  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
//    // sets the value (range from 0 to 255):
//    analogWrite(motorPin, fadeValue);
//    // wait for 30 milliseconds to see the dimming effect
//    delay(30);
//  }
//
//  // fade out from max to min in increments of 5 points:
//  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
//    // sets the value (range from 0 to 255):
//    analogWrite(motorPin, fadeValue);
//    // wait for 30 milliseconds to see the dimming effect
//    delay(30);
//  }
}
