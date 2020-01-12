#include <Encoder.h>
#include <PID_v1.h>

#define A 2
#define B 3
#define forwardPin 5 
#define reversePin 6

double Setpoint, actual_rpm, Output;
long oldPosition  = -999;
double deg;
const double rev = 360.00;
const double PPR = 1080.00;  // This is a constant given by the manufacturer. Amazon says this should be 540, but our own testing revealed it is 1080.
unsigned long delta_t;
double rev_count;
const double Kp = 0.001; //tuning parameter for "error-proportional" term.
const double Ki = 0.001; //tuning parameter for "error-integral" term.
const double Kd = 0.001; //tuning parameter for "error-derivative" term. 


PID myPID(&actual_rpm, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder myEnc(A, B);

void setup() {
  Setpoint = 130;
  actual_rpm = 0;

  myPID.SetMode(AUTOMATIC); //start the PID
}

void loop() {
  unsigned long starttime = millis();
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    unsigned long stoptime = millis();
    delta_t = (stoptime-starttime)/60000.0;
    rev_count = newPosition / 1080.00;
    deg = rev_count * rev;
    actual_rpm = rev_count/delta_t;
    if (deg > rev){
      deg -= 360;
    }
  }
  Serial.print("Measured Speed (rev/min): ");
  Serial.print(actual_rpm);
  Serial.print(" Degrees: ");
  Serial.println(deg);
  myPID.Compute();

   analogWrite(reversePin, 0);
   analogWrite(forwardPin, Output);

}
