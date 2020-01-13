#include <Encoder.h>
#include <PID_v1.h>

#define A 2
#define B 3
#define forwardPin 5 
#define reversePin 6

double Setpoint, actual_rpm, Output;
long oldPosition  = -999;
double cycle_revs = 0;
double previous_change;
double deg;
const double rev = 360.00;
const double PPR = 1080.00;  // This is a constant given by the manufacturer. Amazon says this should be 540, but our own testing revealed it is 1080.
unsigned long delta_t = 1;
double rev_count = 0;
unsigned long starttime = 0;
unsigned long stoptime = 1;
int i = 1;
const double Kp = 0.001; //tuning parameter for "error-proportional" term.
const double Ki = 0.001; //tuning parameter for "error-integral" term.
const double Kd = 0.001; //tuning parameter for "error-derivative" term. 


PID myPID(&actual_rpm, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder myEnc(A, B);

void setup() {
  Serial.begin(9600);
  Setpoint = 10;
  actual_rpm = 0;
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  digitalWrite(forwardPin, HIGH);
  digitalWrite(reversePin, HIGH);
  myPID.SetMode(AUTOMATIC); //start the PID
  Serial.println("done initializing");
}

void loop() {
  if (i == 1) {
  starttime = millis();
  i=0;
  }
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    rev_count = newPosition / 1080.00;
    previous_change = (newPosition-oldPosition)/1080.0;
    deg = rev_count * rev; 
    oldPosition = newPosition;
    if (deg > rev){
      deg -= 360;
    }
  }
  cycle_revs += previous_change;
  delta_t = millis()-starttime;
  if (delta_t>100) { //after every 0.1 revolutions, calculate the approximate rpm
    actual_rpm = cycle_revs*60000.0/delta_t; //revolutions per second
    i=1;
    cycle_revs = 0;
    previous_change = 0;
    myPID.Compute();
  }
  Serial.print("Measured Speed (rev/min): ");
  Serial.print(actual_rpm);
  Serial.print(" OUTPUT: ");
  Serial.print(Output);
//  Serial.print(" revs: ");
//  Serial.println(cycle_revs);
  Serial.print(" Degrees: ");
  Serial.println(deg);
  
   analogWrite(reversePin, 0);
   analogWrite(forwardPin, Output);

}
