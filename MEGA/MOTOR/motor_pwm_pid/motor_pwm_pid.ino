#include <Encoder.h>
#include <PID_v1.h>

#define A 2
#define B 3
#define forwardPin 5 
#define reversePin 6
#define pot A15

double Setpoint, actual_rpm, Output;
long oldPosition  = 0; //this was made zero to prevent the code from reacting to it at initial startup.
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
const double Kp = 0.5; //tuning parameter for "error-proportional" term.
const double Ki = 7.0; //tuning parameter for "error-integral" term.
const double Kd = 0.; //tuning parameter for "error-derivative" term.For Now, it seems that this can stay 0. It introduces instability at larger values. 

const double maxRPM = 130.0;


PID myPID(&actual_rpm, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder myEnc(A, B);

void setup() {
  Serial.begin(9600);
  Setpoint = analogRead(pot)*maxRPM/1023.00;
  actual_rpm = 0;
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
//  digitalWrite(forwardPin, HIGH);
//  digitalWrite(reversePin, HIGH);
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
    float abs_save = abs(rev_count);
    abs_save = abs_save - floor(abs_save);
    
    if (rev_count > 0){
      rev_count = abs_save;
    } 
    else if (rev_count < 0) {
      rev_count = -abs_save;  
    }
  }
  cycle_revs += previous_change;
  delta_t = millis()-starttime;
  if (delta_t>50) { //after every 100 milliseconds, calculate the approximate rpm
    actual_rpm = cycle_revs*60000.0/delta_t; //revolutions per second
    i=1;
    cycle_revs = 0;
    previous_change = 0;
    Setpoint = analogRead(pot)*maxRPM/1023.00;
    myPID.Compute();
  }
  Serial.print(" Set Point: ");
  Serial.print(Setpoint);
  Serial.print(" (PWM: ");
  Serial.print(Setpoint*255.0/maxRPM);
  Serial.print(") Measured Speed (rev/min): ");
  Serial.print(actual_rpm);
  Serial.print(" Applied PWM: ");
  Serial.println(Output);
//  Serial.print(" revs: ");
//  Serial.println(cycle_revs);
  
  
   analogWrite(reversePin, 0);
   analogWrite(forwardPin, Output);

}
