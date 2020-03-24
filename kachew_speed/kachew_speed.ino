// This is a modification of kachew to work with speed
#include <Encoder.h>
#include <PID_v1.h>
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6

const double Kp = 5.0;
const double Ki = 5.0;
const double Kd = 0.4;

double Setpoint, actual_rpm, Output;
double newPosition;
double deg;
const double rev = 360.00;
double previous_change = 0.00;
double cycle_revs;
PID myPID(&actual_rpm, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motor(20, 21);
float oldPosition = -999;
unsigned long starttime;
unsigned long tinit = micros();
unsigned long delta_t;
int i;

void setup() {
  actual_rpm = 0.00;
//  Setpoint = 4.00;
Setpoint = 0.00;
  myPID.SetMode(AUTOMATIC);
  pinMode(ROBOT_MOTOR_MECH_A, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_B, OUTPUT);
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);


  //
  //  Serial.println("<Arduino is ready>");
}

void loop() {
  if (i == 1){
   starttime = micros();
   i = 0;
  }
  newPosition = motor.read();
  if (newPosition != oldPosition) {
    // oldPosition = newPosition;
    previous_change = (newPosition - oldPosition)/1080.00;
    delta_t = micros() - starttime;
    //deg = rev_count * rev;
    oldPosition = newPosition;
    cycle_revs += previous_change;
    i = 1;
      actual_rpm = (previous_change*60000.0)/delta_t;

  }
  
//  unsigned long delta_t = millis() - starttime;
  Serial.print("Actual RPM: ");
  Serial.print(actual_rpm);
  Serial.print(" Output: ");
  Serial.print(Output);
  Serial.print(" Setpoint: ");
  Serial.println(Setpoint);
  
//  cycle_revs = 0;
//  previous_change = 0;
  //Setpoint = 100;
  myPID.Compute();
  //  recvWithStartEndMarkers();
 //  Serial.println(Setpoint);
//  Serial.print("Setpoint ");
//  Serial.print(Setpoint);
//  Serial.print(" Output ");
//  Serial.print(Output);
//  Serial.print(" CurrentPos ");
//  Serial.println(newPosition);
  //Serial.println(newPosition-Setpoint);
  //myPID.Compute();
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, Output);
  //myPID.Compute();
  //if (millis()-tinit>1000) {
  //    Setpoint += 108.0;
  //    tinit = millis();
  // }
  //  replyToPython();
}
