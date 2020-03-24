// This is a modification of kachew to work with speed
#include <Encoder.h>
#include <PID_v1.h>
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6

const double Kp = 10.0;
const double Ki = 10.0;
const double Kd = 0.00;

double Setpoint, actual_rpm, Output;
double newPosition = 0.0;
double deg;
const double rev = 360.00;
double previous_change;
double cycle_revs;
PID myPID(&newPosition, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motor(20, 21);
float oldPosition = -999;
unsigned long starttime;
unsigned long tinit = millis();
int i;

void setup() {
  actual_rpm = 0.00;
  Setpoint = 30.00;
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
   starttime = millis();
   i = 0;
  }
  newPosition = motor.read();
  if (newPosition != oldPosition) {
    // oldPosition = newPosition;
    previous_change = (newPosition - oldPosition)/1080.00;
    //deg = rev_count * rev;
    oldPosition = newPosition;
     
  }
  cycle_revs += previous_change;
  unsigned long delta_t = millis() - starttime;
  if (delta_t>50) {
	actual_rpm = (cycle_revs*60000.0)/delta_t;
        i = 1;
        cycle_revs = 0;
        previous_change = 0;
        //Setpoint = 100;
        myPID.Compute();
	}
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
