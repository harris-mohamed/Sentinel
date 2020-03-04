#include <Encoder.h>
#include <PID_v1.h>
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6

const double Kp = 1.55;
const double Ki = 2.5;
const double Kd = 0.35;

double Setpoint, actual_rpm, Output;
double newPosition = 0.0;
PID myPID(&newPosition, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motor(20, 21);
float oldPosition = -999;
unsigned long tinit = millis();

void setup() {
  newPosition = 0;
  Setpoint = 108.0;
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
  newPosition = motor.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  //  recvWithStartEndMarkers();
  //  Serial.println(Setpoint);
//  Serial.print("Setpoint ");
//  Serial.print(Setpoint);
//  Serial.print(" Output ");
//  Serial.print(Output);
//  Serial.print(" CurrentPos ");
//  Serial.println(newPosition);
  Serial.println(newPosition-Setpoint);
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 0);
  myPID.Compute();
  if (millis()-tinit>1000) {
      Setpoint += 108.0;
      tinit = millis();
  }
  //  replyToPython();
}
