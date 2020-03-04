#include <Encoder.h>
#include <PID_v1.h>
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6

const double Kp = 0.5; 
const double Ki = 7.0;
const double Kd = 0.;

double Setpoint, actual_rpm, Output;
double newPosition = 0.0;
PID myPID(&newPosition, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder motor(20, 21);
float oldPosition = -999;

const byte numChars = 64; 
char receivedChars[numChars];

boolean newData= false; 

void setup() {
  newPosition = 0;
  Setpoint = newPosition;
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
  if (newPosition != oldPosition){
    oldPosition = newPosition;
    }
//  recvWithStartEndMarkers();
  Setpoint = oldPosition + 20.00;
//  Serial.println(Setpoint);
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" Output: ");
  Serial.print(Output);
  Serial.print(" Current Pos: ");
  Serial.println(newPosition);
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 0);
  delay(2000);
  myPID.Compute();
//  replyToPython();
  }

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false; 
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc; 

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc; 
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
          }
        }
        else {
          receivedChars[ndx] = '\0';
          recvInProgress = false; 
          ndx = 0;
          newData = true;
          }
      }

      else if (rc == startMarker) {
        recvInProgress = true;
        }
    }
  }

 void replyToPython() {
    if (newData == true){
      Serial.print("<This just in ...");
      Serial.print(receivedChars);
      if (receivedChars[0] == 'g'){
	analogWrite(ROBOT_MOTOR_MECH_A, 0);
	analogWrite(ROBOT_MOTOR_MECH_B, 180);
	delay(200);
	}
	analogWrite(ROBOT_MOTOR_MECH_A, 0);
	analogWrite(ROBOT_MOTOR_MECH_B, 0);
 if (receivedChars[0] != 'g'){
//  Setpoint +=20;
  myPID.Compute();
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, Output);
  }
 Serial.print(" ");
  Serial.print(newPosition);
      Serial.print(" ");
      // Serial.print();
      //Serial.print(millis());
      Serial.print('>');
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      newData = false;
    }
      }
