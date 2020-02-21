// #include <Encoder.h>

#define ROBOT_MOTOR_RIGHT_A 4
#define ROBOT_MOTOR_RIGHT_B 5
#define ROBOT_MOTOR_LEFT_A 6
#define ROBOT_MOTOR_LEFT_B 7
#define ROBOT_MOTOR_MECH_A 8
#define ROBOT_MOTOR_MECH_B 9

// Encoder motor1(2, 3);
// Encoder motor2(18, 19);
// Encoder motor3(20, 21);

float oldPosition_motor1  = -999;
float oldPosition_motor2  = -999;
float oldPosition_motor3  = -999;

void setup() {
  pinMode(ROBOT_MOTOR_RIGHT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_RIGHT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_A, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_B, OUTPUT);
  Serial.begin(115200);
  Serial.println("Done initializing!");

  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 130);
  delay(2000); 
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 0); 
}

void loop() {
//  analogWrite(ROBOT_MOTOR_RIGHT_A, 130);
//  analogWrite(ROBOT_MOTOR_RIGHT_B, 0);
//  analogWrite(ROBOT_MOTOR_LEFT_A, 130);
//  analogWrite(ROBOT_MOTOR_LEFT_B, 0);
//  analogWrite(ROBOT_MOTOR_MECH_A, 130);
//  analogWrite(ROBOT_MOTOR_MECH_B, 0);
// analogWrite(ROBOT_MOTOR_RIGHT_A, 0);
//  analogWrite(ROBOT_MOTOR_RIGHT_B, 0);
//  analogWrite(ROBOT_MOTOR_LEFT_A, 0);
//  analogWrite(ROBOT_MOTOR_LEFT_B, 0);
//  analogWrite(ROBOT_MOTOR_MECH_A, 0);
//  analogWrite(ROBOT_MOTOR_MECH_B, 0);
//
//  float newPosition_motor1 = motor1.read();
//  float newPosition_motor2 = motor2.read();
//  float newPosition_motor3 = motor3.read();
//
//  if (newPosition_motor1 != oldPosition_motor1) {
//    oldPosition_motor1 = newPosition_motor1;  
//    Serial.print("Motor 1 position: ");
//    Serial.println(newPosition_motor1);
//  }

  //if (newPosition_motor2 != oldPosition_motor2) {
  //  oldPosition_motor2 = newPosition_motor2;
   // Serial.print("Motor 2 position: ");
    //Serial.println(newPosition_motor2);  
  //}

  //if (newPosition_motor3 != oldPosition_motor3) {
   // oldPosition_motor3 = newPosition_motor3; 
   // Serial.print("Motor 3 position: ");
    //Serial.println(newPosition_motor3); 
  //}
}
