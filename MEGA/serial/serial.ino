// #include <Encoder.h>

#define ROBOT_MOTOR_RIGHT_A 8 
#define ROBOT_MOTOR_RIGHT_B 9 
#define ROBOT_MOTOR_LEFT_A 5 
#define ROBOT_MOTOR_LEFT_B 4 
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6 

// Encoder motor1(2, 3);
// Encoder motor2(18, 19);
// Encoder motor3(20, 21);

float oldPosition_motor1  = -999;
float oldPosition_motor2  = -999;
float oldPosition_motor3  = -999;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ROBOT_MOTOR_RIGHT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_RIGHT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_A, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_B, OUTPUT);
  Serial.begin(115200);
  // Serial.println("Done initializing!");

  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 0);
  analogWrite(ROBOT_MOTOR_RIGHT_A, 0);
  analogWrite(ROBOT_MOTOR_RIGHT_B, 0);
  analogWrite(ROBOT_MOTOR_LEFT_A, 0);
  analogWrite(ROBOT_MOTOR_LEFT_B, 0); 
}

void loop() {

	analogWrite(ROBOT_MOTOR_MECH_A, 0);
	analogWrite(ROBOT_MOTOR_MECH_B, 0);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);
	// digitalWrite(LED_BUILTIN, HIGH);
	// int incomingByte = Serial.read();
	// while (incomingByte != 'g') {
	//	incomingByte = Serial.read();
	// }
	// Serial.println('y'); 
	// digitalWrite(LED_BUILTIN, LOW);
 	if (Serial.available()) {
		// digitalWrite(LED_BUILTIN, HIGH);
		char in = Serial.read();
		if (in == 'g'){
		     digitalWrite(LED_BUILTIN, HIGH);
		}
		// Serial.println(in);
	}	

	// Serial.println("yeet");	
	analogWrite(ROBOT_MOTOR_MECH_A, 0);
	analogWrite(ROBOT_MOTOR_MECH_B, 0);
	delay(1000);
	// delay(50);

	// Serial.print('y');  
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
