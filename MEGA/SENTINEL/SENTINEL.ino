/* ----------------------------------------------------------
       SENTINEL - Final Arduino MEGA script 

       Harris M

       February 28, 2020
   ---------------------------------------------------------- */

/* ----------------------------------------------------------
       LIBRARIES
   ---------------------------------------------------------- */
#include <../Encoder/Encoder.h>
#include <EEPROM.h>

/* ----------------------------------------------------------
       PINS and CONSTANTS
   ---------------------------------------------------------- */
#define ROBOT_MOTOR_RIGHT_A 8 
#define ROBOT_MOTOR_RIGHT_B 9 
#define ROBOT_MOTOR_LEFT_A 5 
#define ROBOT_MOTOR_LEFT_B 4 
#define ROBOT_MOTOR_MECH_A 7
#define ROBOT_MOTOR_MECH_B 6 
#define e_one 0 
#define e_two 1 
#define e_three 2 
#define e_four 3
#define flag 4
#define REV 360.00 
#define PPR 1080.00 

#define LED LED_BUILTIN 

/* ----------------------------------------------------------
       GLOBAL ARRAYS/Variables
   ---------------------------------------------------------- */
long oldPosition_motorRight  = -999;
long oldPosition_motorLeft  = -999;
long oldPosition_motorMech  = -999; 
float rev_count_motorRight, rev_count_motorLeft, rev_count_motorMech;
float deg_motorRight, deg_motorLeft, def_motorMech; 
float init_val;
unsigned long t; 

/* ----------------------------------------------------------
       FUNCTIONS
   ---------------------------------------------------------- */
void EEPROM_clear();
void EEPROM_write(int index, int val);
unsigned char EEPROM_read(int index);
float angle_calc(float rev_c);

/*--- SETUP ---*/
void setup() {
  // LED ON while we are initializing
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // Instantiate encoders
  Encoder motor1(2, 3);
  Encoder motor2(18, 19);
  Encoder motor3(20, 21);

  // Instantiate motor control pins
  pinMode(ROBOT_MOTOR_RIGHT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_RIGHT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_A, OUTPUT);
  pinMode(ROBOT_MOTOR_LEFT_B, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_A, OUTPUT);
  pinMode(ROBOT_MOTOR_MECH_B, OUTPUT);

  // Instantiate serial bus
  Serial.begin(115200);

  // Initialize EEPROM 
  char rev_1 = EEPROM_read(e_one);
  char rev_2 = EEPROM_read(e_two);
  char rev_3 = EEPROM_read(e_three);
  char rev_4 = EEPROM_read(e_four);

  long x = (((long)rev_4 << 24) | ((long)rev_3 << 16) | ((long)rev_2 << 8) | (long)rev_1);
  init_val = *(float*)&x;

  Serial.println("Done initializing!");
  digitalWrite(LED, LOW);
}

void loop() {
//  analogWrite(ROBOT_MOTOR_RIGHT_A, 130);
//  analogWrite(ROBOT_MOTOR_RIGHT_B, 0);
//  analogWrite(ROBOT_MOTOR_LEFT_A, 130);
//  analogWrite(ROBOT_MOTOR_LEFT_B, 0);
//  analogWrite(ROBOT_MOTOR_MECH_A, 130);
//  analogWrite(ROBOT_MOTOR_MECH_B, 0);
  analogWrite(ROBOT_MOTOR_RIGHT_A, 0);
  analogWrite(ROBOT_MOTOR_RIGHT_B, 0);
  analogWrite(ROBOT_MOTOR_LEFT_A, 0);
  analogWrite(ROBOT_MOTOR_LEFT_B, 0);
  analogWrite(ROBOT_MOTOR_MECH_A, 0);
  analogWrite(ROBOT_MOTOR_MECH_B, 0);

  float newPosition_motor1 = motor1.read();
  float newPosition_motor2 = motor2.read();
  float newPosition_motor3 = motor3.read();

  if (newPosition_motor1 != oldPosition_motor1) {
    oldPosition_motor1 = newPosition_motor1;  
    Serial.print("Motor 1 position: ");
    Serial.println(newPosition_motor1);
  }

  if (newPosition_motor2 != oldPosition_motor2) {
    oldPosition_motor2 = newPosition_motor2;
    Serial.print("Motor 2 position: ");
    Serial.println(newPosition_motor2);  
  }

  if (newPosition_motor3 != oldPosition_motor3) {
    oldPosition_motor3 = newPosition_motor3; 
    Serial.print("Motor 3 position: ");
    Serial.println(newPosition_motor3); 
  }
}
