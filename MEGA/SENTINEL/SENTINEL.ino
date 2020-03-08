/* ----------------------------------------------------------
       SENTINEL - Final Arduino MEGA script 

       Harris M

       February 28, 2020
   ---------------------------------------------------------- */

/* ----------------------------------------------------------
       LIBRARIES
   ---------------------------------------------------------- */
#include <Encoder.h>
#include <EEPROM.h>
#include <PID_v1.h>

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

#define motorLeft_estart  0 
#define motorRight_estart 4
#define motorMech_estart  8

#define flag 4
#define REV 360.00 
#define PPR 1080.00 
#define NUM_CHARS 64 
#define LED LED_BUILTIN 

#define Kp 1.55 
#define Ki 2.5 
#define Kd 0.35

/* ----------------------------------------------------------
       GLOBAL ARRAYS/Variables
   ---------------------------------------------------------- */
long oldPosition_motorRight  = -999;
long oldPosition_motorLeft  = -999;
long oldPosition_motorMech  = -999; 
float rev_count_motorRight, rev_count_motorLeft, rev_count_motorMech;
double newPosition_motorLeft, newPosition_motorRight, newPosition_motorMech;
float deg_motorRight, deg_motorLeft, def_motorMech, deg; 
float motorLeft_init, motorRight_init, motorMech_init;
unsigned long t; 
char receivedMessage[NUM_CHARS];
bool newData = false; 
bool sendFlag = false;

double Setpoint, actual_rpm, Output; 
PID myPID(&newPosition_motorMech, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* ----------------------------------------------------------
       FUNCTIONS
   ---------------------------------------------------------- */
float angle_calc(double rev_c);
unsigned char EEPROM_read(int index);
void EEPROM_clear();
void EEPROM_write(int index, int val);
void recvFromRPI();
void replyToRPI();
void anothaReplyToRPI();

// Instantiate encoders
Encoder motorLeft(2, 3);
Encoder motorRight(18, 19);
Encoder motorMech(20, 21);

/*--- SETUP ---*/
void setup() {
  // LED ON while we are initializing
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

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
  char motorLeft_e_one = EEPROM_read(motorLeft_estart);
  char motorLeft_e_two = EEPROM_read(motorLeft_estart + 1);
  char motorLeft_e_three = EEPROM_read(motorLeft_estart + 2);
  char motorLeft_e_four = EEPROM_read(motorLeft_estart + 3);

  char motorRight_e_one = EEPROM_read(motorRight_estart);
  char motorRight_e_two = EEPROM_read(motorRight_estart + 1);
  char motorRight_e_three = EEPROM_read(motorRight_estart + 2);
  char motorRight_e_four = EEPROM_read(motorRight_estart + 3);

  char motorMech_e_one = EEPROM_read(motorMech_estart);
  char motorMech_e_two = EEPROM_read(motorMech_estart + 1);
  char motorMech_e_three = EEPROM_read(motorMech_estart + 2);
  char motorMech_e_four = EEPROM_read(motorMech_estart + 3);

  long motorLeft_cat = (((long)motorLeft_e_four << 24) | ((long)motorLeft_e_three << 16) | ((long)motorLeft_e_two << 8) | (long)motorLeft_e_one);
  motorLeft_init = *(float*)&motorLeft_cat;

  long motorRight_cat = (((long)motorRight_e_four << 24) | ((long)motorRight_e_three << 16) | ((long)motorRight_e_two << 8) | (long)motorRight_e_one);
  motorRight_init = *(float*)&motorRight_cat;

  long motorMech_cat = (((long)motorMech_e_four << 24) | ((long)motorMech_e_three << 16) | ((long)motorMech_e_two << 8) | (long)motorMech_e_one);
  motorMech_init = *(float*)&motorMech_cat;

  // Initialize PID 
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  

  Serial.println("<Arduino is ready>");
  digitalWrite(LED, LOW);
}

void loop() {
  newPosition_motorLeft = motorLeft.read();
  newPosition_motorRight = motorRight.read();
  newPosition_motorMech = motorMech.read();

  if (newPosition_motorLeft != oldPosition_motorLeft) {
    oldPosition_motorLeft = newPosition_motorLeft;  

    rev_count_motorLeft = newPosition_motorLeft / 1080.00; 
    rev_count_motorLeft += motorLeft_init;

    float abs_save = abs(rev_count_motorLeft);
    abs_save = abs_save - floor(abs_save);

    if (rev_count_motorLeft > 0){
      rev_count_motorLeft = abs_save;
    } 
    else if (rev_count_motorLeft < 0) {
      rev_count_motorLeft = -abs_save;
    }

    double angle = angle_calc(rev_count_motorLeft);

    byte* byte_split = (byte*)&rev_count_motorLeft; 

    EEPROM_write(motorLeft_estart, byte_split[motorLeft_estart]);
    EEPROM_write(motorLeft_estart + 1, byte_split[motorLeft_estart + 1]);
    EEPROM_write(motorLeft_estart + 2, byte_split[motorLeft_estart + 2]);
    EEPROM_write(motorLeft_estart + 3, byte_split[motorLeft_estart + 3]);
    // Following two lines are for debugging
    // Serial.print("Motor 1 position: ");
    // Serial.println(newPosition_motor1);
  }

  if (newPosition_motorRight != oldPosition_motorRight) {
    oldPosition_motorRight = newPosition_motorRight;

    rev_count_motorRight = newPosition_motorRight / 1080.00; 
    rev_count_motorRight += motorRight_init;

    float abs_save = abs(rev_count_motorRight);
    abs_save = abs_save - floor(abs_save);

    if (rev_count_motorRight > 0){
      rev_count_motorRight = abs_save;
    } 
    else if (rev_count_motorRight < 0) {
      rev_count_motorRight = -abs_save;
    }

    double angle = angle_calc(rev_count_motorRight);

    byte* byte_split = (byte*)&rev_count_motorRight; 

    EEPROM_write(motorRight_estart, byte_split[motorRight_estart]);
    EEPROM_write(motorRight_estart + 1, byte_split[motorRight_estart + 1]);
    EEPROM_write(motorRight_estart + 2, byte_split[motorRight_estart + 2]);
    EEPROM_write(motorRight_estart + 3, byte_split[motorRight_estart + 3]);
    // Following two lines are for debugging
    // Serial.print("Motor 2 position: ");
    // Serial.println(newPosition_motor2);  
  }

  if (newPosition_motorMech != oldPosition_motorMech) {
    oldPosition_motorMech = newPosition_motorMech;

    rev_count_motorMech = newPosition_motorMech / 1080.00; 
    rev_count_motorMech += motorMech_init;

    float abs_save = abs(rev_count_motorMech);
    abs_save = abs_save - floor(abs_save);

    if (rev_count_motorMech > 0){
      rev_count_motorMech = abs_save;
    } 
    else if (rev_count_motorMech < 0) {
      rev_count_motorMech = -abs_save;
    }

    double angle = angle_calc(rev_count_motorMech);

    byte* byte_split = (byte*)&rev_count_motorMech; 

    EEPROM_write(motorMech_estart, byte_split[motorMech_estart]);
    EEPROM_write(motorMech_estart + 1, byte_split[motorMech_estart + 1]);
    EEPROM_write(motorMech_estart + 2, byte_split[motorMech_estart + 2]);
    EEPROM_write(motorMech_estart + 3, byte_split[motorMech_estart + 3]);
    // Following two lines are for debugging
    // Serial.print("Motor 3 position: ");
    // Serial.println(newPosition_motor3); 
  }

  recvFromRPI();
  replyToRPI();

  if (Setpoint - newPosition_motorMech < 10 && sendFlag == true){
      Setpoint = newPosition_motorMech;
      anothaReplyToRPI();
      sendFlag = false;
  }
}

/*--------------------------------------------------
 * Function: EEPROM_clear 
 * Purpose: Clears the EEPROM
 * Input: None
 * Output: None
--------------------------------------------------*/
void EEPROM_clear() {
  // Following loop clears each EEPROM index
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }

  //Following loop checks to see if the CLEAR was successful
  for (int address = 0; address < EEPROM.length(); address++) {
    int current_val = EEPROM.read(address);

    // I commented this out for now, but I should come back to this later 
    // and actually code it 
    if (current_val != 0) {
      // Serial.println("EEPROM CLEAR not successful, something went wrong.");
      break;
    }
  }
  
}

/* ----------------------------------------------------------
   Function:      EEPROM_read
   Description:   Reads a desired index from the EEPROM
   Inputs:        Index to read from the EEPROM
   Outputs:       Value in the index of the EEPROM
   ---------------------------------------------------------- */
unsigned char EEPROM_read(int index) {
  unsigned char current_val = EEPROM.read(index);
  return current_val;
}

/* ----------------------------------------------------------
   Function:      EEPROM_write
   Description:   Writes a value into the desired index of the
                  EEPROM
   Inputs:        Index we want to write to and the value we want
   Outputs:       None
   ---------------------------------------------------------- */
void EEPROM_write(int index, int val) {
  EEPROM.write(index, val);
}

/* ----------------------------------------------------------
   Function:      angle_calc
   Description:   Calculates the angle of the sensor given rev count
   Inputs:        The number of revolutions passed
   Outputs:       The number of degrees turned
   ---------------------------------------------------------- */
float angle_calc(double rev_c) {
  int round_ = (int) rev_c;

  deg = abs(rev_c - round_) * REV;

  if (rev_c < 0) {
    deg *= -1;
  }

  return deg;
}

/* ----------------------------------------------------------
   Function:      recvFromRPI
   Description:   Receives the properly encoded message from the 
                  RPI
   Inputs:        None
   Outputs:       None, but populates receivedChars
   ---------------------------------------------------------- */
void recvFromRPI(){
  static boolean recvInProgress = false; 
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc; 

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedMessage[ndx] = rc; 
        ndx++;
        if (ndx >= NUM_CHARS) {
          ndx = NUM_CHARS - 1;
          }
        }
        else {
          receivedMessage[ndx] = '\0';
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

/* ----------------------------------------------------------
   Function:      replyToRPI
   Description:   Replies to the RPI on the serial bus 
   Inputs:        None
   Outputs:       None
   ---------------------------------------------------------- */
void replyToRPI(){
  if (newData == true){
      if (receivedMessage[0] == 'g'){
        sendFlag = true;
        Setpoint += 108.0;
      }
      Serial.print("<");
      Serial.print(" ");
      Serial.print(newPosition_motorMech);
      Serial.print(" ");
      Serial.print('>');
      newData = false;
    }
    myPID.Compute(); 
    analogWrite(ROBOT_MOTOR_MECH_A, 0);
    analogWrite(ROBOT_MOTOR_MECH_B, Output);
}

/* ----------------------------------------------------------
   Function:      anothaReplyToRPI
   Description:   Replies to the RPI on the serial bus 
   Inputs:        None
   Outputs:       None
   ---------------------------------------------------------- */
void anothaReplyToRPI(){
  Serial.print("<");
  Serial.print("D");
  Serial.print(" ");
  Serial.print(angle_calc(newPosition_motorMech / 1080.00));
//   Serial.print(" ");
  Serial.print(">");
//   delay(500);
}
