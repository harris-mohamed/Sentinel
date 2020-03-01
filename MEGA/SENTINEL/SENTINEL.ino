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
#define NUM_CHARS 64 

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
char receivedMessage[NUM_CHARS];
bool newData = false; 

/* ----------------------------------------------------------
       FUNCTIONS
   ---------------------------------------------------------- */
float angle_calc(float rev_c);
unsigned char EEPROM_read(int index);
void EEPROM_clear();
void EEPROM_write(int index, int val);
void recvFromRPI();
void replyToRPI();

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

  Serial.println("<Arduino is ready>");
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
float angle_calc(float rev_c) {
  int round_ = (int) rev_c;

  deg = abs(rev_count - round_) * rev;

  if (rev_count < 0) {
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

/* ----------------------------------------------------------
   Function:      replyToRPI
   Description:   Replies to the RPI on the serial bus 
   Inputs:        None
   Outputs:       None
   ---------------------------------------------------------- */
void replyToRPI(){
  if (newData == true){
    Serial.print("<This just in ...");
    Serial.print(receivedChars);
      if (receivedChars[0] == 'g'){
        analogWrite(ROBOT_MOTOR_MECH_A, 0);
        analogWrite(ROBOT_MOTOR_MECH_B, 0);
        delay(100);
      }
      analogWrite(ROBOT_MOTOR_MECH_A, 0);
      analogWrite(ROBOT_MOTOR_MECH_B, 0);
      Serial.print(" ");
      Serial.print(newPosition);
      Serial.print(" ");
      // Serial.print();
      //Serial.print(millis());
      Serial.print('>');
      newData = false;
    }
}