/* ----------------------------------------------------------
       SENTINEL - MOTOR CALIBRATION SCRIPT w/EEPROM correctness

       Harris M

       December 31, 2019
   ---------------------------------------------------------- */

/* ----------------------------------------------------------
       LIBRARIES
   ---------------------------------------------------------- */
#include <Encoder.h>
#include <EEPROM.h>

/* ----------------------------------------------------------
       PIN
   ---------------------------------------------------------- */
#define LIVE 7

/* ----------------------------------------------------------
       GLOBAL ARRAYS/Variables
   ---------------------------------------------------------- */
long oldPosition  = -999;
float rev_count;
float deg;
const float rev = 360.00;
const float PPR = 1080.00;  // This is a constant given by the manufacturer. Amazon says this should be 540, but our own testing revealed it is 1080.
const int index_one = 0;
const int index_two = 1;
const int index_three = 2;
const int index_four = 3;

/* ----------------------------------------------------------
       ENCODER INSTANTIATION
   ---------------------------------------------------------- */
Encoder myEnc(2, 3);    // Pins 2 and 3 are interrupt pins 

/* ----------------------------------------------------------
        FUNCTIONS
   ---------------------------------------------------------- */
void EEPROM_clear(); 
byte EEPROM_read(int index);
void EEPROM_write(int index, int val); 
float angle_calc(float rev_c);

void setup() {
    pinMode(LIVE, OUTPUT);
    
    /* ----- UART INIT ----- */
    digitalWrite(LIVE, LOW);
    Serial.begin(9600, SERIAL_8N1);
    digitalWrite(LIVE, HIGH);
    delay(1000);

    /* ----- EEPROM INIT ----- */ 
    EEPROM_clear();
}

void loop() {
    
}

/* ----------------------------------------------------------
 * Function:      EEPROM_clear
 * Description:   Clears the EEPROM
 * Inputs:        None
 * Outputs:       None
 * ---------------------------------------------------------- */
void EEPROM_clear(){
    // Following loop clears each EEPROM index
    for (int i = 0; i < EEPROM.length(); i++){
      EEPROM.write(i, 0);  
    }

    //Following loop checks to see if the CLEAR was successful 
    for (int address = 0; address < EEPROM.length(); address++){
      int current_val = EEPROM.read(address);

      if (current_val != 0){
          Serial.println("EEPROM CLEAR not successful, something went wrong.");
          break;
      }
    }

    Serial.println("EEPROM CLEAR successful");
    Serial.println("Motor calibration complete.");
}

/* ----------------------------------------------------------
 * Function:      EEPROM_read
 * Description:   Reads a desired index from the EEPROM
 * Inputs:        Index to read from the EEPROM
 * Outputs:       Value in the index of the EEPROM
 * ---------------------------------------------------------- */
byte EEPROM_read(int index){
    byte current_val = EEPROM.read(index);
    return current_val;
}


/* ----------------------------------------------------------
 * Function:      EEPROM_write
 * Description:   Writes a value into the desired index of the 
 *                EEPROM
 * Inputs:        Index we want to write to and the value we want
 * Outputs:       None
 * ---------------------------------------------------------- */
void EEPROM_write(int index, int val){
    EEPROM.write(index, val);
}

/* ----------------------------------------------------------
 * Function:      angle_calc
 * Description:   Calculates the angle of the sensor given rev count
 * Inputs:        The number of revolutions passed
 * Outputs:       The number of degrees turned 
 * ---------------------------------------------------------- */
float angle_calc(float rev_c){
    int round_ = (int) rev_c;

    deg = abs(rev_count - round_) * rev;

    if (rev_count < 0) {
       deg *= -1;
    }     

    return deg;
}
