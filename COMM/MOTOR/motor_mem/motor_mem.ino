/* ----------------------------------------------------------
       SENTINEL - MOTOR ENCODER SCRIPT w/EEPROM correctness

       Harris M

       December 30, 2019
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
float init_val;
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
    byte rev_1 = EEPROM_read(index_one);
    byte rev_2 = EEPROM_read(index_two);
    byte rev_3 = EEPROM_read(index_three);
    byte rev_4 = EEPROM_read(index_four);

    long x = (((long)rev_4 << 24) | ((long)rev_3 << 16) | ((long)rev_2 << 8) | (long)rev_1);
    float init_val = *(float*)&x;
//    Serial.print("Initial value read: ");
//    Serial.println(y, DEC);
}

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (EEPROM_read(4) != 0x00){
      newPosition = init_val;
    }
    oldPosition = newPosition;
    rev_count = newPosition / 1080.00;
    double angle = angle_calc(rev_count);

    byte* pointer = (byte*)&rev_count;
    EEPROM_write(index_one, pointer[index_one]);
    EEPROM_write(index_two, pointer[index_two]);
    EEPROM_write(index_three, pointer[index_three]);
    EEPROM_write(index_four, pointer[index_four]);
    EEPROM_write(4, 0xFF);
    
    Serial.print("Revolutions: ");
    Serial.print(rev_count);
    Serial.print(" Degrees: ");
    Serial.println(angle);
  }
  
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
