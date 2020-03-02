/* ----------------------------------------------------------
       SENTINEL - Simple blink script

       Harris M

       March 2, 2020
   ---------------------------------------------------------- */

   void setup() {
       pinMode(LED_BUILTIN, OUTPUT);
       digitalWrite(LED_BUILTIN, LOW);
       Serial.begin(115200);
   }

   void loop() {
       digitalWrite(LED_BUILTIN, HIGH);
       delay(1000);
       digitalWrite(LED_BUILTIN, LOW);
       delay(1000);
   }