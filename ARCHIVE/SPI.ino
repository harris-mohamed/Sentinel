//#include <SPI.h>
//
//void setup() {
//  Serial.begin(115200);
//  SPI.begin();
//}
//
//void loop() {
//  SPIwrite(1, 2);
//  Serial.println("Sent data!");
//  pinMode(10, OUTPUT);
//  delay(1000);
//}
//
//void SPIwrite(int value0, int value1) {
//  SPI.transfer(value0);
//  SPI.transfer(value1);
//}

#include <SPI.h>
byte buf [20];
volatile byte pos;
volatile boolean recibido;
byte pos1 = 90;
byte pos2 = 90;
byte data[] = {45, 34, 2, 12};

void setup() {
  
  Serial.begin (9600);
  pinMode(MISO, OUTPUT);//Configuramos el Miso como salida ya que el Master recibira por aqui.
  SPCR |= _BV(SPE);//Activamos el modo Slave
  pos = 0;   // Posicion 0 de los bytes recibidos
  recibido = false; //Inicializamos que no hemos recibido.

  SPI.attachInterrupt();//Activamos la interrupcion del SPI

}

// Interrupcion SPI
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  buf[pos] = c;
  pos++;
  recibido = true;
}


void loop() {
  
  if (recibido)
  {
    
    for(int x = 0; x<1; x++){
      if(buf[x]==0xFF){break;}
      pos1 = buf[x];
      pos2 = buf[x+1];
      
      
    }
    Serial.println ();
    pos = 0;
    recibido = false;
  }
  Serial.print(pos1);
  Serial.println(pos2);
  
  for (int i=0; i<4; i++) {
   SPI.transfer(data[i]);
   delay(10);    // Send 8 bits
  }
  delay(1000);
  
}
