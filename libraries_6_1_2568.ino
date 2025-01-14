#include "MEBoard.h"

void setup() {
  
  pinmode();
  TimeSensor.setInterval(3000,ReadSensor);  //10000 = 10Sec 3000
  Serial.begin(19200);  // for debug
  slave.begin(9600); // baud-rate at 9600  RS485(1)  
  slave2.begin(9600); // baud-rate at 9600  RS485(2) 
}
void ReadSensor(){
  ReadAI();
  AISet(1,2,0,450);//5, 448
  AISet(2,2,0,450); 
  AISet(3,2,0,450); 
  AISet(4,2,0,450); 
  
}
void loop() {
   TimeSensor.run();
   ReadDI();
   modbusread();
   Eeprom(); 
 /////////////////////////////////////////////
  
 /////////////////////////////////////////////  
  Register();
}
