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
   if(DI1==1){DOut(1,1);}else{DOut(1,0);} //{DigitalOutput(channel,status)
   if(DI2==1){DOut(2,1);}else{DOut(2,0);}
   if(DI3==1){DOut(3,1);}else{DOut(3,0);}
   if(DI4==1){DOut(4,1);}else{DOut(4,0);}
   if(DI5==1){DOut(5,1);}else{DOut(5,0);}
   if(DI6==1){DOut(6,1);}else{DOut(6,0);}
   if(DI7==1){DOut(7,1);}else{DOut(7,0);}
   if(DI8==1){DOut(8,1);}else{DOut(8,0);}

   PIDAOut1(setpoint1,AI2,kp1,ki1,kd1,30,100);//kp1,ki1,kd1
   //PIDAOut2(3000,AI2,kp1,ki1,kd1,30,100);
   //PIDAOut3(3000,AI3,kp1,ki1,kd1,30,100);
   //PIDAOut4(3000,AI4,kp1,ki1,kd1,30,100);
   
  /*Serial.print("AI1 ");
  Serial.print(AI1);
  Serial.print("AI2 ");
  Serial.print(AI2);
  Serial.print(" AI3 ");
  Serial.print(AI3);
  Serial.print(" AI4 ");
  Serial.println(AI4);*/
  
  /*Serial.print("DO1 ");
  Serial.print(DO1);
  Serial.print(" DO2 ");
  Serial.print(DO2);
  Serial.print(" DO3 ");
  Serial.print(DO3);
  Serial.print(" DO4 ");
  Serial.print(DO4);
  Serial.print(" DO5 ");
  Serial.print(DO5);
  Serial.print(" DO6 ");
  Serial.print(DO6);
  Serial.print(" DO7 ");
  Serial.print(DO7);
  Serial.print(" DO8 ");
  Serial.println(DO8);*/

  /*Serial.print("NTC1 ");
  Serial.print(NTC1);
  Serial.print(" NTC2 ");
  Serial.print(NTC2);
  Serial.print(" NTC3 ");
  Serial.print(NTC3);
  Serial.print(" NTC4 ");
  Serial.print(NTC4);
  Serial.print(" NTC5 ");
  Serial.print(NTC5);
  Serial.print(" NTC6 ");
  Serial.print(NTC6);
  Serial.print(" NTC7 ");
  Serial.print(NTC7);
  Serial.print(" NTC8 ");
  Serial.println(NTC8);*/

  
 /////////////////////////////////////////////  
  Register();
}
