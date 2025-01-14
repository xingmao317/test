
//ME Board Libraries 
//Version 1.00
//By Ping
//Last Update 01/11/2024
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SimpleTimer.h>
#include "thermistor.h"
#include <ModbusRtu.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <PID_v1.h>

SimpleTimer TimeSensor;   // timer for Sensor
int i,xx; //parameter read sensor loop
volatile uint16_t au16data[110]; // data array for modbus network sharing
volatile uint16_t Eau16data[110];
/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,1,1); // this is slave @1 and RS-232 or USB-FTDI
Modbus slave2(1,3,1); // this is slave @1 and RS-232 or USB-FTDI

// Big lookup Table (approx 750 entries), subtract 238 from ADC reading to start at 0*C. Entries in 10ths of degree i.e. 242 = 24.2*C Covers 0*C to 150*C For 10k resistor/10k thermistor voltage divider w/ therm on the + side.
const int vtopwm[] PROGMEM = { 255, 255, 255, 255, 255, 200, 200, 200, 200, 200, 190, 190, 190, 190, 190, 180, 180, 180, 180, 180, 175, 172, 169, 165, 162, 159, 156, 153, 150, 147, 145, 142, 139, 137, 134, 131, 128, 126, 123, 120, 118, 117, 114, 111, 109, 106, 104, 102, 100, 98, 96, 93, 91, 89, 87, 85, 83, 81, 80, 79, 76, 74, 72, 71, 69, 67, 65, 64, 62, 60, 59, 57, 55, 54, 52, 51, 49, 48, 46, 45, 44, 42, 41, 39, 38, 37, 35, 34, 33, 32, 30, 29, 28, 27, 26, 24, 23, 22, 21, 20, 19 };

//============= Setting Value =========================
int LED = 24;
//============DigitalInput============
uint16_t DI1; uint16_t DI2; uint16_t DI3; uint16_t DI4; uint16_t DI5; uint16_t DI6; uint16_t DI7; uint16_t DI8;
//============DigitalOutput============ 
uint16_t DO1; uint16_t DO2; uint16_t DO3; uint16_t DO4; uint16_t DO5; uint16_t DO6; uint16_t DO7; uint16_t DO8;
int DOPin; int DOonoff;

//============AnalogInput============
#define NUMSAMPLES 2 //5
int NumAI1[NUMSAMPLES]; float FAI1; float AI1; //int AI1MAP;
int NumAI2[NUMSAMPLES]; float FAI2; float AI2; //int AI2MAP;
int NumAI3[NUMSAMPLES]; float FAI3; float AI3; //int AI3MAP;
int NumAI4[NUMSAMPLES]; float FAI4; float AI4; //int AI4MAP;
int AIPin; int Function;  int Min;  int Max; 
int Function_min; int Function_max;

int16_t AIMin1,AIMax1,AIMin2,AIMax2,AIMin3,AIMax3,AIMin4,AIMax4;// modbus AI
int MapAIMin1,MAP1,MapAIMin2,MAP2,MapAIMin3,MAP3,MapAIMin4,MAP4;// 

int16_t calAI1,calAI2,calAI3,calAI4;
//============AnalogOutput============
int AOpin; int pwm; 
uint16_t AO1;   uint16_t AO2;   uint16_t AO3;   uint16_t AO4;  // modbus AO

//============NTC============
int NT1; float NTC1; int NT2; float NTC2; int NT3; float NTC3; int NT4; float NTC4;
int NT5; float NTC5; int NT6; float NTC6; int NT7; float NTC7; int NT8; float NTC8;
#define NTC_PIN1 A0 // Analog pin used to read the NTC
#define NTC_PIN2 A1
#define NTC_PIN3 A2
#define NTC_PIN4 A3
#define NTC_PIN5 A4
#define NTC_PIN6 A5
#define NTC_PIN7 A6
#define NTC_PIN8 A7
// Analog pin, Nominal resistance at 25 ºC, thermistor's beta coefficient, Value of the series resistor
THERMISTOR thermistor1(NTC_PIN1, 10000, 3950, 10000);         
THERMISTOR thermistor2(NTC_PIN2, 10000, 3950, 10000);         
THERMISTOR thermistor3(NTC_PIN3, 10000, 3950, 10000);         
THERMISTOR thermistor4(NTC_PIN4, 10000, 3950, 10000);                 
THERMISTOR thermistor5(NTC_PIN5, 10000, 3950, 10000);         
THERMISTOR thermistor6(NTC_PIN6, 10000, 3950, 10000);         
THERMISTOR thermistor7(NTC_PIN7, 10000, 3950, 10000);         
THERMISTOR thermistor8(NTC_PIN8, 10000, 3950, 10000);             


//============PID============
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

int PIDOutput1=0;
int PIDOutput2=0;
int PIDOutput3=0;
int PIDOutput4=0;
int kp1=0,ki1=0,kd1=0;
int kp2=0,ki2=0,kd2=0;
int kp3=0,ki3=0,kd3=0;
int kp4=0,ki4=0,kd4=0;
PID myPID1(&Input1, &Output1, &Setpoint1, kp1, 5, 1, REVERSE);//DIRECT
PID myPID2(&Input2, &Output2, &Setpoint2, 10, 5, 1, REVERSE);//DIRECT
PID myPID3(&Input3, &Output3, &Setpoint3, 10, 5, 1, REVERSE);//DIRECT
PID myPID4(&Input4, &Output4, &Setpoint4, 10, 5, 1, REVERSE);//DIRECT


//============Register PID============
int16_t setpoint1,setpoint2,setpoint3,setpoint4;
int16_t outMax1,outMin1,outMax2,outMin2,outMax3,outMin3,outMax4,outMin4;


//============void PID para============
int kp1_1=0,ki1_1=0,kd1_1=0;
int kp2_1=0,ki2_1=0,kd2_1=0;
int kp3_1=0,ki3_1=0,kd3_1=0;
int kp4_1=0,ki4_1=0,kd4_1=0;
//=============Register============
int16_t ps1,ps2,ps3,ps4,ps5,ps6,ps7,ps8,ps9,ps10,ps11,ps12,ps13,ps14,ps15,ps16,ps17,ps18,ps19,ps20,ps21,ps22,ps23,ps24,ps25,ps26,ps27,ps28,ps29,ps30,ps31,ps32,ps33,ps34,ps35,ps36,ps37,ps38,ps39,ps40,ps41,ps42,ps43,ps44,ps45,ps46 ;

//=============Eeprom============
uint16_t Esetpoint1, Ekp1, Eki1, Ekd1, EoutMin1, EoutMax1;
uint16_t Esetpoint2, Ekp2, Eki2, Ekd2, EoutMin2, EoutMax2;
uint16_t Esetpoint3, Ekp3, Eki3, Ekd3, EoutMin3, EoutMax3;
uint16_t Esetpoint4, Ekp4, Eki4, Ekd4, EoutMin4, EoutMax4;
uint16_t EcalAI1, EcalAI2, EcalAI3, EcalAI4;
uint16_t Eps1, Eps2, Eps3, Eps4, Eps5, Eps6, Eps7, Eps8, Eps9, Eps10, Eps11, Eps12, Eps13, Eps14, Eps15, Eps16, Eps17, Eps18, Eps19, Eps20, Eps21, Eps22, Eps23, Eps24, Eps25, Eps26, Eps27, Eps28, Eps29, Eps30, Eps31, Eps32, Eps33, Eps34, Eps35, Eps36, Eps37, Eps38, Eps39, Eps40, Eps41, Eps42, Eps43, Eps44, Eps45, Eps46;

void pinmode(){
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
//============SetLowOutput============
  digitalWrite(30, LOW);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW);
  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);
  digitalWrite(48, LOW);
  pinMode(10, OUTPUT);   //0-10 VDC
  analogWrite (10, 255);       //0% 
  pinMode(11, OUTPUT);   //0-10 VDC 
  analogWrite (11, 255);       //0% 
  pinMode(12, OUTPUT);   //0-10 VDC 
  analogWrite (12, 255);       //0% 
  pinMode(13, OUTPUT);   //0-10 VDC 
  analogWrite (13, 255);       //0% 
 
  if (DO1 > 1) {DO1 = 1;}
  else if (DO1 < 0) {DO1  = 0;}
  if (DO2 > 1) {DO2 = 1;}
  else if (DO2 < 0) {DO2  = 0;}    
  if (DO3 > 1) {DO3 = 1;}
  else if (DO3 < 0) {DO3 = 0;}
  if (DO4 > 1) {DO4 = 1;}
  else if (DO4 < 0) {DO4 = 0;}      
  if (DO5 > 1) {DO5 = 1;}
  else if (DO5 < 0) {DO5 = 0;}     
  if (DO6 > 1) {DO6 = 1;}
  else if (DO6 < 0) {DO6 = 0;}     
  if (DO7 > 1) {DO7 = 1;}
  else if (DO7 < 0) {DO7 = 0;}    
  if (DO8 > 1) {DO8 = 1;}
  else if (DO8 < 0) {DO8 = 0;}
  //==============ROM====================
  EEPROM_readAnything(0, Esetpoint1);setpoint1 = Esetpoint1; //============PID1=============
  EEPROM_readAnything(2, Ekp1);kp1 = Ekp1;
  EEPROM_readAnything(4, Eki1);ki1 = Eki1;
  EEPROM_readAnything(6, Ekd1);kd1 = Ekd1;
  EEPROM_readAnything(8, EoutMin1);outMin1 = EoutMin1;
  EEPROM_readAnything(10, EoutMax1);outMax1 = EoutMax1;
  EEPROM_readAnything(12, Esetpoint2);setpoint2 = Esetpoint2; //============PID2=============
  EEPROM_readAnything(14, Ekp2);kp2 = Ekp2;
  EEPROM_readAnything(16, Eki2);ki2 = Eki2;
  EEPROM_readAnything(18, Ekd2);kd2 = Ekd2;
  EEPROM_readAnything(20, EoutMin2);outMin2 = EoutMin2;
  EEPROM_readAnything(22, EoutMax2);outMax2 = EoutMax2;
  EEPROM_readAnything(24, Esetpoint3);setpoint3 = Esetpoint3; //============PID3=============
  EEPROM_readAnything(26, Ekp3);kp3 = Ekp3;
  EEPROM_readAnything(28, Eki3);ki3 = Eki3;
  EEPROM_readAnything(30, Ekd3);kd3 = Ekd3;
  EEPROM_readAnything(32, EoutMin3);outMin3 = EoutMin3;
  EEPROM_readAnything(34, EoutMax3);outMax3 = EoutMax3;  
  EEPROM_readAnything(36, Esetpoint4);setpoint4 = Esetpoint4; //============PID4=============
  EEPROM_readAnything(38, Ekp4);kp4 = Ekp4;
  EEPROM_readAnything(40, Eki4);ki4 = Eki4;
  EEPROM_readAnything(42, Ekd4);kd4 = Ekd4;
  EEPROM_readAnything(44, EoutMin4);outMin4 = EoutMin4;
  EEPROM_readAnything(46, EoutMax4);outMax4 = EoutMax4;  
  EEPROM_readAnything(48, EcalAI1);calAI1 = EcalAI1;//============calAI=============
  EEPROM_readAnything(50, EcalAI2);calAI2 = EcalAI2; 
  EEPROM_readAnything(52, EcalAI3);calAI3 = EcalAI3; 
  EEPROM_readAnything(54, EcalAI4);calAI4 = EcalAI4;  
  EEPROM_readAnything(56, Eps1);ps1 = Eps1;//============ps=============
  EEPROM_readAnything(58, Eps2);ps2 = Eps2;
  EEPROM_readAnything(60, Eps3);ps3 = Eps3;
  EEPROM_readAnything(62, Eps4);ps4 = Eps4;
  EEPROM_readAnything(64, Eps5);ps5 = Eps5;
  EEPROM_readAnything(66, Eps6);ps6 = Eps6;
  EEPROM_readAnything(68, Eps7);ps7 = Eps7;
  EEPROM_readAnything(70, Eps8);ps8 = Eps8;
  EEPROM_readAnything(72, Eps9);ps9 = Eps9;
  EEPROM_readAnything(74, Eps10);ps10 = Eps10;
  EEPROM_readAnything(76, Eps11);ps11 = Eps11;
  EEPROM_readAnything(78, Eps12);ps12 = Eps12;
  EEPROM_readAnything(80, Eps13);ps13 = Eps13;
  EEPROM_readAnything(82, Eps14);ps14 = Eps14;
  EEPROM_readAnything(84, Eps15);ps15 = Eps15;
  EEPROM_readAnything(86, Eps16);ps16 = Eps16;
  EEPROM_readAnything(88, Eps17);ps17 = Eps17;
  EEPROM_readAnything(90, Eps18);ps18 = Eps18;
  EEPROM_readAnything(92, Eps19);ps19 = Eps19;
  EEPROM_readAnything(94, Eps20);ps20 = Eps20;
  EEPROM_readAnything(96, Eps21);ps21 = Eps21;
  EEPROM_readAnything(98, Eps22);ps22 = Eps22;
  EEPROM_readAnything(100, Eps23);ps23 = Eps23;
  EEPROM_readAnything(102, Eps24);ps24 = Eps24;
  EEPROM_readAnything(104, Eps25);ps25 = Eps25;
  EEPROM_readAnything(106, Eps26);ps26 = Eps26;
  EEPROM_readAnything(108, Eps27);ps27 = Eps27;
  EEPROM_readAnything(110, Eps28);ps28 = Eps28;
  EEPROM_readAnything(112, Eps29);ps29 = Eps29;
  EEPROM_readAnything(114, Eps30);ps30 = Eps30;
  EEPROM_readAnything(116, Eps31);ps31 = Eps31;
  EEPROM_readAnything(118, Eps32);ps32 = Eps32;
  EEPROM_readAnything(120, Eps33);ps33 = Eps33;
  EEPROM_readAnything(122, Eps34);ps34 = Eps34;
  EEPROM_readAnything(124, Eps35);ps35 = Eps35;
  EEPROM_readAnything(126, Eps36);ps36 = Eps36;
  EEPROM_readAnything(128, Eps37);ps37 = Eps37;
  EEPROM_readAnything(130, Eps38);ps38 = Eps38;
  EEPROM_readAnything(132, Eps39);ps39 = Eps39;
  EEPROM_readAnything(134, Eps40);ps40 = Eps40;
  EEPROM_readAnything(136, Eps41);ps41 = Eps41;
  EEPROM_readAnything(138, Eps42);ps42 = Eps42;
  EEPROM_readAnything(140, Eps43);ps43 = Eps43;
  EEPROM_readAnything(142, Eps44);ps44 = Eps44;
  EEPROM_readAnything(144, Eps45);ps45 = Eps45;
  EEPROM_readAnything(146, Eps46);ps46 = Eps46;
  }

void Eeprom(){
  if (setpoint1 != Esetpoint1) {Esetpoint1 = setpoint1;EEPROM_writeAnything(0, Esetpoint1);}//============PID1=============
  if (kp1 != Ekp1) {Ekp1 = kp1;EEPROM_writeAnything(2, Ekp1);}
  if (ki1 != Eki1) {Eki1 = ki1;EEPROM_writeAnything(4, Eki1);}
  if (kd1 != Ekd1) {Ekd1 = kd1;EEPROM_writeAnything(6, Ekd1);}
  if (outMin1 != EoutMin1) {EoutMin1 = outMin1;EEPROM_writeAnything(8, EoutMin1);}
  if (outMax1 != EoutMax1) {EoutMax1 = outMax1;EEPROM_writeAnything(10, EoutMax1);}
  if (setpoint2 != Esetpoint2) {Esetpoint2 = setpoint2;EEPROM_writeAnything(12, Esetpoint2);}//============PID2=============
  if (kp2 != Ekp2) {Ekp2 = kp2;EEPROM_writeAnything(14, Ekp2);}
  if (ki2 != Eki2) {Eki2 = ki2;EEPROM_writeAnything(16, Eki2);}
  if (kd2 != Ekd2) {Ekd2 = kd2;EEPROM_writeAnything(18, Ekd2);}
  if (outMin2 != EoutMin2) {EoutMin2 = outMin2;EEPROM_writeAnything(20, EoutMin2);}
  if (outMax2 != EoutMax2) {EoutMax2 = outMax2;EEPROM_writeAnything(22, EoutMax2);}
  if (setpoint3 != Esetpoint3) {Esetpoint3 = setpoint3;EEPROM_writeAnything(24, Esetpoint3);}//============PID3=============
  if (kp3 != Ekp3) {Ekp3 = kp3;EEPROM_writeAnything(26, Ekp3);}
  if (ki3 != Eki3) {Eki3 = ki3;EEPROM_writeAnything(28, Eki3);}
  if (kd3 != Ekd3) {Ekd3 = kd3;EEPROM_writeAnything(30, Ekd3);}
  if (outMin3 != EoutMin3) {EoutMin3 = outMin3;EEPROM_writeAnything(32, EoutMin3);}
  if (outMax3 != EoutMax3) {EoutMax3 = outMax3;EEPROM_writeAnything(34, EoutMax3);}
  if (setpoint4 != Esetpoint4) {Esetpoint4 = setpoint4;EEPROM_writeAnything(36, Esetpoint4);}//============PID4=============
  if (kp4 != Ekp4) {Ekp4 = kp4;EEPROM_writeAnything(38, Ekp4);}
  if (ki4 != Eki4) {Eki4 = ki4;EEPROM_writeAnything(40, Eki4);}
  if (kd4 != Ekd4) {Ekd4 = kd4;EEPROM_writeAnything(42, Ekd4);}
  if (outMin4 != EoutMin4) {EoutMin4 = outMin4;EEPROM_writeAnything(44, EoutMin4);}
  if (outMax4 != EoutMax4) {EoutMax4 = outMax4;EEPROM_writeAnything(46, EoutMax4);}
  if (calAI1 != EcalAI1) {EcalAI1 = calAI1;EEPROM_writeAnything(48, EcalAI1);}//============calAI=============
  if (calAI2 != EcalAI2) {EcalAI2 = calAI2;EEPROM_writeAnything(50, EcalAI2);}
  if (calAI3 != EcalAI3) {EcalAI3 = calAI3;EEPROM_writeAnything(52, EcalAI3);}
  if (calAI4 != EcalAI4) {EcalAI4 = calAI4;EEPROM_writeAnything(54, EcalAI4);}
  if (ps1 != Eps1) {Eps1 = ps1;EEPROM_writeAnything(56, Eps1);}//============ps=============
  if (ps2 != Eps2) {Eps2 = ps2;EEPROM_writeAnything(58, Eps2);}
  if (ps3 != Eps3) {Eps3 = ps3;EEPROM_writeAnything(60, Eps3);}
  if (ps4 != Eps4) {Eps4 = ps4;EEPROM_writeAnything(62, Eps4);}
  if (ps5 != Eps5) {Eps5 = ps5;EEPROM_writeAnything(64, Eps5);}
  if (ps6 != Eps6) {Eps6 = ps6;EEPROM_writeAnything(66, Eps6);}
  if (ps7 != Eps7) {Eps7 = ps7;EEPROM_writeAnything(68, Eps7);}
  if (ps8 != Eps8) {Eps8 = ps8;EEPROM_writeAnything(70, Eps8);}
  if (ps9 != Eps9) {Eps9 = ps9;EEPROM_writeAnything(72, Eps9);}
  if (ps10 != Eps10) {Eps10 = ps10;EEPROM_writeAnything(74, Eps10);}
  if (ps11 != Eps11) {Eps11 = ps11;EEPROM_writeAnything(76, Eps11);}
  if (ps12 != Eps12) {Eps12 = ps12;EEPROM_writeAnything(78, Eps12);}
  if (ps13 != Eps13) {Eps13 = ps13;EEPROM_writeAnything(80, Eps13);}
  if (ps14 != Eps14) {Eps14 = ps14;EEPROM_writeAnything(82, Eps14);}
  if (ps15 != Eps15) {Eps15 = ps15;EEPROM_writeAnything(84, Eps15);}
  if (ps16 != Eps16) {Eps16 = ps16;EEPROM_writeAnything(86, Eps16);}
  if (ps17 != Eps17) {Eps17 = ps17;EEPROM_writeAnything(88, Eps17);}
  if (ps18 != Eps18) {Eps18 = ps18;EEPROM_writeAnything(90, Eps18);}
  if (ps19 != Eps19) {Eps19 = ps19;EEPROM_writeAnything(92, Eps19);}
  if (ps20 != Eps20) {Eps20 = ps20;EEPROM_writeAnything(94, Eps20);}
  if (ps21 != Eps21) {Eps21 = ps21;EEPROM_writeAnything(96, Eps21);}
  if (ps22 != Eps22) {Eps22 = ps22;EEPROM_writeAnything(98, Eps22);}
  if (ps23 != Eps23) {Eps23 = ps23;EEPROM_writeAnything(100, Eps23);}
  if (ps24 != Eps24) {Eps24 = ps24;EEPROM_writeAnything(102, Eps24);}
  if (ps25 != Eps25) {Eps25 = ps25;EEPROM_writeAnything(104, Eps25);}
  if (ps26 != Eps26) {Eps26 = ps26;EEPROM_writeAnything(106, Eps26);}
  if (ps27 != Eps27) {Eps27 = ps27;EEPROM_writeAnything(108, Eps27);}
  if (ps28 != Eps28) {Eps28 = ps28;EEPROM_writeAnything(110, Eps28);}
  if (ps29 != Eps29) {Eps29 = ps29;EEPROM_writeAnything(112, Eps29);}
  if (ps30 != Eps30) {Eps30 = ps30;EEPROM_writeAnything(114, Eps30);}
  if (ps31 != Eps31) {Eps31 = ps31;EEPROM_writeAnything(116, Eps31);}
  if (ps32 != Eps32) {Eps32 = ps32;EEPROM_writeAnything(118, Eps32);}
  if (ps33 != Eps33) {Eps33 = ps33;EEPROM_writeAnything(120, Eps33);}
  if (ps34 != Eps34) {Eps34 = ps34;EEPROM_writeAnything(122, Eps34);}
  if (ps35 != Eps35) {Eps35 = ps35;EEPROM_writeAnything(124, Eps35);}
  if (ps36 != Eps36) {Eps36 = ps36;EEPROM_writeAnything(126, Eps36);}
  if (ps37 != Eps37) {Eps37 = ps37;EEPROM_writeAnything(128, Eps37);}
  if (ps38 != Eps38) {Eps38 = ps38;EEPROM_writeAnything(130, Eps38);}
  if (ps39 != Eps39) {Eps39 = ps39;EEPROM_writeAnything(132, Eps39);}
  if (ps40 != Eps40) {Eps40 = ps40;EEPROM_writeAnything(134, Eps40);}
  if (ps41 != Eps41) {Eps41 = ps41;EEPROM_writeAnything(136, Eps41);}
  if (ps42 != Eps42) {Eps42 = ps42;EEPROM_writeAnything(138, Eps42);}
  if (ps43 != Eps43) {Eps43 = ps43;EEPROM_writeAnything(140, Eps43);}
  if (ps44 != Eps44) {Eps44 = ps44;EEPROM_writeAnything(142, Eps44);}
  if (ps45 != Eps45) {Eps45 = ps45;EEPROM_writeAnything(144, Eps45);}
  if (ps46 != Eps46) {Eps46 = ps46;EEPROM_writeAnything(146, Eps46);}
  

}

void ReadAI(){ //============ReadSenSor============

    // Fnt = Float NT
    NT1 = thermistor1.read(); NTC1 = NT1 / 10.0; 
    NT2 = thermistor2.read(); NTC2 = NT2 / 10.0;  
    NT3 = thermistor3.read(); NTC3 = NT3 / 10.0;
    NT4 = thermistor4.read(); NTC4 = NT4 / 10.0;  
    NT5 = thermistor5.read(); NTC5 = NT5 / 10.0;
    NT6 = thermistor6.read(); NTC6 = NT6 / 10.0;  
    NT7 = thermistor7.read(); NTC7 = NT7 / 10.0;
    NT8 = thermistor8.read(); NTC8 = NT8 / 10.0;  
    // Read temperature NTC
//It's about finding the average.
    for (i = 0; i < NUMSAMPLES; i++) {
      NumAI1[i] = analogRead(8);   //Read AI1.         
      NumAI2[i] = analogRead(9);   //Read AI2.                                    
      NumAI3[i] = analogRead(10);  //Read AI3.           
      NumAI4[i] = analogRead(11);  //Read AI4.                                   
      //delay(10);
    }

    FAI1 = 0; FAI2 = 0; FAI3 = 0; FAI4 = 0;       
    for (i = 0; i < NUMSAMPLES; i++) {FAI1 += NumAI1[i]; FAI2 += NumAI2[i]; FAI3 += NumAI3[i]; FAI4 += NumAI4[i];}           
    FAI1 /= NUMSAMPLES; FAI2 /= NUMSAMPLES; FAI3 /= NUMSAMPLES; FAI4 /= NUMSAMPLES;

}


void Digitalinput(int DIpin){ 
//pull up active low DI
switch (DIpin) {
   case 1: DIpin = 3;
     if (digitalRead(DIpin) == 0) {DI1 = 1;}else {DI1 = 0;} break;
   case 2: DIpin = 4;
     if (digitalRead(DIpin) == 0) {DI2 = 1;}else {DI2 = 0;}break;
   case 3: DIpin = 5;
     if (digitalRead(DIpin) == 0) {DI3 = 1;}else {DI3 = 0;}break;
   case 4: DIpin = 6;
     if (digitalRead(DIpin) == 0) {DI4 = 1;}else {DI4 = 0;}break;
   case 5: DIpin = 7;
    if (digitalRead(DIpin) == 0) {DI5 = 1;}else {DI5 = 0;}break;
   case 6: DIpin = 8;
    if (digitalRead(DIpin) == 0) {DI6 = 1;}else {DI6 = 0;}break;
   case 7: DIpin = 9;
    if (digitalRead(DIpin) == 0) {DI7 = 1;}else {DI7 = 0;}break;
   case 8: DIpin = 2;
     if (digitalRead(DIpin) == 0) {DI8 = 1;}else {DI8 = 0;}break;
     }
   pinMode(DIpin, INPUT);     // IN1 - DI1
  }
void ReadDI(){ 
  Digitalinput(1);
  Digitalinput(2);
  Digitalinput(3);
  Digitalinput(4);
  Digitalinput(5);
  Digitalinput(6);
  Digitalinput(7);
  Digitalinput(8);
}
  
void DOut(int DOPin, int DOonoff){
  switch (DOPin) {
   case 1:
    DOPin = 30; break;
   case 2:
    DOPin = 32; break;
   case 3:
    DOPin = 34; break;
   case 4:
    DOPin = 36; break;
   case 5:
    DOPin = 42; break;
   case 6:
    DOPin = 44; break;
   case 7:
    DOPin = 46; break;
   case 8:
    DOPin = 48; break;} 
   pinMode(DOPin, OUTPUT);
  switch (DOonoff) {
   case 0:
    digitalWrite(DOPin, LOW); break;
   case 1:
    digitalWrite(DOPin,HIGH); break;}
    if (DOPin == 30) {if(DOonoff==HIGH){DO1 = 1;}else {DO1 = 0;}}
    if (DOPin == 32) {if(DOonoff==HIGH){DO2 = 1;}else {DO2 = 0;}}
    if (DOPin == 34) {if(DOonoff==HIGH){DO3 = 1;}else {DO3 = 0;}}
    if (DOPin == 36) {if(DOonoff==HIGH){DO4 = 1;}else {DO4 = 0;}}
    if (DOPin == 42) {if(DOonoff==HIGH){DO5 = 1;}else {DO5 = 0;}}
    if (DOPin == 44) {if(DOonoff==HIGH){DO6 = 1;}else {DO6 = 0;}}
    if (DOPin == 46) {if(DOonoff==HIGH){DO7 = 1;}else {DO7 = 0;}}
    if (DOPin == 48) {if(DOonoff==HIGH){DO8 = 1;}else {DO8 = 0;}}
    
}

int AI1C, AI2C, AI3C, AI4C; 

void AISet(int AIPin, int Function,int Min, int Max ){
     
switch (Function) {
  case 1:
  Function_min=0; Function_max=1024; break; //0-5V
  case 2:
  Function_min=198; Function_max=973; break;//4-20mA

}
  switch (AIPin) {
    case 1:
    AI1 = map(FAI1,Function_min,Function_max,Min,Max);
   if(FAI1 > Function_max){FAI1 = Function_max;}
   if(FAI1 < Function_min){FAI1 = Function_min;}
   if (AI1 < Min) {AI1 = Min;}//1
   if (AI1 > Max) {AI1 = Max;}//450
    break;
    case 2:
     AI2 = map(FAI2,Function_min,Function_max,Min,Max);
   if(FAI2 > Function_max){FAI2 = Function_max;}
   if(FAI2 < Function_min){FAI2 = Function_min;}
   if (AI2 < Min) {AI2 = Min;}//1
   if (AI2 > Max) {AI2 = Max;}//450
   break;
    case 3:
     AI3 = map(FAI3,Function_min,Function_max,Min,Max); //map(FAI3,Function_min,Function_max,Min,Max);
   if(FAI3 > Function_max){FAI3 = Function_max;} //if(FAI3 > Function_max){FAI3 = Function_max;}
   if(FAI3 < Function_min){FAI3 = Function_min;} //if(FAI3 < Function_min){FAI3 = Function_min;} 
   if (AI3 < Min) {AI3 = Min;}//1
   if (AI3 > Max) {AI3 = Max;}//450
   break;
    case 4:
     AI4 = map(FAI4,Function_min,Function_max,Min,Max);
   if(FAI4 > Function_max){FAI4 = Function_max;}
   if(FAI4 < Function_min){FAI4 = Function_min;}
   if (AI4 < Min) {AI4 = Min;}//1
   if (AI4 > Max) {AI4 = Max;}//450
   break;
  }
  AI1C = AI1+(calAI1/10);
  AI2C = AI2+(calAI2/10);
  AI3C = AI3+(calAI3/10);
  AI4C = AI4+(calAI4/10);
}


void AOut(int AOpin, int pwm){
    // 255 - 0 0-10vdc 
    // เพิ่ม map  0-100% กับ 255 - 0
    switch (AOpin) {
    case 1:
    AOpin =10;AO1 = pwm; pwm = pgm_read_word(&vtopwm[pwm]); 
    break;
    case 2:
    AOpin =11;AO2 = pwm; pwm = pgm_read_word(&vtopwm[pwm]); 
    break;
    case 3:
    AOpin =12;AO3 = pwm; pwm = pgm_read_word(&vtopwm[pwm]); 
    break;
    case 4: 
    AOpin =13;AO4 = pwm; pwm = pgm_read_word(&vtopwm[pwm]); 
    break;
     }
    analogWrite(AOpin, pwm);    
}




void modbusread(){

  ///////////////////////////Modbus PID1////////////////////////////
   if (Eau16data[36] != au16data[36]) {
      setpoint1 = au16data[36];
    Eau16data[36] = au16data[36];}
    if (Eau16data[37] != au16data[37]) {
      kp1 = au16data[37];
    Eau16data[37] = au16data[37];}    
    if (Eau16data[38] != au16data[38]) {
      ki1 = au16data[38];
    Eau16data[38] = au16data[38];} 
    if (Eau16data[39] != au16data[39]) {
      kd1 = au16data[39];
    Eau16data[39] = au16data[39];}
    if (Eau16data[40] != au16data[40]) {
      outMin1 = au16data[40];
    Eau16data[40] = au16data[40];}
    if (Eau16data[41] != au16data[41]) {
      outMax1 = au16data[41];
    Eau16data[41] = au16data[41];}   
  ///////////////////////////Modbus PID2////////////////////////////
    if (Eau16data[42] != au16data[42]) {
      setpoint2 = au16data[42];
    Eau16data[42] = au16data[42];}
     if (Eau16data[43] != au16data[43]) {
      kp2 = au16data[43];
    Eau16data[43] = au16data[43];}    
    if (Eau16data[44] != au16data[44]) {
      ki2 = au16data[44];
    Eau16data[44] = au16data[44];} 
    if (Eau16data[45] != au16data[45]) {
      kd2 = au16data[45];
    Eau16data[45] = au16data[45];}
    if (Eau16data[46] != au16data[46]) {
      outMin2 = au16data[46];
    Eau16data[46] = au16data[46];}
    if (Eau16data[47] != au16data[47]) {
      outMax2 = au16data[47];
    Eau16data[47] = au16data[47];}   
  ///////////////////////////Modbus PID3////////////////////////////
    if (Eau16data[48] != au16data[48]) {
      setpoint3 = au16data[48];
    Eau16data[48] = au16data[48];}
     if (Eau16data[49] != au16data[49]) {
      kp3 = au16data[49];
    Eau16data[49] = au16data[49];}    
    if (Eau16data[50] != au16data[50]) {
      ki3 = au16data[50];
    Eau16data[50] = au16data[50];} 
    if (Eau16data[51] != au16data[51]) {
      kd3 = au16data[51];
    Eau16data[51] = au16data[51];}
    if (Eau16data[52] != au16data[52]) {
      outMin3 = au16data[52];
    Eau16data[52] = au16data[52];}
    if (Eau16data[53] != au16data[53]) {
      outMax3 = au16data[53];
    Eau16data[53] = au16data[53];}  
  ///////////////////////////Modbus PID4////////////////////////////
    if (Eau16data[54] != au16data[54]) {
      setpoint4 = au16data[54];
    Eau16data[54] = au16data[54];}
     if (Eau16data[55] != au16data[55]) {
      kp4 = au16data[55];
    Eau16data[55] = au16data[55];}    
    if (Eau16data[56] != au16data[56]) {
      ki4 = au16data[56];
    Eau16data[56] = au16data[56];} 
    if (Eau16data[57] != au16data[57]) {
      kd4 = au16data[57];
    Eau16data[57] = au16data[57];}
    if (Eau16data[58] != au16data[58]) {
      outMin4 = au16data[58];
    Eau16data[58] = au16data[58];}
    if (Eau16data[59] != au16data[59]) {
      outMax4 = au16data[59];
    Eau16data[59] = au16data[59];} 
  /////////////////////////Calibrate AI/////////////////////////////
    if (Eau16data[60] != au16data[60]) {
      calAI1 = au16data[60];
    Eau16data[60] = au16data[60];}
    if (Eau16data[61] != au16data[61]) {
      calAI2 = au16data[61];
    Eau16data[61] = au16data[61];}
    if (Eau16data[62] != au16data[62]) {
      calAI3 = au16data[62];
    Eau16data[62] = au16data[62];}
    if (Eau16data[63] != au16data[63]) {
      calAI4 = au16data[63];
    Eau16data[63] = au16data[63];}
    
  //////////////////////Modbus Other Set Parameter//////////////////
    if (Eau16data[64] != au16data[64]) {
      ps1 = au16data[64];
    Eau16data[64] = au16data[64];}
    if (Eau16data[65] != au16data[65]) {
      ps2 = au16data[65];
    Eau16data[65] = au16data[65];}
    if (Eau16data[66] != au16data[66]) {
      ps3 = au16data[66];
    Eau16data[66] = au16data[66];}
    if (Eau16data[67] != au16data[67]) {
      ps4 = au16data[67];
    Eau16data[67] = au16data[67];}
    if (Eau16data[68] != au16data[68]) {
      ps5 = au16data[68];
    Eau16data[68] = au16data[68];}
    if (Eau16data[69] != au16data[69]) {
      ps6 = au16data[69];
    Eau16data[69] = au16data[69];}
    if (Eau16data[70] != au16data[70]) {
      ps7 = au16data[70];
    Eau16data[70] = au16data[70];}
    if (Eau16data[71] != au16data[71]) {
      ps8 = au16data[71];
    Eau16data[71] = au16data[71];}
    if (Eau16data[72] != au16data[72]) {
      ps9 = au16data[72];
    Eau16data[72] = au16data[72];}
    if (Eau16data[73] != au16data[73]) {
      ps10 = au16data[73];
    Eau16data[73] = au16data[73];}
    if (Eau16data[74] != au16data[74]) {
      ps11 = au16data[74];
    Eau16data[74] = au16data[74];}
    if (Eau16data[75] != au16data[75]) {
      ps12 = au16data[75];
    Eau16data[75] = au16data[75];}
    if (Eau16data[76] != au16data[76]) {
      ps13 = au16data[76];
    Eau16data[76] = au16data[76];}
    if (Eau16data[77] != au16data[77]) {
      ps14 = au16data[77];
    Eau16data[77] = au16data[77];}
    if (Eau16data[78] != au16data[78]) {
      ps15 = au16data[78];
    Eau16data[78] = au16data[78];}
    if (Eau16data[79] != au16data[79]) {
      ps16 = au16data[79];
    Eau16data[79] = au16data[79];}
    if (Eau16data[80] != au16data[80]) {
      ps17 = au16data[80];
    Eau16data[80] = au16data[80];}
    if (Eau16data[81] != au16data[81]) {
      ps18 = au16data[81];
    Eau16data[81] = au16data[81];}
    if (Eau16data[82] != au16data[82]) {
      ps19 = au16data[82];
    Eau16data[82] = au16data[82];}
    if (Eau16data[83] != au16data[83]) {
      ps20 = au16data[83];
    Eau16data[83] = au16data[83];}
    if (Eau16data[84] != au16data[84]) {
      ps21 = au16data[84];
    Eau16data[84] = au16data[84];}


    if (Eau16data[85] != au16data[85]) {
      ps22 = au16data[85];
    Eau16data[85] = au16data[85];}
    if (Eau16data[86] != au16data[86]) {
      ps23 = au16data[86];
    Eau16data[86] = au16data[86];}
    if (Eau16data[87] != au16data[87]) {
      ps24 = au16data[87];
    Eau16data[87] = au16data[87];}
    if (Eau16data[88] != au16data[88]) {
      ps25 = au16data[88];
    Eau16data[88] = au16data[88];}
    if (Eau16data[89] != au16data[89]) {
      ps26 = au16data[89];
    Eau16data[89] = au16data[89];}
    if (Eau16data[90] != au16data[90]) {
      ps27 = au16data[90];
    Eau16data[90] = au16data[90];}
    if (Eau16data[91] != au16data[91]) {
      ps28 = au16data[91];
    Eau16data[91] = au16data[91];}
    if (Eau16data[92] != au16data[92]) {
      ps29 = au16data[92];
    Eau16data[92] = au16data[92];}
    if (Eau16data[93] != au16data[93]) {
      ps30 = au16data[93];
    Eau16data[93] = au16data[93];}

    if (Eau16data[94] != au16data[94]) {
      ps31 = au16data[94];
    Eau16data[94] = au16data[94];}
    if (Eau16data[95] != au16data[95]) {
      ps32 = au16data[95];
    Eau16data[95] = au16data[95];}
    if (Eau16data[96] != au16data[96]) {
      ps33 = au16data[96];
    Eau16data[96] = au16data[96];}
    if (Eau16data[97] != au16data[97]) {
      ps34 = au16data[97];
    Eau16data[97] = au16data[97];}
    if (Eau16data[98] != au16data[98]) {
      ps35 = au16data[98];
    Eau16data[98] = au16data[98];}
    if (Eau16data[99] != au16data[99]) {
      ps36 = au16data[99];
    Eau16data[99] = au16data[99];}
    if (Eau16data[100] != au16data[100]) {
      ps37 = au16data[100];
    Eau16data[100] = au16data[100];}
    if (Eau16data[101] != au16data[101]) {
      ps38 = au16data[101];
    Eau16data[101] = au16data[101];}
    if (Eau16data[102] != au16data[102]) {
      ps39 = au16data[102];
    Eau16data[102] = au16data[102];}
    if (Eau16data[103] != au16data[103]) {
      ps40 = au16data[103];
    Eau16data[103] = au16data[103];}
    if (Eau16data[104] != au16data[104]) {
      ps41 = au16data[104];
    Eau16data[104] = au16data[104];}
    if (Eau16data[105] != au16data[105]) {
      ps42 = au16data[105];
    Eau16data[105] = au16data[105];}

    if (Eau16data[106] != au16data[106]) {
      ps43 = au16data[106];
    Eau16data[106] = au16data[106];}
    if (Eau16data[107] != au16data[107]) {
      ps44 = au16data[107];
    Eau16data[107] = au16data[107];}
    if (Eau16data[108] != au16data[108]) {
      ps45 = au16data[108];
    Eau16data[108] = au16data[108];}
    if (Eau16data[109] != au16data[109]) {
      ps46 = au16data[109];
    Eau16data[109] = au16data[109];}


} 

void PIDAOut1(int setpoint1_1,float Feedback1_1,int kp1_1,int ki1_1,int kd1_1,int outMin1_1,int outMax1_1){
  Input1 = Feedback1_1;      // Feedback
  Setpoint1 = setpoint1_1*0.1; // Set point 
  myPID1.SetOutputLimits(outMin1_1, outMax1_1);
  myPID1.SetMode(AUTOMATIC);  
  myPID1.Compute();
  kp1 = kp1_1;
  ki1 = ki1_1;
  kd1 = kd1_1;
  PIDOutput1 = int(Output1);
  if (PIDOutput1<=outMin1_1){PIDOutput1=outMin1_1;}  // Set minimum
  if (PIDOutput1>=outMax1_1){PIDOutput1=outMax1_1;}  // Set maximum
  AOut(1,PIDOutput1);
  Serial.print("Input1 ");
  Serial.print(Input1);
  Serial.print("   Setpoint1 ");
  Serial.print(Setpoint1);
  Serial.print("  kp1 ");
  Serial.print(kp1);
  Serial.print("  ki1 ");
  Serial.print(ki1);
  Serial.print("  kd1 ");
  Serial.print(kd1);
  Serial.print("   PIDOutput1 ");
  Serial.println(PIDOutput1);
}

void PIDAOut2(int setpoint2_1,float Feedback2_1,int kp2_1,int ki2_1,int kd2_1,int outMin2_1,int outMax2_1){
  Input2 = Feedback2_1;      // Feedback
  Setpoint2 = setpoint2_1*0.1; // Set point 
  myPID2.SetOutputLimits(outMin2_1, outMax2_1);
  myPID2.SetMode(AUTOMATIC);  
  myPID2.Compute();
  kp2 = kp2_1;
  ki2 = ki2_1;
  kd2 = kd2_1;
  PIDOutput2 = int(Output2);
  if (PIDOutput2<=outMin2_1){PIDOutput2=outMin2_1;}  // Set minimum
  if (PIDOutput2>=outMax2_1){PIDOutput2=outMax2_1;}  // Set maximum
  AOut(2,PIDOutput2);
  Serial.print("Input2 ");
  Serial.print(Input2);
  Serial.print("   Setpoint2 ");
  Serial.print(Setpoint2);
  Serial.print("   PIDOutput2 ");
  Serial.println(PIDOutput2);
}

void PIDAOut3(int setpoint3_1,float Feedback3_1,int kp3_1,int ki3_1,int kd3_1,int outMin3_1,int outMax3_1){
  Input3 = Feedback3_1;      // Feedback
  Setpoint3 = setpoint3_1*0.1; // Set point 
  myPID3.SetOutputLimits(outMin3_1, outMax3_1);
  myPID3.SetMode(AUTOMATIC);  
  myPID3.Compute();
  kp3 = kp3_1;
  ki3 = ki3_1;
  kd3 = kd3_1;
  PIDOutput3 = int(Output3);
  if (PIDOutput3<=outMin3_1){PIDOutput3=outMin3_1;}  // Set minimum
  if (PIDOutput3>=outMax3_1){PIDOutput3=outMax3_1;}  // Set maximum
  AOut(3,PIDOutput3);
  Serial.print("Input3 ");
  Serial.print(Input3);
  Serial.print("   Setpoint3 ");
  Serial.print(Setpoint3);
  Serial.print("   PIDOutput3 ");
  Serial.println(PIDOutput3);
}

void PIDAOut4(int setpoint4_1,float Feedback4_1,int kp4_1,int ki4_1,int kd4_1,int outMin4_1,int outMax4_1){
  Input4 = Feedback4_1;      // Feedback
  Setpoint4 = setpoint4_1*0.1; // Set point 
  myPID4.SetOutputLimits(outMin4_1, outMax4_1);
  myPID4.SetMode(AUTOMATIC);  
  myPID4.Compute();
  kp4 = kp4_1;
  ki4 = ki4_1;
  kd4 = kd4_1;
  PIDOutput4 = int(Output4);
  if (PIDOutput4<=outMin4_1){PIDOutput4=outMin4_1;}  // Set minimum
  if (PIDOutput4>=outMax4_1){PIDOutput4=outMax4_1;}  // Set maximum
  AOut(4,PIDOutput4);
  Serial.print("Input4 ");
  Serial.print(Input4);
  Serial.print("   Setpoint4 ");
  Serial.print(Setpoint4);
  Serial.print("   PIDOutput4 ");
  Serial.println(PIDOutput4);
}



void Register(){
  //Read Only
  au16data[0]=DI1; 
  au16data[1]=DI2; 
  au16data[2]=DI3;   
  au16data[3]=DI4; 
  au16data[4]=DI5; 
  au16data[5]=DI6; 
  au16data[6]=DI7;   
  au16data[7]=DI8;
  au16data[8]=int(AI1); //C = cal AI1C
  au16data[9]=int(AI2); 
  au16data[10]=int(AI3);
  au16data[11]=int(AI4);
  au16data[12]=int(NTC1*10);
  au16data[13]=int(NTC2*10); 
  au16data[14]=int(NTC3*10);
  au16data[15]=int(NTC4*10);   
  au16data[16]=int(NTC5*10);
  au16data[17]=int(NTC6*10); 
  au16data[18]=int(NTC7*10);
  au16data[19]=int(NTC8*10);
  au16data[20]=DO1;   
  au16data[21]=DO2; 
  au16data[22]=DO3; 
  au16data[23]=DO4; 
  au16data[24]=DO5;   
  au16data[25]=DO6;     
  au16data[26]=DO7; 
  au16data[27]=DO8;
  au16data[28]=AO1; 
  au16data[29]=AO2; 
  au16data[30]=AO3; 
  au16data[31]=AO4; 
  au16data[32]=PIDOutput1;                                
  au16data[33]=PIDOutput2;
  au16data[34]=PIDOutput3;
  au16data[35]=PIDOutput4;

  //Read and Write
  au16data[36]=setpoint1;        //PID1
  au16data[37]=kp1;
  au16data[38]=ki1;
  au16data[39]=kd1;                        
  au16data[40]=outMin1;                        
  au16data[41]=outMax1;
  au16data[42]=setpoint2;        //PID2
  au16data[43]=kp2;
  au16data[44]=ki2;
  au16data[45]=kd2;                       
  au16data[46]=outMin2;                        
  au16data[47]=outMax2;
  au16data[48]=setpoint3;        //PID3
  au16data[49]=kp3;
  au16data[50]=ki3;
  au16data[51]=kd3;                       
  au16data[52]=outMin3;                        
  au16data[53]=outMax3;
  au16data[54]=setpoint4;        //PID4
  au16data[55]=kp4;
  au16data[56]=ki4;
  au16data[57]=kd4;                       
  au16data[58]=outMin4;                        
  au16data[59]=outMax4;
  au16data[60]=calAI1;
  au16data[61]=calAI2;
  au16data[62]=calAI3;
  au16data[63]=calAI4;
  au16data[64]=ps1; //
  au16data[65]=ps2; //
  au16data[66]=ps3; //
  au16data[67]=ps4; //
  au16data[68]=ps5; //
  au16data[69]=ps6; //
  au16data[70]=ps7; //
  au16data[71]=ps8; //
  au16data[72]=ps9; //
  au16data[73]=ps10;//
  au16data[74]=ps11;
  au16data[75]=ps12; 
  au16data[76]=ps13;
  au16data[77]=ps14;
  au16data[78]=ps15;  
  au16data[79]=ps16;  
  au16data[80]=ps17;  
  au16data[81]=ps18;  
  au16data[82]=ps19;  
  au16data[83]=ps20; 
  au16data[84]=ps21; 
  au16data[85]=ps22;  
  au16data[86]=ps23;  
  au16data[87]=ps24;  
  au16data[88]=ps25; 
  au16data[89]=ps26; 
  au16data[90]=ps27;//
  au16data[91]=ps28;
  au16data[92]=ps29; 
  au16data[93]=ps30;
  au16data[94]=ps31;
  au16data[95]=ps32;  
  au16data[96]=ps33;  
  au16data[97]=ps34;  
  au16data[98]=ps35;  
  au16data[99]=ps36;  
  au16data[100]=ps37; 
  au16data[101]=ps38;
  au16data[101]=ps38;
  au16data[102]=ps39;
  au16data[103]=ps40;
  au16data[104]=ps41;
  au16data[105]=ps42;
  au16data[106]=ps43;
  au16data[107]=ps44;
  au16data[108]=ps45;
  au16data[109]=ps46;

  slave.poll( au16data, 112 );
  slave2.poll( au16data, 112 );
}       
