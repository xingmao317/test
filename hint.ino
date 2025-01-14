 /*
  parameter    modbus      details  
  ------------Read Only------------
  DI1            0         alarm 
  DI2            1         alarm 
  DI3            2         alarm 
  DI4            3         alarm 
  DI5            4         alarm 
  DI6            5         alarm 
  DI7            6         
  DI8            7         
  AI1            8         
  AI2            9         
  AI3            10        
  AI4            11        
  NTC1           12         
  NTC2           13        probe  
  NTC3           14        probe    
  NTC4           15        probe   
  NTC5           16        probe 
  NTC6           17        probe 
  NTC7           18        probe   
  NTC8           19        probe    
  DO1            20         
  DO2            21          
  DO3            22          
  DO4            23          
  DO5            24          
  DO6            25         
  DO7            26        
  DO8            27          
  AO1            28         
  AO2            29        
  AO3            30        
  AO4            31       
  PIDOutput1     32         
  PIDOutput2     33           
  PIDOutput3     34           
  PIDOutput4     35           
  ----------Read and Write-----------       
  setpoint1      36          PID1
  kp1            37         reserve
  ki1            38         reserve  
  kd1            39         reserve            
  outMin1        40               
  outMax1        41
  setpoint2      42          PID2
  kp2            43         reserve  
  ki2            44         reserve  
  kd2            45         reserve         
  outMin2        46               
  outMax2        47
  setpoint3      48          PID3
  kp3            49         reserve  
  ki3            50         reserve  
  kd3            51         reserve              
  outMin3        52              
  outMax3        53
  setpoint4      54          PID4
  kp4            55         reserve  
  ki4            56         reserve  
  kd4            57         reserve             
  outMin4        58              
  outMax4        59
  calAI1         60
  calAI2         61
  calAI3         62
  calAI4         63
  ps1            64             
  ps2            65            
  ps3            66            
  ps4            67            
  ps5            68            
  ps6            69           
  ps7            70           
  ps8            71           
  ps9            72           
  ps10           73           
  ps11           74           
  ps12           75           
  ps13           76           
  ps14           77           
  ps15           78           
  ps16           79           
  ps17           80           
  ps18           81           
  ps19           82           
  ps20           83           
  ps21           84           
  ps22           85           
  ps23           86           
  ps24           87           
  ps25           88           
  ps26           89           
  ps27           90           
  ps28           91           
  ps29           92           
  ps30           93           
  ps31           94           
  ps32           95           
  ps33           96           
  ps34           97           
  ps35           98           
  ps36           99           
  ps37           100          
  ps38           101 
  ps39           102          
  ps40           103           
  ps41           104          
  ps42           105          
  ps43           106          
  ps44           107
  ps45           108           
  ps46           109          
  

  DOut(channel,output)
  chennel =>select chennel 1-4
  output => 0,1
  
  AISet(channel,input,min,max)
  chennel =>select chennel 1-4
  input =>sensor type  1=0-5VDC , 2=4-20mA 
  min => minimum value of sensor range
  max => maximum value of sensor range  

  AOut(channel,output)
  chennel =>select chennel 1-4
  output => 0-10VDC 
   
   PIDAOut1(setpoint,input,direction,kP,kI,kD,min,max)
   setpoint => setpoint value for pid control
   input => input feedback for pid control
   direction => pid control type 0 = Forward , 1 = Backward 
   kP = > propotional constant value (0-100%)
   kI = > integral constant value (0-100%)
   kD = > differential constant value (0-100%)
   min => minimum value of pid output 
   max => maximum value of pid output   */

   /*if(DI1==1 || ps1==1){DOut(1,1);}else{DOut(1,0);} //{DOut(channel,status)
    //AOut(1,ps9);
   PIDAOut1(setpoint1,AI1C,0,kp1,ki1,kd1,outMin1,outMax1);
   
  Serial.print("  NTC2 ");
  Serial.print(NTC2);
  Serial.print("  NTC3 ");
  Serial.print(NTC3);
  Serial.print("  NTC4 ");
  Serial.println(NTC4);
*/