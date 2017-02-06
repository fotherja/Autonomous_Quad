/*
 * This program receives CPPM from a RF receiver and passes it through to the flight controller.
 * 
 * In future, the aim is to communicate with the RPi3 which will provide serial information for pitch, roll and yaw to follow a line using 
 * a gimbal mounted camera.
 * 
 * To Do:
 *  - improve PPM reading resolution from 4us to 1us. 
 *  
 * 
 */

#include <PID_v1.h>
#include "Support.h"

//------------------------------------------------------------------------------------
void setup() {
  // Configure pins, interrupts and the serial port:
  pinMode(PPM_In_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_In_Pin), Interrupt_Fxn, CHANGE);

  pinMode(PPM_Out_Pin, OUTPUT);
  digitalWrite(PPM_Out_Pin, HIGH);  

  Serial.begin(115200); 
    

  // Initiallize default PPM values:
  PPM_Out[PITCH_CH]     = 1500;
  PPM_Out[ROLL_CH]      = 1500;
  PPM_Out[THROTTLE_CH]  = 1000;
  PPM_Out[YAW_CH]       = 1500;
  PPM_Out[ARM_CH]       = 1000;
  PPM_Out[SWITCH_CH]    = 1000;
  PPM_Out[AUX_CH]       = 1000;
  PPM_Out[GIMBAL_CH]    = 1912;

  // Configure Timer1 for use as a timer for CPPM receiving:  
  TCCR1A = 0;                                                                                        
  TCCR1B = 0;  
  OCR1A = 100;                                                                        // compare match register, change this
  TCCR1B |= (1 << WGM12);                                                             // turn on CTC mode
  TCCR1B |= (1 << CS11);                                                              // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A);                                                            // enable timer compare interrupt

  // Configure PID Stuff
  X_Dev_Setpoint = 0.0;
  X_Dev_PID.SetMode(MANUAL);
  X_Dev_PID.SetOutputLimits(-100.0, 100.0);
  X_Dev_PID.SetSampleTime(100);

  Y_Dev_Setpoint = 0.0;
  Y_Dev_PID.SetMode(MANUAL);
  Y_Dev_PID.SetOutputLimits(-100.0, 100.0);
  Y_Dev_PID.SetSampleTime(100);
}

//------------------------------------------------------------------------------------
void loop() 
{   
  if(PPM_In.Ch[SWITCH_CH] < MANUAL_PULSEWIDTH)                                        // Perform CPPM Pass through
  {
    Iterate_Manual_Mode();
  }  
  else                                                                                // Allow Channels to be slightly adjusted based commands from Pi
  {
    Iterate_Auto_Mode();
  }
}

//------------------------------------------------------------------------------------
void Iterate_Manual_Mode()
{    
  X_Dev_PID.SetMode(MANUAL);                                                          // Ensure PIDs are in Manual mode
  Y_Dev_PID.SetMode(MANUAL);  

  for (int i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                            
    PPM_Out[i]= constrain(PPM_In.Ch[i], 1000, 2000);
  }
}

//------------------------------------------------------------------------------------
void Iterate_Auto_Mode()
{
  X_Dev_PID.SetMode(AUTOMATIC);                                                       // Ensure PIDs are in Auto mode
  Y_Dev_PID.SetMode(AUTOMATIC);
  
  for (int i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                    // Copy Reciver CPPM pulsewidths to a non volitile memory location                                         
    PPM_In.Ch_Modify[i]= PPM_In.Ch[i];
  }  

  X_Dev_PID.Compute();                                                                                                                            
  Y_Dev_PID.Compute();

  PPM_In.Ch_Modify[ROLL_CH] += (int)X_Dev_Output;
  PPM_In.Ch_Modify[YAW_CH] += (int)Y_Dev_Output;   
  
  for (int i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                    // Write the non volitile, adjusted channels for PPM output to flight controller                            
    PPM_Out[i]= constrain(PPM_In.Ch_Modify[i], 1000, 2000);
  }
}

//------------------------------------------------------------------------------------
void Print_PPM_Channel_Values()
{
  for(int i = 0;i < PPM_CHANNELS;i++)  {
    Serial.print("Ch "); Serial.print(i+1);  
    Serial.print(": ");  Serial.println(PPM_In.Ch[i]);    
  }
}

//------------------------------------------------------------------------------------
void serialEvent() {                // 255, X, Y
  static byte Char_Count = 5;

  while (Serial.available() > 0)
  {
    byte inChar = 0;

    //Read in the first byte
    if (Char_Count >= 2) {
      inChar = Serial.read();
      if (inChar == 255)
        Char_Count = 0;
    } 
    
    else {
      switch (Char_Count) {
        case 0: // Read in the X Value
          Serial_X = Serial.read();
          Char_Count++;
          break;

        case 1: // Read in the Y Value
          Serial_Y = Serial.read();
          Char_Count++;
          X_Dev_Input = ((double)Serial_X) - 127.0;
          Y_Dev_Input = ((double)Serial_Y) - 127.0;
          break;

        default: // Shouldn't get here discard that byte
          // do nothing
          break;
      }
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#### Interrupts #####################################################################################################################################################
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Interrupt_Fxn ()
{
  // Detects start of pulse chain by ignoring pin changes until no rising edge for 5 milliseconds
  // Then it reads each of the pulse widths and overwrites to PPM_Channel struct         
  static unsigned long Time, Time_of_Previous_Pin_Change;
  static int Pulses_Remaining = 0;


  unsigned long Time_at_Start_of_Interrupt = micros();
  char state = digitalRead(PPM_In_Pin);  
  unsigned long Time_Difference = Time_at_Start_of_Interrupt - Time_of_Previous_Pin_Change;
 
  if(Pulses_Remaining && Time_Difference < 3000)
  {
    if(state)
    {
      Time = Time_at_Start_of_Interrupt;
    }
    else
    {
      Pulses_Remaining--;
      PPM_In.Ch[(PPM_CHANNELS - 1) - Pulses_Remaining] = Time_at_Start_of_Interrupt - Time + 400;
    }
  } 
    
  else if(Time_Difference > 5000)
  {     
    Pulses_Remaining = PPM_CHANNELS;
  }   
  
  Time_of_Previous_Pin_Change = Time_at_Start_of_Interrupt;
}

//------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{  
  // Outputs CPPM over the PPM_Out_Pin. Uses timer1 to do this
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {                                                                        // start pulse
    digitalWrite(PPM_Out_Pin, LOW);
    OCR1A = PULSE_LENGTH * 2;                                                         // Need *2 since the timer ticks every 0.5ms. (There's no prescaler setting allowing ticks every 1ms)
    state = false;
  } 
  else  {                                                                             // end pulse and calculate when to start the next pulse
    static byte cur_chan_numb = 0;
    static unsigned int calc_rest = 0;
  
    digitalWrite(PPM_Out_Pin, HIGH);
    state = true;

    if(cur_chan_numb >= PPM_CHANNELS){
      cur_chan_numb = 0;
      calc_rest += PULSE_LENGTH;
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else  {
      OCR1A = (PPM_Out[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest += PPM_Out[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}





