/*
 * This program receives CPPM from a RF receiver and passes it through to the flight controller.
 * 
 * If channel 6 is > 1750us, auto mode is entered: Received serial data (from the RPi3) is put
 * through PID loops. The outputs from these PIDs modifies the yaw and roll values slightly. The
 * user still has most of the control however.
 * 
 * A Gimbal channel is set on initatiation to 1912us which happens to be the value which points it
 * vertically downwards.
 * 
 * To Do:
 *  - improve PPM reading resolution from 4us to 1us. 
 *  - Tune X & Y PIDs
 */

#include <PID_v1.h>
#include "Average.h"
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
  X_Dev_Setpoint = 25.0;
  X_Dev_PID.SetMode(MANUAL);
  X_Dev_PID.SetOutputLimits(-200.0, 200.0);
  X_Dev_PID.SetSampleTime(100);

  Y_Dev_Setpoint = 0.0;
  Y_Dev_PID.SetMode(MANUAL);
  Y_Dev_PID.SetOutputLimits(-150.0, 150.0);
  Y_Dev_PID.SetSampleTime(100);
}

//------------------------------------------------------------------------------------
void loop() 
{   
  if(PPM_In.Ch[SWITCH_CH] > MANUAL_PULSEWIDTH and Serial_Rx_Timeout > millis())       // Allow Channels to be slightly adjusted based commands from Pi
  {
    Iterate_Auto_Mode();
  }  
  else                                                                                // Perform CPPM Pass through
  {
    Iterate_Manual_Mode();
  }
}                                                                                     // Serial_Event is run after each loop

//------------------------------------------------------------------------------------
void Iterate_Manual_Mode()
{    
  X_Dev_PID.SetMode(MANUAL);                                                          // Ensure PIDs are in Manual mode
  Y_Dev_PID.SetMode(MANUAL);
  X_Dev_Output = 0.0;
  Y_Dev_Output = 0.0;    

  for (byte i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                   // Copy CPPM from receiver directly across to CPPM output to FC                                         
    PPM_Out[i]= constrain(PPM_In.Ch[i], 950, 2050);
  }
}

//------------------------------------------------------------------------------------
void Iterate_Auto_Mode()
{
  X_Dev_PID.SetMode(AUTOMATIC);                                                       // Ensure PIDs are in Auto mode
  Y_Dev_PID.SetMode(AUTOMATIC);

  for (byte i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                   // Copy Reciver CPPM pulsewidths to a non volitile memory location                                         
    PPM_In.Ch_Modify[i] = PPM_In.Ch[i];
  }  

  //Pitch_Forward();                                                                  // Apply a small pitch forward. Or just trim this in using tx...

  X_Dev_PID.Compute();                                                                // Run the PID loop (if enough time has elapsed)                                                                                                                          
  Y_Dev_PID.Compute();

  PPM_In.Ch_Modify[ROLL_CH] += (int)X_Dev_Output;                                     // Here we modify the Yaw and Roll channels based on PID output
  PPM_In.Ch_Modify[YAW_CH] += (int)Y_Dev_Output;   
  
  for (byte i = 0; i < PPM_CHS_EXCLUDING_GIM; i++){                                   // Write the adjusted channels for CPPM output to FC                            
    PPM_Out[i]= constrain(PPM_In.Ch_Modify[i], 950, 2050);
  }
}

//------------------------------------------------------------------------------------
void Pitch_Forward()                                                                  // Apply a little forward pitch every so often...
{
  PPM_In.Ch_Modify[PITCH_CH] += 25;
}

//------------------------------------------------------------------------------------
void serialEvent() {                                                                  // Receive Packets (255 X_error Y_error) over serial
  static byte Char_Count = 5;
  
  while (Serial.available() > 0)
  {   
    byte inChar = 0;
    
    if (Char_Count >= 2) {                                                            // Read in the first byte
      inChar = Serial.read();
      if (inChar == 255)  {
        Char_Count = 0;
      }
    }     
    else 
    {
      switch (Char_Count) {
        case 0:                                                                       // Read in the X Value
          Serial_X = Serial.read();
          Char_Count++;
          break;

        case 1:                                                                       // Read in the Y Value
          Serial_Y = Serial.read();
          Char_Count++;
          X_Dev_Input = ((double)Serial_X) - 127.0;                                   // COULD ADD A SIMPLE AVERGAE FILTER HERE
          Y_Dev_Input = ((double)Serial_Y) - 127.0;
          Serial_Rx_Timeout = millis() + SERIAL_TIMEOUT;                              // Update the Serial time out variable
          break;

        default:                                                                      // Shouldn't get here          
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
  static char Pulses_Remaining = 0;

  unsigned long Time_at_Start_of_Interrupt = micros();
  char state = PPM_PIN_STATE;
    
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
      PPM_In.Ch[PPM_CHS_EXCLUDING_GIM - Pulses_Remaining] = Time_at_Start_of_Interrupt - Time + 400;
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
    PPM_OUT_LOW;
    OCR1A = PULSE_LENGTH * 2;                                                         // Need *2 since the timer ticks every 0.5ms. (There's no prescaler setting allowing ticks every 1ms)
    state = false;
  } 
  else  {                                                                             // end pulse and calculate when to start the next pulse
    static byte cur_chan_numb = 0;
    static unsigned int calc_rest = 0;
  
    PPM_OUT_HIGH;
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





