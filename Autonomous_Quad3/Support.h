// DEFINITIONS:
#define           PPM_CHANNELS            8
#define           PPM_CHS_EXCLUDING_GIM   PPM_CHANNELS - 1 

#define           PPM_OUT_HIGH            PORTB |= 0x01                               // digitalWrite(PPM_Out_Pin, HIGH)   
#define           PPM_OUT_LOW             PORTB &= 0xFE                               // digitalWrite(PPM_Out_Pin, LOW) 
#define           PPM_PIN_STATE           PIND & 0x08                                 // digitalRead(PPM_In_Pin)

#define           ROLL_CH                 0
#define           PITCH_CH                1
#define           THROTTLE_CH             2
#define           YAW_CH                  3
#define           ARM_CH                  4
#define           SWITCH_CH               5

#define           AUX_CH                  6
#define           GIMBAL_CH               7

#define           MANUAL_PULSEWIDTH       1750
#define           FRAME_LENGTH            20000                                       // Set the PPM frame length in microseconds (1ms = 1000µs)
#define           PULSE_LENGTH            400                                         // Set the pulse length

#define           SERIAL_TIMEOUT          1000
#define           RECEIVER_TIMEOUT        1000

#define           PPM_In_Pin              3
#define           PPM_Out_Pin             8

// FUNCTION PROTOTYPES:
void Iterate_Manual_Mode();
void Iterate_Auto_Mode();
void Pitch_Forward();

// GLOBALS:
static struct {                                                                       // Store for PPM Pulsewidths
  volatile int Ch[PPM_CHANNELS];
  int Ch_Modify[PPM_CHANNELS];
} PPM_In;

int PPM_Out[PPM_CHANNELS];

unsigned long Serial_Rx_Timeout;

int Serial_X, Serial_Y;

Average FilterA(2);
Average FilterB(2);

double X_Dev_Setpoint, X_Dev_Input, X_Dev_Output;
PID X_Dev_PID(&X_Dev_Input, &X_Dev_Output, &X_Dev_Setpoint, 0.75, 0.0, 1.0, DIRECT);  

double  Y_Dev_Setpoint, Y_Dev_Input, Y_Dev_Output;
PID Y_Dev_PID(&Y_Dev_Input, &Y_Dev_Output, &Y_Dev_Setpoint, 5.0, 0.0, 0.0, REVERSE);


// Finished yesturday with 2.0, 0, 0 but that was overshooting
// Sunday:
// 1.0,0,0.25 worked quite well but still overshot
// 0.8,0,0.4 and set limits from 150 to 200 - pretty much working. Need a fixed altitude!
// 1.0,0.01,0.6 - changed X set point to 25.0 to make quad fly inside line. (must fly clockwise I think) - SET LAP TIMES WITH THIS!!
// 1.0,0.01,0.8 - oscillitory.



// 1.0,0,0 - divergent oscillations
// 0.5,0,0 - same but slower
// 3.0 - massive oscilations
// 1.5, 0.0, 0.5 - trying to compensate for undamped roll/acc response
//
// increased yaw p from 5 to 7.5 - yaw oscillations
// yaw p back to 5.0 changed set point from 25 to -25
// yaw to p=5.0, setpoint -25 to 50 cause it doesn't seem to be doing anything (optim probs between 5-6)
// yaw added 0.5 to d, setpoint from 50 to 25 (correct for clockwise flight)


