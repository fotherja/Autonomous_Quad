// DEFINITIONS:
#define           PPM_CHANNELS            8 

#define           PITCH_CH                0
#define           ROLL_CH                 1
#define           THROTTLE_CH             2
#define           YAW_CH                  3
#define           AUX_CH                  4
#define           SWITCH_CH               5

#define           MANUAL_PULSEWIDTH       1750
#define           FRAME_LENGTH            20000                                       // Set the PPM frame length in microseconds (1ms = 1000µs)
#define           PULSE_LENGTH            400                                         // Set the pulse length

#define           GIMBAL_SWIVEL_TIME      2000

#define           PPM_In_Pin              3
#define           PPM_Out_Pin             8

// FUNCTION PROTOTYPES:
void Print_PPM_Channel_Values();
void Iterate_Manual_Mode();
void Iterate_Auto_Mode();

// GLOBALS:
static struct {                                                                       // Store for PPM Pulsewidths
  volatile int Ch[PPM_CHANNELS];
  int Ch_Copy[PPM_CHANNELS];
} PPM_In;

int PPM_Out[PPM_CHANNELS];

int Serial_X, Serial_Y;

double X_Dev_Setpoint, X_Dev_Input, X_Dev_Output;
PID X_Dev_PID(&X_Dev_Input, &X_Dev_Output, &X_Dev_Setpoint, 1.0, 0.5, 0.0, DIRECT);

double  Y_Dev_Setpoint, Y_Dev_Input, Y_Dev_Output;
PID Y_Dev_PID(&Y_Dev_Input, &Y_Dev_Output, &Y_Dev_Setpoint, 1.0, 0.5, 0.0, DIRECT);





