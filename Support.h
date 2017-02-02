// DEFINITIONS:
#define           PPM_CHANNELS            8 

#define           PITCH_CH                0
#define           ROLL_CH                 1
#define           THROTTLE_CH             2
#define           YAW_CH                  3
#define           AUX_CH                  4
#define           SWITCH_CH               5

#define           CENTRE_PULSEWIDTH       1500
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
} PPM_Channel;

int PPM[PPM_CHANNELS];








