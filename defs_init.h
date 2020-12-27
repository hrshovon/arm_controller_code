//#define EXTERNAL_CONTROL 1

#define NUM_ANGLES 4
#define COMMAND_LENGTH 7
#define STOPBYTE 245
#define RESET_CMD cmd=0;


#define ELBOW_ADC A2
#define ARM_ADC A1
#define WRIST_ADC A3
#define FINGER_ADC A4
#define BASE_ADC A0

#define BASE_AND_ARM_STBY 22
#define BASE_MOTOR_D1 24
#define BASE_MOTOR_D2 26
#define BASE_PWM 7

#define ARM_MOTOR_D1 30
#define ARM_MOTOR_D2 28
#define ARM_PWM 8


#define ELBOW_AND_WRIST_STBY 32
#define ELBOW_MOTOR_D1 34
#define ELBOW_MOTOR_D2 36
#define ELBOW_PWM 9

#define WRIST_MOTOR_D1 38
#define WRIST_MOTOR_D2 40
#define WRIST_PWM 10

#define FINGER_AND_EXTRA_STBY 42
#define FINGER_MOTOR_D1 44
#define FINGER_MOTOR_D2 46
#define FINGER_PWM 12

#define EXTRA_MOTOR_D1 48
#define EXTRA_MOTOR_D2 50
#define EXTRA_PWM 12

uint8_t D1_pins[]={BASE_MOTOR_D1,ARM_MOTOR_D1,ELBOW_MOTOR_D1,WRIST_MOTOR_D1,FINGER_MOTOR_D1,EXTRA_MOTOR_D1};
uint8_t D2_pins[]={BASE_MOTOR_D2,ARM_MOTOR_D2,ELBOW_MOTOR_D2,WRIST_MOTOR_D2,FINGER_MOTOR_D2,EXTRA_MOTOR_D2};
uint8_t PWM_pins[]={BASE_PWM,ARM_PWM,ELBOW_PWM,WRIST_PWM,FINGER_PWM,EXTRA_PWM};


uint8_t byte_count=0;
uint8_t command_buffer[20];
int converted_angles[NUM_ANGLES];
int current_state_channels[NUM_ANGLES]={BASE_ADC,ARM_ADC,ELBOW_ADC,WRIST_ADC};
int value_at_max_angle[NUM_ANGLES]={2,93,600,189};
int value_at_min_angle[NUM_ANGLES]={1004,474,987,266};
int max_angle[NUM_ANGLES]={240,90,90,180};
int p_vals[NUM_ANGLES]={8,25,20,8};
int max_na_elim[NUM_ANGLES]={4,1,1,1};
bool en_pid[NUM_ANGLES]={true,true,true,false};
int max_dir[NUM_ANGLES]={true,true,true,false};
bool command_set=false;
byte cmd=0;
int time_data = 0;
#define NO_CMD 250
#define SWITCH_DELAY 5
#define ROTATE_BASE_CW 97
#define ROTATE_BASE_CCW 98
#define STOP_BASE 99

#define ROTATE_ARM_CW 100
#define ROTATE_ARM_CCW 101
#define STOP_ARM 102

#define ROTATE_ELBOW_CW 103
#define ROTATE_ELBOW_CCW 104
#define STOP_ELBOW 105

#define ROTATE_WRIST_CW 106
#define ROTATE_WRIST_CCW 107
#define STOP_WRIST 108

#define FINGER_CLOSE 109
#define FINGER_OPEN 110
#define STOP_FINGER 111
