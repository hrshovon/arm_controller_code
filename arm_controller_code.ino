#include "defs_init.h"

int read_sensor_channel(uint8_t channel)
{
  uint8_t i=0;
  int adc_sum=0;
  for(i=0;i<10;i++)
  {
    adc_sum+=analogRead(channel);  
  }
  return adc_sum/10;
}


void convert_command_angles()
{
  //loop through the command angles and map them to adc values.
  if ((command_buffer[0]==210) && (command_buffer[COMMAND_LENGTH-1]==135))
  {
    for(uint8_t i=0;i<NUM_ANGLES;i++)
    {
      
      if(command_buffer[i+1]<=max_angle[i])
      {
        converted_angles[i]=map(command_buffer[i+1],0,max_angle[i],value_at_min_angle[i],value_at_max_angle[i]);  
      }
      else
      {
        converted_angles[i]=read_sensor_channel(current_state_channels[i]);  
      }
      //Serial.print(command_buffer[i+1]);
      //Serial.print(","); 
      //Serial.print(converted_angles[i]);
      //Serial.print(","); 
      //Serial.print(read_sensor_channel(current_state_channels[i]));
      //Serial.print(","); 
    }
  }
  //Serial.println();
}

void run_pid()
{
  int current_value=0;
  int error=0;
  int outputval=0;
  bool dir_motor=false;
  for(uint8_t i=0;i<NUM_ANGLES;i++)
  {
    if(en_pid[i]==false)
    {
      continue;
    }
    current_value=read_sensor_channel(current_state_channels[i]);
    error=converted_angles[i]-current_value;
    outputval=p_vals[i]*abs(error);
    
    if(outputval>255)
    {
      outputval=255;
    }
    if(error>=0)
    {
      dir_motor=max_dir[i];
    }
    else
    {
      dir_motor=!max_dir[i];
    }
    //Serial.print(error);
    //Serial.print(",");
    if(abs(error)>max_na_elim[i])
    {
      move_motor(i,dir_motor,outputval);  
    }
    else
    {
      stop_motor(i);
    }
    delay(SWITCH_DELAY);
  }
  //Serial.println();
}

void stop_motor(uint8_t motor_index)
{
  uint8_t d1_motor=D1_pins[motor_index];
  uint8_t d2_motor=D2_pins[motor_index];
  uint8_t pwm_motor=PWM_pins[motor_index];
  analogWrite(pwm_motor,0);
  digitalWrite(d1_motor,HIGH);
  digitalWrite(d2_motor,HIGH);
}

void move_motor(uint8_t motor_index,bool forward,uint8_t speed)
{
  uint8_t d1_motor=D1_pins[motor_index];
  uint8_t d2_motor=D2_pins[motor_index];
  uint8_t pwm_motor=PWM_pins[motor_index];
  digitalWrite(d1_motor,HIGH);
  digitalWrite(d2_motor,HIGH);
  //delay(100);
  analogWrite(pwm_motor,speed);
  if(forward==true)
  {
    digitalWrite(d1_motor,HIGH);
    digitalWrite(d2_motor,LOW);
  }
  else
  {
    digitalWrite(d1_motor,LOW);
    digitalWrite(d2_motor,HIGH);
  }
}

void setup() {
  delay(1000);
  // put your setup code here, to run once:
  pinMode(BASE_AND_ARM_STBY,OUTPUT);
  pinMode(BASE_MOTOR_D1,OUTPUT);
  pinMode(BASE_MOTOR_D2,OUTPUT);
  pinMode(ARM_MOTOR_D1,OUTPUT);
  pinMode(ARM_MOTOR_D2,OUTPUT);
  digitalWrite(BASE_AND_ARM_STBY,HIGH);
  
  pinMode(ELBOW_AND_WRIST_STBY,OUTPUT);
  pinMode(ELBOW_MOTOR_D1,OUTPUT);
  pinMode(ELBOW_MOTOR_D2,OUTPUT);
  pinMode(WRIST_MOTOR_D1,OUTPUT);
  pinMode(WRIST_MOTOR_D2,OUTPUT);
  digitalWrite(ELBOW_AND_WRIST_STBY,HIGH);

  pinMode(FINGER_AND_EXTRA_STBY,OUTPUT);
  pinMode(FINGER_MOTOR_D1,OUTPUT);
  pinMode(FINGER_MOTOR_D2,OUTPUT);
  pinMode(EXTRA_MOTOR_D1,OUTPUT);
  pinMode(EXTRA_MOTOR_D2,OUTPUT);
  digitalWrite(FINGER_AND_EXTRA_STBY,HIGH);

  Serial.begin(115200);
  Serial.setTimeout(100);
  /*
  move_motor(0,true,255);
  delay(2000);
  move_motor(0,false,255);
  delay(2000);
  stop_motor(0);
  move_motor(1,true,255);
  delay(2000);
  move_motor(1,false,255);
  delay(2000);
  stop_motor(1);
  move_motor(2,true,255);
  delay(2000);
  move_motor(2,false,255);
  delay(2000);
  stop_motor(2);
  move_motor(3,true,255);
  delay(2000);
  move_motor(3,false,255);
  delay(2000);
  stop_motor(3);
  move_motor(4,true,255);
  delay(2000);
  move_motor(4,false,255);
  delay(2000);
  stop_motor(4);
  */
}



void loop() {
  // put your main code here, to run repeatedly:
  //#ifdef EXTERNAL_CONTROL
  if(millis()-time_data>=100)
  {
    int base=read_sensor_channel(BASE_ADC);
    int arm=read_sensor_channel(ARM_ADC);
    int elbow=read_sensor_channel(ELBOW_ADC);
    int wrist=read_sensor_channel(WRIST_ADC);
    int finger=read_sensor_channel(FINGER_ADC);
    Serial.print(base);
    Serial.print(",");
    Serial.print(arm);
    Serial.print(",");
    Serial.print(elbow);
    Serial.print(",");
    Serial.print(wrist);
    Serial.print(",");
    Serial.println(finger);
    time_data = millis(); 
  }
  //#endif
  
  if(Serial.available())
  {
    #ifndef EXTERNAL_CONTROL
    byte_count=Serial.readBytesUntil((char)STOPBYTE,command_buffer,COMMAND_LENGTH);
    //Serial.println(byte_count);
    //cmd=Serial.read();
    if(byte_count==COMMAND_LENGTH)
    {
      convert_command_angles();
      command_set=true;
    }
    #else
    cmd=Serial.read();
    #endif
  }
  #ifndef EXTERNAL_CONTROL
  if(command_set==true)
  {
    run_pid();
  }
  #else
  
  switch(cmd)
  {
    case ROTATE_BASE_CW:
    move_motor(0,true,255);
    RESET_CMD;
    break;

    case ROTATE_BASE_CCW:
    move_motor(0,false,255);
    RESET_CMD;
    break;

    case STOP_BASE:
    stop_motor(0);
    RESET_CMD;
    break;

    case ROTATE_ARM_CW:
    move_motor(1,true,255);
    RESET_CMD;
    break;

    case ROTATE_ARM_CCW:
    move_motor(1,false,255);
    RESET_CMD;
    break;

    case STOP_ARM:
    stop_motor(1);
    RESET_CMD;
    break;

    case ROTATE_ELBOW_CW:
    move_motor(2,true,255);
    RESET_CMD;
    break;

    case ROTATE_ELBOW_CCW:
    move_motor(2,false,255);
    RESET_CMD;
    break;

    case STOP_ELBOW:
    stop_motor(2);
    RESET_CMD;
    break;

    case ROTATE_WRIST_CW:
    move_motor(3,true,255);
    RESET_CMD;
    break;

    case ROTATE_WRIST_CCW:
    move_motor(3,false,255);
    RESET_CMD;
    break;

    case STOP_WRIST:
    stop_motor(3);
    RESET_CMD;
    break;

    case FINGER_CLOSE:
    move_motor(4,false,255);
    RESET_CMD;
    break;

    case FINGER_OPEN:
    move_motor(4,true,255);
    RESET_CMD;
    break;

    case STOP_FINGER:
    stop_motor(4);
    RESET_CMD;
    break;
  }
  
  #endif
  
}
