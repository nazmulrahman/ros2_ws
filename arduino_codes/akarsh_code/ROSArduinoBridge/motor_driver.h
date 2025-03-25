/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/


#ifdef BLD30A_DRIVER
  #define RIGHT_MOTOR_DIRECTION 8
  #define LEFT_MOTOR_DIRECTION  7
  
  #define RIGHT_MOTOR_ENABLE 9
  #define LEFT_MOTOR_ENABLE 6

  #define RIGHT_MOTOR_PWM  10
  #define LEFT_MOTOR_PWM   5
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
