/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef BLD30A_DRIVER
  void initMotorController() {
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
    pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);

    digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
    digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  }


  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) {

      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_DIRECTION, LOW); analogWrite(LEFT_MOTOR_PWM, spd); }
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_DIRECTION, HIGH); analogWrite(LEFT_MOTOR_PWM, spd); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {

      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_DIRECTION, LOW); analogWrite(RIGHT_MOTOR_PWM, spd); }
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_DIRECTION, HIGH); analogWrite(RIGHT_MOTOR_PWM, spd); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif
