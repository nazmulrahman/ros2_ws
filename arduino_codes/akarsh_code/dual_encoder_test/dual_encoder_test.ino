#define leftEncoderPinA 18 //HA
#define leftEncoderPinB 19 //HB

#define rightEncoderPinA 20 //HB
#define rightEncoderPinB 21 //HA

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

char chr;

void setup() {
  Serial.begin(57600);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), change_right_b, CHANGE);
}

void loop() {

  Serial.print(encoder0Pos);
  Serial.print(" ");
  Serial.println(encoder1Pos);
  delay(33);
  // if(Serial.available() > 0)
  // {
  //   chr = Serial.read();
    
  //   if (chr == 'e')
  //   {
  //     Serial.print(encoder0Pos);
  //     Serial.print(" ");
  //     Serial.println(encoder1Pos);
  //     delay(33);
  //   }
  //   else if (chr == 'r')
  //   {
  //     encoder0Pos = 0;
  //     encoder1Pos = 0;
  //   }
  // }
}

// ************** encoders interrupts **************

void change_left_a(){  
  if (digitalRead(leftEncoderPinA) == HIGH) { 
    if (digitalRead(leftEncoderPinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  } else { 
    if (digitalRead(leftEncoderPinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
}

void change_left_b(){  
  if (digitalRead(leftEncoderPinB) == HIGH) {   
    if (digitalRead(leftEncoderPinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  } else { 
    if (digitalRead(leftEncoderPinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
}

void change_right_a(){  
  if (digitalRead(rightEncoderPinA) == HIGH) { 
    if (digitalRead(rightEncoderPinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;
    } else {
      encoder1Pos = encoder1Pos + 1;
    }
  } else { 
    if (digitalRead(rightEncoderPinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;
    } else {
      encoder1Pos = encoder1Pos + 1;
    }
  }
}

void change_right_b(){  
  if (digitalRead(rightEncoderPinB) == HIGH) {   
    if (digitalRead(rightEncoderPinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;
    } else {
      encoder1Pos = encoder1Pos + 1;
    }
  } else { 
    if (digitalRead(rightEncoderPinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;
    } else {
      encoder1Pos = encoder1Pos + 1;
    }
  }
}
