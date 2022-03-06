/*
By DrakerDG
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx
*/

#include <AFMotor.h>
#include <Servo.h> 

// DC motor on M4 and M3
AF_DCMotor motorR(4);
AF_DCMotor motorL(3);

// DC hobby servo
Servo servoU;

void setup() {
  // turn on servo
  servoU.attach(9);
   
  // turn on motor #2 and #3
  motorR.setSpeed(200);
  motorR.run(RELEASE);
  motorL.setSpeed(200);
  motorL.run(RELEASE);
}

int i, j;

// Test the DC motors and servo
void loop() {
  motorR.run(FORWARD);
  motorL.run(FORWARD);
  for (i=0; i<255; i++) {
    j = map(i, 0, 255, 0, 180); 
    servoU.write(j);
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(3);
 }
 
  for (i=255; i!=0; i--) {
    j = map(i, 0, 255, 0, 180); 
    servoU.write(j);
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(3);
 }
 
  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  for (i=0; i<255; i++) {
    j = map(i, 0, 255, 0, 180); 
    servoU.write(j);
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(3);
  }
 
  for (i=255; i!=0; i--) {
    j = map(i, 0, 255, 0, 180); 
    servoU.write(j);
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(3);
 }
}
