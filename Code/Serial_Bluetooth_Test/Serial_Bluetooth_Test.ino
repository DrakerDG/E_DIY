/*
By DrakerDG (c)

Follow me:
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx
*/

#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Servo.h>

// Operation mode
// Modo de operación
char BT_cmd;
int mode = 1;

// DC motor on M4 and M3
AF_DCMotor motorR(4);
AF_DCMotor motorL(3);

// DC hobby servo
Servo servoU;

// Oun Softeware Serial
SoftwareSerial myBT(19, 18); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  // Abre la comunicación del puerto serial y espera a que se abra el puerto:
  Serial.begin(57600);
  while (!Serial) {;}

  Serial.println("Bluetooth Test!");

  // set the data rate for the SoftwareSerial port
  myBT.begin(9600);

  // turn on servo
  servoU.attach(9);
   
  // turn on motor #4 and #3
  motorR.setSpeed(200);
  motorR.run(RELEASE);
  motorL.setSpeed(200);
  motorL.run(RELEASE);

}

void loop() { // run over and over
//  BT_cmd = myBT.read();
  if (myBT.available()) {
    BT_cmd = myBT.read();
//    Serial.write(BT_cmd);
  }
  if (BT_cmd == '1') {
    Serial.println("Line Follower Mode");
    mode = 1;
  }
  else if (BT_cmd == '2') {
    Serial.println("Obstacle Aboider Mode");
    mode = 2;
  }
  else if (BT_cmd == '3') {
    Serial.println("Bluetooth Mode");
    mode = 3;
  }

  if (mode == 2) {
    if (BT_cmd == 'C') {
      Serial.println("Servo Left");
      servoU.write(255);
    }
    else if (BT_cmd == 'D') {
      Serial.println("Servo Right");
      servoU.write(0);
    }
    else if ((BT_cmd == 'A') || (BT_cmd == 'B')) {
      Serial.println("Servo Center");
      servoU.write(90);
    }
  }

  if (mode == 3) {
    if (BT_cmd == 'A') {
      Serial.println("Foward");
      motorR.run(FORWARD);
      motorL.run(FORWARD);
      motorR.setSpeed(200);  
      motorL.setSpeed(200);  
    delay(3);

    }
    else if (BT_cmd == 'B') {
      Serial.println("Reverse");
      motorR.run(BACKWARD);
      motorL.run(BACKWARD);
      motorR.setSpeed(200);  
      motorL.setSpeed(200);  
    }
    else if (BT_cmd == 'C') {
      Serial.println("Left");
      motorR.run(FORWARD);
      motorL.run(BACKWARD);
      motorR.setSpeed(200);  
      motorL.setSpeed(200);  
    }
    else if (BT_cmd == 'D') {
      Serial.println("Right");
      motorR.run(BACKWARD);
      motorL.run(FORWARD);
      motorR.setSpeed(200);  
      motorL.setSpeed(200);  
    }
    else if (BT_cmd == 'Z') {
      Serial.println("Stop");
      motorR.run(RELEASE);
      motorL.run(RELEASE);
    }
  }

  BT_cmd = '\n';

}
