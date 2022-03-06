/*
Kit_Smart_Car_2WD_Plus_V2 28-02-2022
    ____             __             ____  ______
   / __ \_________ _/ /_____  _____/ __ \/ ____/
  / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
 / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
/_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
                                                
By DrakerDG (c)
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx

* Bluetooth Controller: SoftwareSerial.h library
* Motor Shield L293D V1.2: AFMotor.h library
* Servo motor: Servo.h library
*/

#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Servo.h>

// Own Software Serial Instance
SoftwareSerial myBT(19, 18); // RX, TX

// Operation mode - Modo de operación
char BT_cmd;
int mode = 1;

// DC motor on M4 and M3
AF_DCMotor motorR(4);
AF_DCMotor motorL(3);

// DC servo motor
Servo servoU;

int on_Robot = 0;

// Number of sensors to use
#define IR 8

// Sensor Pins
const byte pSen[IR] = {17, 16};

// Line Sensors Limits
int sen_max[] = {300,300};
int sen_min[] = {300,300};
int sen_val[] = {0,0};
const int KS = 1000;
int cal_IRS = 0;
int cal_TME = 0;
int online = 0;

/*
Speed and PID Setings:
Speed = 255/240
Kp = 0.13
Ki = 0.0001
Kd = 0.001
*/

// PID constants
#define Kp 0.13
#define Ki 0.0001
#define Kd 0.001
float P_error;
float I_error;
float D_error;
float PID_value;
float lastError;
unsigned long cTime;
unsigned long pTime;
float eTime;

unsigned long TimeE0 = 0;
unsigned long TimeE1;


// Front limits
#define lim1 45
#define vec1 -10
#define vec2 -9

// Max speed motors
#define maxSR 255
#define maxSL 240

// Proportional error key
#define Ke 1.5
// Time to stop (miliseconds)
#define Stop 200
// Time to turn (miliseconds)
#define Turn 350
int speedR = 0;
int speedL = 0;

// Time for servo movement
#define period 20
// Servomotor step
#define lap1 9
// Counter step 
int i = 10;
// Direction spin
int CCW = 1;

// Ultrasonic pins
#define trig1 15
#define echo1 14

// Distance Objects Array
int distances[21];
// Detection Object Vector
int xVec = 0;
int zVec = 0;

// Status robot
int avoidSR = 0;

// Tone Timers
unsigned long Time0 = 0;
unsigned long Time1;

// Modules and functions declarations
 int get_DIS(void);
void set_INI(void);
void set_IRS(void);
long get_IRS(long);
void get_PID(void);
void set_VEC(void);
void uss_RAD(void);
void get_VEC(void);
void snd_BZR(int) ;
void set_SPD(void);
void run_FWD(void);
void run_BWD(void);
void run_STP(void);
void run_TRN(int);

void setup() {
  /*
  // Open serial communications and wait for port to open (To test mode)
  Serial.begin(9600);
  while (!Serial) {;}
  Serial.println("Robot Ready!");
  */

  // set the data rate for the SoftwareSerial port
  myBT.begin(9600);

  // turn on servo and aling to the front
  servoU.attach(9);
  servoU.write(i * lap1);
  delay(100);
     
  // turn on motor #4 and #3
  motorR.setSpeed(200);
  motorR.run(RELEASE);
  motorL.setSpeed(200);
  motorL.run(RELEASE);

  // Ultrasonic Sensor Pins
  pinMode (trig1, OUTPUT);  
  pinMode (echo1,INPUT);

  // Initialice vector array
  set_VEC();

}

void loop() {
  if (myBT.available()) BT_cmd = myBT.read();

  // Line Follower Mode Sellected
  if (BT_cmd == '1') {
    mode = 1;
    set_INI();
  }

  // Obstacle Avoider Mode Sellected
  else if (BT_cmd == '2') {
    mode = 2;
    set_INI();
  }

  // Bluetooth Control Mode Sellected
  else if (BT_cmd == '3') {
    mode = 3;
    set_INI();
  }
  
  // Line Follower Mode
  if (mode == 1) {
    if ((BT_cmd == 'A') && (cal_IRS == 1)) {
      on_Robot = 1;
      snd_BZR(3);
      motorR.run(FORWARD);
      motorL.run(FORWARD);
    }
    else if (BT_cmd == 'B') {
      snd_BZR(3);
      on_Robot = 0;
      run_STP();
    }
    else if ((BT_cmd == 'C') && (cal_IRS == 0)) {
      speedR = 160;
      speedL = 160;
      snd_BZR(3);
      run_TRN(1);
      set_IRS();
    }
    else if ((BT_cmd == 'D') && (cal_IRS == 0)) {
      speedR = 160;
      speedL = 160;
      snd_BZR(3);
      run_TRN(-1);
      set_IRS();
    }
    else if ((BT_cmd == 'Z') && (cal_IRS == 0)) {
      run_STP();
    }
    if (on_Robot == 1) {
      get_PID();
      motorR.setSpeed(speedR);  
      motorL.setSpeed(speedL);
    }
  }

  // Obstacle Avoider Mode
  else if (mode == 2) {
    if (BT_cmd == 'A') {
      snd_BZR(3);
      on_Robot = 1;
    }
    else if (BT_cmd == 'B') {
      snd_BZR(3);
      on_Robot = 0;
      run_STP();
    }
    if (on_Robot == 1) {
      uss_RAD();
      if (avoidSR == 0) {
        set_SPD();
        run_FWD();
      }
      else if (avoidSR == 1) {
        run_STP();
        delay(Stop);
        avoidSR = 2;
      }
      else if (avoidSR == 2) {
        run_BWD();
      }
      else if (avoidSR == 3) {
        run_TRN(xVec/abs(xVec));
        delay(Turn);
        avoidSR = 0;
      }
    }
  }

  // Bluetooth Control Mode
  else if (mode == 3) {
    if (BT_cmd == 'A') {
      snd_BZR(3);
      speedR = maxSR;
      speedL = maxSL;
      run_FWD();  
    }
    else if (BT_cmd == 'B') {
      snd_BZR(3);
      run_BWD();  
    }
    else if (BT_cmd == 'C') {
      snd_BZR(3);
      run_TRN(1); 
    }
    else if (BT_cmd == 'D') {
     snd_BZR(3);
     run_TRN(-1);
    }
    else if (BT_cmd == 'Z') {
      run_STP();
    }
  }

  BT_cmd = '\n';
}

// Get distance ultasonic sensor value
int get_DIS(void){
  long Duration1;
  int Distance1;
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  Duration1 = pulseIn(echo1, HIGH);
  Distance1 = Duration1*0.03432/2;
  return Distance1;
}

// Set initial status robot
void set_INI(void) {
  on_Robot = 0;
  servoU.write(90);
  snd_BZR(5);
}

// Set infrared sensor values (calibration)
void set_IRS(void){
  int senx = 0;
  for (int k = 0; k < 2; k++){
    senx = analogRead(pSen[k]);
    if (senx < sen_min[k]) sen_min[k] = senx;
    if (senx > sen_max[k]) sen_max[k] = senx;
  }
  cal_TME += 1;
  /*
  char DataX[100];
  sprintf(DataX,"T: %i   min0: %i   max0: %i   min1: %i   max1: %i", cal_TME, sen_min[0], sen_max[0], sen_min[1], sen_max[1]);
  Serial.println(DataX);
  */
  if ((cal_TME > 20) && (sen_max[0] > sen_min[0]) && (sen_max[1] > sen_min[1])) {
    run_STP();
    snd_BZR(3);
    delay(500);
    noTone(10);
    delay(50);
    snd_BZR(10);
    delay(500);
    noTone(10);
    cal_IRS = 1;
  }
}

// Get infrared sensor values
long get_IRS(long Pant){
  online = 0;
  unsigned long Perr;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
   
  for(int k = 0; k < 2; k++){
    sen_val[k] = analogRead(pSen[k]);
    sen_val[k] = map(sen_val[k], sen_min[k], sen_max[k], 0, KS);
    sen_val[k] = constrain(sen_val[k], 0, KS);
    if (sen_val[k]>150) online = 1;
    avgS += (long)sen_val[k]*(k*KS);
    sumS += sen_val[k];
  }
  if (online == 1) Perr = avgS/sumS - 500;
  else Perr = Pant;

  delayMicroseconds(140);
  
  /*
  // Only to sensors test
  char DataX[100];
  sprintf(DataX,"L: %4d   R: %4d   P: %4d", sen_val[0], sen_val[1], Perr);
  Serial.println(DataX);
  */
  return Perr;
}

// Get PID calculation error
void get_PID(void){
  int medR, medL;
  P_error = get_IRS(P_error);
  cTime = millis();
  eTime = (float)(cTime - pTime) / 1000;
  I_error = I_error * 2 / 3 + P_error * eTime;
  D_error = (P_error - lastError) / eTime;
  PID_value = Kp * P_error + Ki * I_error + Kd * D_error;

  lastError = P_error;

  medR = maxSR - abs(PID_value);
  medL = maxSL - abs(PID_value);
  speedR = medR - PID_value;
  speedL = medL + PID_value;

  /*
  // Only to sensors test
  char DataX[100];
  sprintf(DataX,"P: %5d   I: %5d   D: %5d   *  PID: %5d", P * Kp, I * Ki, D * Kd, PID);
  Serial.println(DataX);
  */
 
}

// Set distance vector array
void set_VEC(void){
  for (int j = 0; j <= 20; j++){
    distances[j] = lim1;
  }
}

// Ultrasonic sensor scanner
void uss_RAD(void){
  servoU.write(i * lap1);
  delay(period);
  distances[i] = get_DIS();
  get_VEC();
  if (distances[i] < lim1) snd_BZR(distances[i]);

  if ((zVec < vec1) and (avoidSR == 0)) avoidSR = 1;  // Stop robot
  if ((zVec > vec2) and (avoidSR == 2)) avoidSR = 3;  // Turn robot
  
  i = i + CCW;
  if (i < 0){
    i = 1;
    CCW = 1;
  }
  if (i > 20){
    i = 19;
    CCW = -1;
  }
}

// Distance vector calculator
void get_VEC(void){
  long vectx = 0;
  long vectz = 0;
  int objects = 0;
  for (int j = 0; j <= 20; j++){
    if (distances[j] < lim1){
      vectx += sin((j - 10) * lap1 * PI / 180) * (distances[j] - lim1);
      vectz += cos((j - 10) * lap1 * PI / 180) * (distances[j] - lim1);
      objects += 1;
    }
  }
  if (objects > 0) {
     xVec = vectx / objects;
     zVec = vectz / objects;
  }
  else {
    xVec = 0;
    zVec = 0;
  }
}

// Sound buzzer
void snd_BZR(int distx) {
  int Interval;
  Time1 = millis();
  Interval = 15 * distx;
  Interval = constrain(Interval, 50, 2500);
  if (Time1 - Time0 >= Interval) {
    Time0 = Time1;
    tone(10, 1760 + 1320 - distx * 66, 25);
  }
}

// Set speed DC motors
void set_SPD(void) {
  int errP = int(xVec * Ke);
  speedR = maxSR - abs(errP);
  speedL = maxSL - abs(errP);
  speedR = speedR + errP;
  speedL = speedL - errP;
  speedR = constrain(speedR, 0, 255);
  speedL = constrain(speedL, 0, 255);
}

// Run forward
void run_FWD(void) {
  motorR.run(FORWARD);
  motorL.run(FORWARD);
  motorR.setSpeed(speedL);
  motorL.setSpeed(speedL);
  delay(3);
}

// Run stop
void run_STP(void) {
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  delay(3);
}

// Run backward
void run_BWD(void) {
  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  motorR.setSpeed(maxSR);  
  motorL.setSpeed(maxSL);  
  delay(3);
}

// Run turn (left or right)
void run_TRN(int CW) {
  if (CW > 0){
    motorR.run(FORWARD);
    motorL.run(BACKWARD);
  }
  else {
    motorR.run(BACKWARD);
    motorL.run(FORWARD);
  }
  motorR.setSpeed(maxSR);  
  motorL.setSpeed(maxSL);
  delay(3);
}
