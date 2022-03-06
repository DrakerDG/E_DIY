/*
By DrakerDG (c)
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

// Fronts limits
const int lim1 = 45;
const int vec1 = -10;
const int vec2 = -9;

// Max speed motors
const int maxSL = 180;
const int maxSR = 200;
// Proportional error key
const float Kp = 1.5;
// Time to stop (miliseconds)
const int Stop = 200;
// Time to turn (miliseconds)
const int Turn = 350;
int speedL = 0;
int speedR = 0;

// Time for servo movement
const int period = 20;
// Servomotor step
const int lap1 = 9;
// Counter step 
int i = 10;
// Direction spin
int CCW = 1;

// Ultrasonic pins
const int trig1 = 15;
const int echo1 = 14;

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

// modules and functions
 int get_DIS(void);
void set_VEC(void);
void uss_RAD(void);
void get_VEC(void);
void snd_BZR(int);
void set_SPD(void);
void run_FWD(void);
void run_BWD(void);
void run_STP(void);
void run_TRN(int);

void setup() {
   
  Serial.begin(9600);
  while (!Serial) {;}
  
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
  uss_RAD();

  char DataX[100];
  sprintf(DataX,"Status: %i   Z: %i   X: %i", avoidSR, zVec, xVec);
  Serial.println(DataX);

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

void set_VEC(void){
  for (int j = 0; j <= 20; j++){
    distances[j] = lim1;
  }
}


int get_DIS(){
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

void set_SPD(void) {
  int errP = int(xVec * Kp);
  speedR = maxSR - abs(errP);
  speedL = maxSL - abs(errP);
 
  speedR = speedR + errP;
  speedL = speedL - errP;


}

void run_FWD(void) {
  /*  
  // Testing forward speed motors
  char DataX[100];
  sprintf(DataX,"Z: %i   X: %i   R: %i   L: %i", zVec, xVec, speedR, speedL);
  Serial.println(DataX);
  */
  
  motorR.run(FORWARD);
  motorL.run(FORWARD);
  motorR.setSpeed(speedL);
  motorL.setSpeed(speedL);
  delay(3);
}

void run_STP(void) {
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  delay(3);
}

void run_BWD(void) {
  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  motorR.setSpeed(maxSR);  
  motorL.setSpeed(maxSL);  
  delay(3);
}

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
