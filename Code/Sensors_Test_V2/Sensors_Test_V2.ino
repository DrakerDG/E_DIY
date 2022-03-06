/*
By DrakerDG (c)

Follow me:
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx
*/

#include <AFMotor.h>

// DC motor on M3 and M4
AF_DCMotor motorR(4);
AF_DCMotor motorL(3);

// Sensor Pins
const byte pSen[2] = {17, 16};

int sen_max[] = {300,300};
int sen_min[] = {300,300};
int sen_val[] = {0,0};
const int KS = 1000;
const int maxSR = 140;
const int maxSL = 140;
int speedR = 0;
int speedL = 0;
int online = 0;

/*
Speed and PID Key Setings:

Speed = 160
Kp = 0.12
Ki = 0.0001
Kd = 0.001
*/

// PID constants
float Kp = 0.12; // 0.12;
float Ki = 0.0001; // 0.0001;
float Kd = 0.001; // 0.001;
long P=0, I=0, D=0, PID=0, P_old=0;

int i = 0;

void set_IRS(void);
long get_IRS(long);
void get_PID(void);

void setup() {
  Serial.begin(9600);
  while (!Serial) {;}
  
  // turn on motor #4 and #3
  motorR.setSpeed(maxSR);
  motorR.run(RELEASE);
  motorL.setSpeed(maxSL);
  motorL.run(RELEASE);
  
  set_IRS();

  motorR.run(FORWARD);
  motorL.run(FORWARD);
}

void loop() {
  get_PID();
  motorR.setSpeed(speedR);  
  motorL.setSpeed(speedL);
}

void set_IRS(){
  int senx = 0;
  tone(10, 1760);
  delay(500);
  noTone(10);
  for (int j = 0; j < 30000; j++){
    for (i = 0; i < 2; i++){
      senx = analogRead(pSen[i]);
      if (senx < sen_min[i]) sen_min[i] = senx;
      if (senx > sen_max[i]) sen_max[i] = senx;
    }  
  }
  tone(10, 1760);
  delay(500);
  noTone(10);
  delay(50);
  tone(10, 880);
  delay(500);
  noTone(10);
  delay(3000);

  // View max min IR values
/*  
  char DataX[100];
  sprintf(DataX,"MinL: %4d   maxSL: %4d   MinR: %4d   maxSR: %4d", sen_min[0], sen_max[0],  sen_min[1], sen_max[1]);
  Serial.println(DataX);
*/

}

long get_IRS(long Pant){
  online = 0;
  unsigned long Perr;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
   
  for(i = 0; i < 2; i++){
    sen_val[i] = analogRead(pSen[i]);
    sen_val[i] = map(sen_val[i], sen_min[i], sen_max[i], 0, KS);
    sen_val[i] = constrain(sen_val[i], 0, KS);
    if (sen_val[i]>150) online = 1;
    avgS += (long)sen_val[i]*(i*KS);
    sumS += sen_val[i];
  }
  if (online == 1) Perr = avgS/sumS - 500;

  // Only to sensors test
/**/
  char DataX[100];
  sprintf(DataX,"L: %4d   R: %4d   P: %4d", sen_val[0], sen_val[1], Perr);
  Serial.println(DataX);
/**/
  return Perr;
  
}

void get_PID(void){
  int medR, medL;
  P = get_IRS(P);
  I += P;
  D = D * 0.5 + P - P_old;

  PID = Kp * P + Ki * I + Kd * D;

  P_old = P;

  medR = maxSR - abs(PID);
  medL = maxSL - abs(PID);
  speedR = medR - PID;
  speedL = medL + PID;

  // Only to sensors test //
/*  
  char DataX[100];
  sprintf(DataX,"SP_L: %4d   SP_R: %4d   PID: %4d", speedL, speedR, PID);
  Serial.println(DataX);
*/
}
