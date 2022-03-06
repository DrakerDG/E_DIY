/*
By DrakerDG (c)

Follow me:
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx
*/

// Sensor Pins
const byte pSen[2] = {17, 16};

int sen_max[] = {300,300};
int sen_min[] = {300,300};
int sen_val[] = {0,0};
#define KS 1000
int online = 0;

// Base Speed Motors 
#define SpBase 140 //140

// PID constants
float Kp = 0; // 0.020;
float Ki = 0; // 0.001;
float Kd = 0; // 0.010;
long P=0, I=0, D=0, PID=0, P_old=0;

int i = 0;

void sensor_cal(void);
void sensor_val(void);

void setup() {
  Serial.begin(9600);
  while (!Serial) {;}
  sensor_cal();
}

void loop() {
  sensor_val();

}

void sensor_cal(){
  int senx = 0;
  for (int j = 0; j < 30000; j++){
    if ((j % 1000) == 0) tone(10, 1000);
    else if ((j % 500) == 0) noTone(10);
    for (i = 0; i < 2; i++){
      senx = analogRead(pSen[i]);
      if (senx < sen_min[i]) sen_min[i] = senx;
      if (senx > sen_max[i]) sen_max[i] = senx;
    }  
  }
  noTone(10);
}

void sensor_val(){
  online = 0;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
   
  for(i = 0; i < 2; i++){
    sen_val[i] = analogRead(pSen[i]);
    sen_val[i] = map(sen_val[i], sen_min[i], sen_max[i], 0, KS);
    sen_val[i] = constrain(sen_val[i], 0, KS);
    if (sen_val[i]>500) online = 1;
    if (sen_val[i]>50){
      avgS += (long)sen_val[i]*(i*KS);
      sumS += sen_val[i];
    }
  }
  if (online) P = avgS/sumS - 500;
  
  else if(P < 0.5) P = -500;
  else P = 500;

  // Only to sensors test //
  char DataX[100];
  sprintf(DataX,"L: %4d   R: %4d   P: %4d", sen_val[0], sen_val[1], P);
  Serial.println(DataX);
}
