/*
By DrakerDG (c)

Follow me:
Youtube: https://youtube.com/user/DrakerDG
Facebook: https://www.facebook.com/DrakerDG
Instagram: https://www.instagram.com/drakerdgx
*/

const int trig1 = 15;
const int echo1 = 14;
long Duration1;
int Distance1;

void setup() { 
  Serial.begin(9600);
  pinMode (trig1, OUTPUT);  
  pinMode (echo1,INPUT);

}

void loop() {
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  Duration1 = pulseIn(echo1, HIGH);
  Distance1 = Duration1*0.03432/2;

  Serial.print(Distance1);
  Serial.println(" centimeters");
}
