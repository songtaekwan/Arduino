int echo = 3;     //IO 핀번호
int trig = 2;       //IO 핀번호
unsigned long duration;
float distance;
void setup() {
  //Initialize serial and wait for port to open:
   Serial.begin(9600);                //시리얼 셋팅
    pinMode(echo, INPUT);       //초음파 input설정
    pinMode(trig, OUTPUT);      //초음파 out설정
    while (!Serial) {
   }
  
   // prints title with ending line break
   Serial.println( "------- Start ------- ");
}
void loop() {
  char i;
  duration = 0;
  for( i=0 ; i<10 ; i++)
  {
    // Trigger Signal output
    digitalWrite(trig, HIGH);   // trigpin에 High를 출력
     delay(10);
    digitalWrite(trig, LOW);  // // trigpin에 Low를 출력
   
     // Echo Signal Input
    duration += pulseIn(echo, HIGH); //echoPin에서 펄스값을 받아온다
     delay(10);
  }
  duration = duration/10;
 
  // Calcurate distance
  distance = ((float)(340*duration)/10000) / 2;
  Serial.print(distance);
  Serial.println("cm");
  delay(100); 
}
