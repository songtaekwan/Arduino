int cds = A1;
int LED1 = 13;

void setup()
{
  Serial.begin(9600); 
  pinMode(cds,INPUT);
  pinMode(LED1,OUTPUT);
}

void loop()
{
  cds = analogRead(A1);
  Serial.println(cds);
  if(cds<500)
  { 
    digitalWrite(LED1,LOW);
  }
  else
  {
    digitalWrite(LED1,HIGH);
  }
  delay(100);
}
