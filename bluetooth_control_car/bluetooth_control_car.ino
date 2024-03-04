char value; 

void setup() {
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  Serial.begin(9600);
}
void forward()
{
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
}
void backward()
{
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
}
void right()     // Sharp right trun
{
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
}
void left()      //sharp left trun
{
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
}
void stops()
{

 digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  }

void loop() {
  if(Serial.available()>0){

    value = Serial.read();
    Serial.println(value);
    if(value=='F')
    {
      right();
    
    }
    if(value=='B')
    {
      right();
    
    }
    if(value=='R')
    {
      right();
    
    }
    if(value=='L')
    {
      right();
    
    }  
    if(value=='S')
    {
      stops();
    }
  }
}
