#define btn (4)
#define led1 (0)
#define led2 (2)

void trigger_isr(){
  if(digitalRead(btn)){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
  }
}

void setup(){
  pinMode(btn, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  attachInterrupt( btn, trigger_isr, FALLING );
}

void loop(){

}
