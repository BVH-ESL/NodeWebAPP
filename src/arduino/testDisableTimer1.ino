#include <Arduino.h>

int count = 0;
static unsigned long stateTime = millis();

void countData(){
  if(count == 10){
    timer1_detachInterrupt();
  }else{
    count++;
  }
}

void setup(){
  Serial.begin(921600);
  timer1_isr_init();
  timer1_attachInterrupt(reinterpret_cast<timercallback>(countData));
  timer1_enable(TIM_DIV16, TIM_EDGE, 1);    //TIM_DIV16 -> 5MHz = 5 ticks/us, TIM_DIV1 -> 80MHz = 80 ticks/us
  timer1_write(10000);                       //call interrupt after ... tick
}

void loop(){
  if(millis() - stateTime > 100){
    Serial.println(count);
    count = 0
    timer1_attachInterrupt(reinterpret_cast<timercallback>(countData));
  }
}
