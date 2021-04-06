#include <Arduino.h>
#include "Timer.h"

Timer timer(1000);


int ledPin = 13;
enum States {
  on,
  off
  } state;

void blink() {
  switch(state) {
    case on:
    digitalWrite(ledPin, HIGH);
    if(timer.isExpired()) state = off;
    break;
    case off:
    digitalWrite(ledPin, LOW);
    if(timer.isExpired()) state = on;
    break;
      
  }
}


void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  blink();
}