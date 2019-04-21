#include <stdlib.h>
#include <unistd.h>
#include "DumpedCurrentMotor_test2_ino.h"

int valo;
Motor M1;

void setup()
{
  Serial.begin(9600);
  
  cli();                                      //stop interrupts
  M1.attach(11);                              //attach motor
  sei();                                      //allow interrupts
}

void loop() 
{
 if(Serial.available()){
  valo = Serial.parseInt();
  M1.set_value(valo);
 }
}
