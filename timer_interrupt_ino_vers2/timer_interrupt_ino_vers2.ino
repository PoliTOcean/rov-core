#include <stdlib.h>
#include <unistd.h>
#include "DumpedCurrentMotor_test2_ino.h"

int valo;
Motor M1(2);

void setup()
{

  Serial.begin(9600);
  cli();                                           //stop interrupts

  //first timer

  TCCR2A = 0;                                    // set entire TCCR1A register to 0
  TCCR2B = 0;                                    // same for TCCR1B
  TCNT2  = 0;                                    // initialize counter value to 0
  OCR2A  = 255;                                 // COMPARE REGISTER A = (16*10^6) / (1*1024) - 1 (must be <65536 -> 16 bit)-------> [16*10^6/(prescale*desired frequncy)] -1
  TCCR2B |= (1 << WGM22);                        // turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);           // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK2 |= (0 << OCIE2A);                       // enable timer compare interrupt
  
  sei();                                           //allow interrupts
}

void loop() 
{
 while (!Serial.available());
 valo = Serial.parseInt();
   M1.set_value(valo);
}
