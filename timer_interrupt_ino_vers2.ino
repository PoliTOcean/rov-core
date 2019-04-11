#include "DumpedCurrentMotor_test2_ino.h"
#include <stdlib.h>
#include <unistd.h>

int valo;
Motor M1,M2,M3,M4,M5,M6,M7;

void setup()
{

  Serial.begin(9600);
  cli();                                           //stop interrupts

  //first timer

  TCCR1A = 0;                                    // set entire TCCR1A register to 0
  TCCR1B = 0;                                    // same for TCCR1B
  TCNT1  = 0;                                    // initialize counter value to 0
  OCR1A  = 15624;                                 // COMPARE REGISTER A = (16*10^6) / (1*1024) - 1 (must be <65536 -> 16 bit)-------> [16*10^6/(prescale*desired frequncy)] -1
  TCCR1B |= (1 << WGM12);                        // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);           // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (0 << OCIE1A);                       // enable timer compare interrupt
  
  sei();                                           //allow interrupts

}

void loop() 
{
 while (!Serial.available());
 valo = Serial.parseInt();
   M1.set_value(valo);
   M2.set_value(valo);
   M3.set_value(valo);
   M4.set_value(valo);
   M5.set_value(valo);
   M6.set_value(valo);
   M7.set_value(valo);
}
