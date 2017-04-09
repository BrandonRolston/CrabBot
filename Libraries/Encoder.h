#ifndef Arduino_h
	#include <Arduino.h>
#endif

#ifndef Wire_h
	#include <Wire.h>
#endif

#ifndef avr/interrupt_h
	#include <avr/interrupt.h>
#endif

#ifndef Encoder_h
#define Encoder_h

class Encoder {

private:
  volatile float speed;
  volatile float prevSpeed;
  volatile float prevSpeed2;
  unsigned int slots;
  float circumference;
  float diameter;
  float sec;   
  float rotational_speed;
  int TimerSelected;
  volatile int overflowCount;
  volatile float timer4_ticks;
  volatile float timer5_ticks;
  volatile bool timer_ready;

  // Private Functions
  bool check();
  void updateTime(unsigned int time);
  int IncrementOVFCount();
  
public:
  //Constructor	(number of slots in encoder, diameter of wheel, Timer Number (4 or 5 only!), capture pin)
  Encoder(unsigned int slots, unsigned int diameter, int TimerNumber, int unsigned pin);
  // getSpeed calls check() and if 
  float getSpeed();
  void timerCapture();
  void OverflowHandle();

};

#endif
