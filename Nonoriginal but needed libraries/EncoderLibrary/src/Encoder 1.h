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
  unsigned int currentTime;
  unsigned int prevTime;
  unsigned int deltaTime;
  unsigned int slots;
  float circumference;
  float diameter;
  float sec;   
  float rotational_speed;
  int TimerSelected;
  volatile int overflowcount;
  volatile float timer4_ticks;
  volatile float timer5_ticks;
  volatile bool timer_ready;
  
  void AddOverflowCount();

public:
  Encoder(unsigned int slots, unsigned int diameter, int TimerNumber, int unsigned pin);
  void updateTime(unsigned int time);
  double getSpeed();
  void zeroSpeed();
  void timerCapture();
  void OverflowHandle();
  bool check();
  
  
  };

#endif
