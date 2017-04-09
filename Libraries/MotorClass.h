#include "arduino.h"
#include <Adafruit_MotorShield.h>

#ifndef Motor_h
#define Motor_h

class Motor{
private:
    byte m_motorNumber;
    boolean m_polarity;
    Adafruit_DCMotor *m_motor;
    unsigned int m_command;
    unsigned int m_dutyCycle;
    
public:
    Motor(Adafruit_MotorShield *shieldPtr, byte motorNumber, boolean polarity);
    void setDrive(unsigned int drive, unsigned int command);
    unsigned int getDrive();
    void stop();
};

#endif