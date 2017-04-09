#include "MotorClass.h"
//#include <Adafruit_MotorShield.h>

Motor::Motor(Adafruit_MotorShield *shieldPtr, byte motorNumber, boolean polarity) {
    m_motorNumber = motorNumber;
    m_polarity = polarity;
    m_motor = shieldPtr->getMotor(motorNumber);
    m_command = 0;
    m_dutyCycle = 0;
}

void Motor::setDrive(unsigned int dutyCycle, unsigned int command) {
    m_dutyCycle = dutyCycle;
    m_command = command;
    if(command == RELEASE)
        m_motor->run(RELEASE);
//    else if(command == BRAKE) It seems that "brake' is defined in the library, but never used.
//        m_motor->run(BRAKE);
    else if ((m_polarity && m_command == FORWARD) || (!m_polarity && m_command == BACKWARD))
        m_motor->run(FORWARD);
    else
        m_motor->run(BACKWARD);
        
    m_motor->setSpeed(m_dutyCycle);
}

unsigned int Motor::getDrive() {
    return(m_dutyCycle);
}

void Motor::stop() {
    m_motor->setSpeed(0);
}
