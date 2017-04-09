#include "Gripper.h"
#include "arduino.h"
#define OPEN_ANGLE 150
#define CLOSE_ANGLE 60

Gripper::Gripper(unsigned int pin){
	m_gripper = new MegaServo;
	m_gripper->attach(pin,800,2200);
	m_gripper->write(OPEN_ANGLE);
	m_position = 1;
}

void Gripper::Open() {
if (m_position) //Then the thing is already open
	return;
else{
	m_gripper->write(OPEN_ANGLE);
	m_position = 1;
}
}

void Gripper::Close() {
if (!m_position) //Then the thing is already closed
	return;
else{
	m_gripper->write(CLOSE_ANGLE);
	m_position = 0;
}
}

bool Gripper::isOpen() {
	return(m_position);
}