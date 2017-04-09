#include "LineSensor.h"
#include "arduino.h"

LineSensor::LineSensor(unsigned int pin){
	m_pin = pin;
	}
	
bool LineSensor::lineDetect() {
	return(digitalRead(m_pin));
}