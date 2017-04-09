#include "SpeedControl.h"
#include "arduino.h"
#include <Encoder.h>
#include <MotorClass.h>

SpeedControl::SpeedControl(Encoder *encoder, Motor *motor1, Motor *motor2, int newMax, float sampleTime, int ID, bool new_testing)
{
	m_setPoint = 0;
	m_currentSpeed = 0;
	m_prevSpeed = 0;
	m_sampleTime = sampleTime;
	m_error = 0;
	m_errorIntegral = 0;
	m_gainProportional = 4;
	m_gainIntegral = 0.03;
	m_gainDifferential = 0;
	m_encoder = encoder;
	m_motor1 = motor1;
	m_motor2 = motor2;
	m_max = newMax;
	id = ID;
	testing = new_testing;
	
	if(ID == 1)  // Right side
		duty_max = 245.0;
	else		// left side
		duty_max = 255.0;
}	
void SpeedControl::setDesiredSpeed(float setPoint) {
	m_setPoint = setPoint;
    m_motor1->setDrive(m_setPoint,FORWARD);
    m_motor2->setDrive(m_setPoint,FORWARD);
}

void SpeedControl::Update() {
	
	// Import speeds 
	m_prevSpeed = m_currentSpeed;
	m_currentSpeed = m_encoder->getSpeed();
	
	// Find the Error
	m_error = m_currentSpeed - (m_max*m_setPoint)/duty_max;
	
	// Integrate that Error
	m_errorIntegral = m_errorIntegral + m_error*m_sampleTime;

	// Limit that integrator wind-up
	if(m_errorIntegral > 10.0)
		m_errorIntegral = 10.0;
	else if(m_errorIntegral < -10.0)
		m_errorIntegral = -10.0;

	// Calculate the gain terms
	float pTerm = m_gainProportional*m_error;
	float iTerm = m_gainIntegral*m_errorIntegral;
	float dTerm = m_gainDifferential * (m_currentSpeed - m_prevSpeed)/m_sampleTime;

	
	// Calulate the newDrive value that the motors should be set to.
	float newDrive = -(iTerm + pTerm - dTerm);

	// scale Newdrive
	newDrive = duty_max*newDrive/m_max; //  Well, how about 3800?
	
	// Make limits for newDrive
	if(newDrive < 0.0)
		newDrive = 0;
	else if(newDrive > 255.0)
		newDrive = 255.0;
		
	if(newDrive == 0.0)
	{
		m_motor1->setDrive(0, RELEASE);
		m_motor2->setDrive(0, RELEASE);
	}
	else
	{
		m_motor1->setDrive ((int)newDrive, FORWARD);
		m_motor2->setDrive ((int)newDrive, FORWARD);
	}

	if(testing)
	{
		Serial.print(m_prevSpeed);
		Serial.print(",");
		Serial.print(m_currentSpeed);
		Serial.print(",");
		Serial.print(m_error);
		Serial.print(",");
		Serial.print(m_errorIntegral);	
		Serial.print(",");
		Serial.print(pTerm);
		Serial.print(",");
		Serial.print(iTerm);
		Serial.print(",");
		Serial.println(newDrive);	
	}	

}