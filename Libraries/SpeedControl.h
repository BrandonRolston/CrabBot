
#ifndef arduino_h
	#include "arduino.h"
#endif

#ifndef Encoder_h
	#include <Encoder.h>
#endif

#ifndef MotorClass_h
	#include <MotorClass.h>
#endif

#ifndef SpeedControl_h
#define SpeedControl_h

class SpeedControl{
private:
	volatile float m_setPoint;
	volatile float m_currentSpeed;
	float m_prevSpeed;
	float m_sampleTime;
	float m_error;
	float m_errorIntegral;
	float m_gainProportional;
	float m_gainIntegral;
	float m_gainDifferential;
	bool m_enable;
	Encoder *m_encoder;
	Motor *m_motor1;
	Motor *m_motor2;
	int m_max;
	int id;
	float duty_max;
	bool testing;
	
public:
	SpeedControl(Encoder *encoder, Motor *motor1, Motor *motor2, int newMax, float sampleTime, int ID, bool testing);
	void setDesiredSpeed(float setPoint);
	void Update();
};

#endif