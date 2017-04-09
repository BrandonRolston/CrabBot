#ifndef LineFollower_h
#define LineFollower_h
#include <SpeedControl.h>
#include <LineSensor.h>
#include <RangeFinder.h>


class LineFollower{
private:
	LineSensor *m_leftSensor;
	LineSensor *m_middleSensor;
	LineSensor *m_rightSensor;
	Motor *m_rfMotor;
	Motor *m_rrMotor;
	Motor *m_lfMotor;
	Motor *m_lrMotor;
	RangeFinder *m_rangeFinder;
	int m_count;
	bool m_offCourse;
	bool m_rightLine;
	bool m_leftLine;
	bool m_middleLine;
	bool volatile m_isPathClear;
	bool volatile m_changed;
	bool prev_left;
	bool prev_middle;
	bool prev_right;  
	int m_leftSensorPin;
	int m_middleSensorPin;
	int m_rightSensorPin;
	int left;
	int right;
	int middle;
	bool volatile looking;
	float volatile distance;
	
	
public:
	LineFollower(unsigned int leftSensorPin, unsigned int middleSensorPin, unsigned int rightSensorPin, 
				 Motor *rfMotor, Motor *rrMotor, Motor *lfMotor, Motor *lrMotor, RangeFinder *forwardSensor);
	bool follow(bool changed);
	bool change_test();
	void isPathClear();
};

#endif