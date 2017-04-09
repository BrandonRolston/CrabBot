// Gripper Class
// Reproduced from a classroom example by:
// A. Douglas, 2016

#ifndef Gripper_h
#define Gripper_h

#include <MegaServo.h>

class Gripper{
private:
	bool m_position; //0 = closed, 1 = open
	MegaServo *m_gripper;
	
public:
	Gripper(unsigned int pin);
	void Open();
	void Close();
	bool isOpen();
};

#endif