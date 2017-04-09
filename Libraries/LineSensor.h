#ifndef LineSensor_h
#define LineSensor_h

class LineSensor{
private:
	unsigned int m_pin;
	
public:
	LineSensor(unsigned int pin);
	bool lineDetect();
};

#endif