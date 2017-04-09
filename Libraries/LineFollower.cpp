#include "LineFollower.h"
#include "arduino.h"

LineFollower::LineFollower(unsigned int leftSensorPin, unsigned int middleSensorPin, unsigned int rightSensorPin,
				           Motor *rfMotor, Motor *rrMotor, Motor *lfMotor, Motor *lrMotor, RangeFinder *forwardSensor){
	
	m_leftSensorPin=leftSensorPin;
	m_middleSensorPin=middleSensorPin;
	m_rightSensorPin = rightSensorPin;
	m_leftSensor =   new LineSensor(leftSensorPin);
	m_middleSensor = new LineSensor(middleSensorPin);
	m_rightSensor =  new LineSensor(rightSensorPin);
	m_rfMotor = rfMotor;
	m_rrMotor = rrMotor;
	m_lfMotor = lfMotor;
	m_lrMotor = lrMotor;
	m_rangeFinder = forwardSensor;
	m_offCourse = 1;
	m_rightLine = 0;
	m_leftLine = 0;
	m_middleLine = 0;
	m_isPathClear = 1;
	m_changed = 0;
	m_count = 1;
	distance=0.0;
	
	left = false;
	middle = false;
	right = false;
	
	prev_left = false;
	prev_middle = false;
	prev_right = false;
	looking = false;
	
}

bool LineFollower::follow(bool changed) {
  m_changed = changed;
  
  
  if (m_isPathClear == 0) {            // The way ahead is NOT clear!
    m_rfMotor->setDrive(0,RELEASE); 
    m_rrMotor->setDrive(0,RELEASE);
    m_lfMotor->setDrive(0,RELEASE);   // Hit the brakes
    m_lrMotor->setDrive(0,RELEASE);
    m_offCourse = 0;                // This should resume behavior as if it just started once the way becomes clear
	//Serial.println("P");
  }
  else if (m_changed == 1) 
  {                       // If the sensors detect a change
    m_rightLine = m_rightSensor->lineDetect();
    m_middleLine = m_middleSensor->lineDetect();  // Read the values off the sensors
    m_leftLine = m_leftSensor->lineDetect();
	
	/*
	Serial.print(m_leftLine);
	Serial.print(",");
	Serial.print(m_middleLine);
	Serial.print(",");
	Serial.println(m_rightLine);
	*/
	
    if (m_offCourse == 0)
	{                        // If the line had previously been followed
      
	  if (m_leftLine == 1 && m_middleLine == 1 && m_rightLine == 0) { // Sharp Left Turn
        m_rfMotor->setDrive(255,FORWARD); 
        m_rrMotor->setDrive(255,FORWARD);
        m_lfMotor->setDrive(50,BACKWARD);
        m_lrMotor->setDrive(50,BACKWARD);
        m_offCourse = 1;
      }
	  
      else if (m_leftLine == 0  && m_middleLine == 1 &&  m_rightLine == 1) { //Sharp Right Turn
        //Serial.println("HARD Right");
		m_rfMotor->setDrive(50,BACKWARD); 
        m_rrMotor->setDrive(50,BACKWARD);
        m_lfMotor->setDrive(255,FORWARD);
        m_lrMotor->setDrive(255,FORWARD);
        m_offCourse = 1;
      }	  
	 
	  else if(m_leftLine == 0 && m_middleLine == 0 && m_rightLine == 1) { //Drifting off to the left, turn right gradually
        //Serial.println("Drifting Right");
		m_rfMotor->setDrive(50,BACKWARD);
        m_rrMotor->setDrive(0,BACKWARD);
        m_lfMotor->setDrive(225,FORWARD);
        m_lrMotor->setDrive(200,FORWARD);
        m_offCourse = 1;
      }
	 
      else if (m_leftLine == 1 && m_middleLine == 0 && m_rightLine == 0) { //Drifting off to the right, turn left gradually
        m_rfMotor->setDrive(225,FORWARD);
        m_rrMotor->setDrive(200,FORWARD);
        m_lfMotor->setDrive(0,BACKWARD);
        m_lrMotor->setDrive(50,BACKWARD);
        m_offCourse = 1;
      }
	  
      //else { // All three sensors see nothing / see the line, the line has been lost.
      //  m_rfMotor->setDrive(0,RELEASE);
      //  m_rrMotor->setDrive(0,RELEASE);
      //  m_lfMotor->setDrive(0,RELEASE);
      //  m_lrMotor->setDrive(0,RELEASE);
      //  m_offCourse = 1;
      //}
    }
    else if (m_middleLine == 1){  //We're right on the money
      m_rfMotor->setDrive(225,FORWARD);
      m_rrMotor->setDrive(225,FORWARD);   
      m_lfMotor->setDrive(225,FORWARD);
      m_lrMotor->setDrive(225,FORWARD);
      m_offCourse = 0;
    }
	
    m_changed = 0;
	return(m_changed);
	}
}

bool LineFollower::change_test()
{
	prev_left = left;
	prev_middle = middle;
	prev_right = right;
	
	left = digitalRead(m_leftSensorPin);
	middle = digitalRead(m_middleSensorPin);
	right = digitalRead(m_rightSensorPin);
	
	/*
	Serial.print("l=");
	Serial.print(left);
	Serial.print(",r=");
	Serial.print(right);
	Serial.print(",m=");
	Serial.println(middle);
	Serial.print("pl=");
	Serial.print(prev_left);
	Serial.print(",pm=");
	Serial.print(prev_middle);
	Serial.print(",pr=");
	Serial.println(prev_right);
	*/
	
	if(prev_left != left || prev_middle != middle || prev_right != right)
	{
		return true;
	}
	return false;
}

void LineFollower::isPathClear()
{
	// Range Finder Logic
	if(!looking)    // Counter to ensure that the range finder isn't pinged too often
    {
     m_rangeFinder->EnquireForDistance();  //Ask to get distance
	 looking = true;
	 m_isPathClear = false;
	 
    }
     
     if(m_rangeFinder->HasData()) //Data is ready to be used.
     {
        //m_isPathClear
		distance = m_rangeFinder->getDistance();   //Ask if the path is clear (using tolerance)
		looking = false;
		//Set count to zero to get ready to generate the pulse again.
     }
	 Serial.print("Looking=");
	 Serial.println(looking);
	 Serial.print("distance=");
	 Serial.println(distance,4);
}