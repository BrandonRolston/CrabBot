/*
 * RangeFinder.cpp
 *
 * This code defines the class RangeFinder for a HC-SR04
 *
 * Main Library: EE335
 *
 */
 
 #include "RangeFinder.h"

RangeFinder::RangeFinder(float new_tolerance, int instance_id, int new_trigger_pin, int new_echo_pin, bool new_testing)
{
	distance=0.0;
	calc_time=0.0;
	
	trig_time=0;
	ping_time=0;
	tolerance=new_tolerance;
	object_dis=0.0;
	flag_change=0;
	overflow_count=0;
	ID = instance_id;
	
	state = 0;						
	//sound_conversions = new float[2]{340.29,1125.327};  // [0] = m/s and [1] = ft/s
	EchoPulseDuration=0.0;
	RoundTripDistance=0.0;
	
	testing = new_testing;  //Normal mode:false, Testing Mode: True
	
	trigger_pin = new_trigger_pin;
	echo_pin = new_echo_pin;
	
	pinMode(echo_pin,INPUT);
	pinMode(trigger_pin, OUTPUT);
	
	isClear=false;
	
	tick_offset=1;  //There is a slew rate inside the Arduino, this helps to compensate for the difference
	
    noInterrupts();
    TCCR2B = 0x00;        //Disbale Timer2 while we set it up
    TCCR2A = 0;// set entire TCCR2A register to 0
    TCNT2  = 0;                       // Set timer2 to 0
    OCR2A = 400; // Approx 1 ms
    TCCR2A |= (1 << WGM21); // CTC mode
    TCCR2B = 0x00; // Turn off clock (will be set to Prescale 128)
    TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupts
	TIMSK2 |= (1 << TOIE2);
    interrupts();
	
}

void RangeFinder::EnquireForDistance()  //This happens when a command button is pressed
{
	active = true;
	genTrigger();
	
}

bool RangeFinder::HasData()
{
	return flag_change;
}

bool RangeFinder::isPathClear()
{
	object_dis= getDistance();
	
	if(object_dis < tolerance)
		isClear=false;
	else
		isClear=true;
	
	return isClear;
}

void RangeFinder::genTrigger()
{
	//start timer2
	state=0;
	overflow_count=0;
	TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupts
	TCCR2B = 0x04; // Prescale 64
	
	//Turn on trigger signal
	digitalWrite(trigger_pin, HIGH);
}

float RangeFinder::getDistance()
{

	if(overflow_count)
	{
		ping_time=(ping_time+(255*overflow_count)-tick_offset);
	}
	else
	{
		ping_time-tick_offset;
	}
	
	//Serial.println(ping_time);
	calc_time = (1/16000000.0*8.0*(float)ping_time*1000.0);
	
	if(testing)
	{
		//Serial.println(sound_conversions[units_type]);
		//sound_conversions[units_type] I can eventually add this
		Serial.print("E_PW=");
		Serial.println(calc_time,4);		//These will be removed
	}
	
	EchoPulseDuration = calc_time*0.001;  //convert ms to s
	RoundTripDistance = (1125.327*EchoPulseDuration);
	distance = RoundTripDistance/2;
	
	if(testing)
	{
			Serial.print("D=");
			Serial.println(distance,6);
	}

	active = false;
	flag_change=0;
	return distance;
	
}

float RangeFinder::ReportDistance()
{
	return distance;
}

void RangeFinder::setTolerance(float new_distance)
{
	tolerance = new_distance;
}

float RangeFinder::getTolerance()
{
	return tolerance;
}

void RangeFinder::IncrementOVFcount()
{
	overflow_count=overflow_count+1;
}

void RangeFinder::UpdatePingTime(int new_time)
{
	ping_time = new_time;
	flag_change=1;
}

bool RangeFinder::IsActive()
{
	return active;
}

void RangeFinder::CompaVectorHandler()
{
   digitalWrite(trigger_pin,LOW);
   TIMSK2 &= ~(1 << OCIE2A); // desable timer compare interrupts
   TCNT2=0;
   TCCR2B=0x00;
}