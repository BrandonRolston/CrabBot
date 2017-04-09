/*
 * This is the cpp file for the Encoder class. It defines how our two encoders will function
 * 
 * Resourse Used:
 * Timer4 and Timer5, Pins 48, 49
 *
*/

#include "Encoder.h"

Encoder::Encoder(unsigned int new_slots, unsigned int new_diameter, int new_TimerSelected, unsigned int pin) {
	
	speed = 0.0;
	prevSpeed = 0.0;
    slots = new_slots;
  	pinMode(pin,INPUT);
	diameter = new_diameter;
	timer4_ticks = 0.0;
	timer5_ticks = 0.0;
	sec = 0.0;
	circumference= (3.1415926*(float)diameter);
	rotational_speed = (360/(float)slots);
	TimerSelected = new_TimerSelected;
	overflowCount = 0;
	timer_ready = false;
	
	noInterrupts();
	
	if (TimerSelected == 4)   //LEFT ENCODER!
	{
		//Serial.println("Madeit4");
		//Setup Timer 4  
		TCCR4A = 0;                   
		TCNT4  = 0;                        // Set timer4 to 0
		//ICNC4=noise canceller on, ICES4=rising edge, clk_(i/o)/64 (prescaler)
		TCCR4B |= (1<<ICNC4)  | (1<<ICES4) | (1<<CS41) |(1<<CS40);
		TIFR4  |= (1<<ICF4)  | (1<<TOV4);     // clear pending
		TIMSK4 |= (1<<ICIE4) | (1<<TOIE4);   // and enable
		PORTL  |= (1<<PORTL1)| (1<<PORTL0);
	}
	else if (TimerSelected == 5)  //RIGHT ENCODER!
	{
		//Serial.println("Madeit5");
		//Setup Timer 5  
		TCCR5A = 0;
		TCNT5  = 0;                       // Set timer5 to 0
		
		//ICNC4=noise canceller on, ICES4=rising edge, clk_(i/o)/64 (prescaler)
		TCCR5B |= (1<<ICNC4)| (1<<ICES5) | (1<<CS51); //| (1<<CS50);
		TIFR5  |= (1<<ICF5)| (1<<TOV5);    // clear pending
		TIMSK5 |= (1<<ICIE5)| (1<<TOIE5);  // and enable
		//(DI, 0, T5 INPUT, ICP5 INPUT, ICP4 INPUT)
		PORTL  |= (1<<PORTL1)| (1<<PORTL0); 
	}
	
	interrupts();
}

void Encoder::updateTime(unsigned int ticks)
 {
	//Serial.println(ticks);
	
	if(ticks < 18)
	{
		prevSpeed2 = prevSpeed;
		speed = prevSpeed;
	}
	else
	{//*/
		//Serial.println(ticks);
		sec = (float)ticks * 8/16000000;
		//Serial.print("Sec=");
		//Serial.println(sec,4);
		speed = ((circumference/360)*rotational_speed/sec);  //mm/sec
		
		speed = (prevSpeed2 + prevSpeed + speed)/3;
	}
	prevSpeed = speed; 
	
	//Serial.println(speed, 6);
}

float Encoder::getSpeed() {
	this->check();
	return(speed);
}

bool Encoder::check()
{
	bool responce=false;
	if (timer_ready)  //Ready to update prevSpeed;
	{
		if (TimerSelected==4)
		{
			updateTime(timer4_ticks);
			timer_ready=false;
			responce=true;
		}
		else
		{
			updateTime(timer5_ticks);
			timer_ready=false;
			responce=true;
		}
	}
	else
	{
		prevSpeed2 = prevSpeed;
		speed=prevSpeed;
	}
	
	return responce;
}

void Encoder::timerCapture()
{
	if (TimerSelected==4)
	{
		timer4_ticks = ICR4;
		TCNT4 = 0;
		timer_ready=true;
	}		
	else if(TimerSelected==5)
	{
		timer5_ticks = ICR5;
		TCNT5 = 0;		
		timer_ready=true;
	}
}

void Encoder::OverflowHandle()
{
	if(IncrementOVFCount() >= 1)
	{
		//Serial.println("HereAtTheWall");
		speed = 0.0;
		prevSpeed=0.0;
		overflowCount=0;
		timer_ready=false;
	}

}

int Encoder::IncrementOVFCount()
{
	overflowCount = overflowCount+1;
	return overflowCount;
}