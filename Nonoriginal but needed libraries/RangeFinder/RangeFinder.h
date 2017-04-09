// RangeFinder.h

#ifndef Arduino_h
	#include <Arduino.h>
#endif

#ifndef avr/interrupt_h
	#include <avr/interrupt.h>
#endif

#ifndef RangeFinder_h
#define RangeFinder_h

class RangeFinder {
	private:
		
		float distance; 			//Final Distance Measurment
		float calc_time;
		float* sound_conversions;   // Sounds speed using different units
		unsigned int trig_time; 	// The time of the trigger signal (not needed)
		unsigned int ping_time;		// Ping Time is used to calculate the length of the Echo signal
		
		//int tcontrol_ON;	//Preset control ON (Set to a position in the presets array)
		//int tcontrol_OFF;	//Preset control OFF (set to 0)
		//int* presets;
		
		float tolerance;	// What is considered an open path? Set this value to that.
		float object_dis;	// This is the calculated distance to be used for comparison
		float EchoPulseDuration;
		float RoundTripDistance;
		
		bool isClear;		// This is the bool that reports if a path is clear or not
		byte tick_offset;
		int units_type;
		
		
		volatile int overflow_count;	//Number of overflows given in the echo signal
		volatile int flag_change;		// Has Data flag (if true, then ready to calculate)
		volatile int state;				// are we going into the first time or second time 
										// with this interrupt?
		bool testing;
		
		int trigger_pin;	// Trigger pin that is connected to the triggerpin of the RangeFinder
		int echo_pin;		// Echo pin that is connected to the echo pin of the RangeFinder			
		//int testpin;		// a test pin that can be used to test data
		
	public:
	
		RangeFinder(float new_tolerance,int units_type, int trigger_pin, int echo_pint, bool testing);
		void EnquireForDistance();
		bool HasData();
		float getDistance();
		void genTrigger();
		float ReportDistance();
		//int echoReciver();
		bool isPathClear();
		void setTolerance(float new_distance);
		void IncrementOVFcount();
		void UpdatePingTime(int new_time);
};

#endif