/* RangeFinder.h
 *
 * This is the header file for the Untrasonic Range Finder libray.
 *
 * The purpose of this code is to control the Untrasonic Range Finder chip on a robot inorder to navigate through obsticals.
 *
 * Main library: EE335
 *
 */
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
		bool active;
		
		float tolerance;	// What is considered an open path? Set this value to that.
		float object_dis;	// This is the calculated distance to be used for comparison
		float EchoPulseDuration;
		float RoundTripDistance;
		
		bool isClear;		// This is the bool that reports if a path is clear or not
		byte tick_offset;
		int ID;
		
		volatile int overflow_count;	//Number of overflows given in the echo signal
		volatile int flag_change;		// Has Data flag (if true, then ready to calculate)
		volatile int state;				// are we going into the first time or second time 
										// with this interrupt?
		bool testing;
		
		int trigger_pin;	// Trigger pin that is connected to the triggerpin of the RangeFinder
		int echo_pin;		// Echo pin that is connected to the echo pin of the RangeFinder			
		//int testpin;		// a test pin that can be used to test data
		
		//Private Functions
		void genTrigger();
		
	public:
	
		RangeFinder(float new_tolerance,int ID, int trigger_pin, int echo_pint, bool testing);
		void EnquireForDistance();		//First function called
		bool HasData();					// Function that should be polled by the main loop. 
										// When this returns true, then call the other funcitons 
		
		//  (next line) Returns the value that has been calculated (make sure that HasData is true!)
		float ReportDistance();
		float getDistance();		
		// (next line) Returns a true if the distance measure is less then the tolerance value
		bool isPathClear();
		// (next line) Sets the tolerance distance that isPathClear checks
		void setTolerance(float new_distance);
		// (next line) Adds one to the number of overflow counts for Timer2
		void IncrementOVFcount();
		// (next line) Gets the time from Timer2 and updates the ping_time variable
		void UpdatePingTime(int new_time);
		void CompaVectorHandler();
		bool IsActive();
		float getTolerance();
};

#endif