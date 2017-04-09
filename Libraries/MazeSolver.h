/* MazeSolver.h
 *
 * This is the header file for the mazesolving algorithm libray.
 *
 * The purpose of this code is to control the mazesolving algorithm for the Arduino Robot
 *
 * Main library: EE335
 *
 */
 
 /* Solving Algorithm
	1. Check mode (we need to be in the MazeSolver mode before progressing)
	2. Construct the MazeSolver Class(Ptrs to the motors, pass in the ptrs to the 2 rangefinders, pins for testing, testing mode bool value?)
			
	3.  StartSolving
		{	
			StartTimer  //Timer 3
			LookForward
			MoveForward
			{
				while(LookForward)
				{
					drive
				}
				//Something is in front of us
				if(LookLeft)
				{
					
				}
				
			}	
		}
		
		ISR(timer3_comp_vec)	// 10ms
		{
			lookForward
			LookLeft
		}
//*/
 
 #ifndef Arduino_h
	#include <Arduino.h>
#endif

#ifndef avr/interrupt_h
	#include <avr/interrupt.h>
#endif

#ifndef RangeFinder_h
	#include <RangeFinder.h>
#endif

#ifndef MotorClass_h
	#include <MotorClass.h>
#endif

#ifndef Adafruit_MotorShield_h
	#include <Adafruit_MotorShield.h>
#endif
	
#ifndef MazeSolver_h

#define MazeSolver_h

class MazeSolver{
	
	private:
	
		bool solved;			// True when we are done
		float oldTolerancef;    // Saved tolerence of the first range finder (to be restored when ReportSolved is finished)
		float oldTolerancel;    // Saved tolerence of the left range finder (to be restored when ReportSolved is finished)
		float ftolerance;        // Value to be checked
		float ltolerance;
		bool lookingforward;
		bool lookingleft;
		bool doneLookingForward;
		bool doneLookingLeft;
		bool goingforward;
		bool goingright;
		bool goingleft;
		
		float distanceForward;
		float distanceLeft;
		float maxdistance;
		
		//RangeFinders
		RangeFinder* frange;
		RangeFinder* lrange;
		
		//Motors
		Motor* lfMotor;
		Motor* lrMotor;
		Motor* rfMotor;
		Motor* rrMotor;
		
		// Private Functions
		void LookForward();		// Calls the floward facing rangerfinder to tell if the path is clear
		void LookLeft();		// Calls the left facing rangefinder to tell if the path is clear
		void TurnRight();		// Turn right till the path is clear 
		void TurnLeft();		// Turn left till the path is clear
		void GoForward();		// Drive Forward
		void Stop();
		void ReportSolved();	// Report that we are out of the maze (when both forward facing and left facing see a clear path)
		
	public:
		MazeSolver(RangeFinder* fRangeFinder, RangeFinder* lRangeFinder, Motor * rfMotor, Motor * rrMotor, Motor * lfMotor, Motor * lrMotor, float ftolerance, float rtolerance, float new_maxdistance);			// Constructor
		bool SolvingStep();		// Main Control
		void StartSolving(); 	// Tells the solver that it is in position to start
};
#endif