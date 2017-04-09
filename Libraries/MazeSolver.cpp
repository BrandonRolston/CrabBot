/* MazeSolver.cpp
 *
 * This is the cpp file for the mazesolving algorithm libray.
 *
 * The purpose of this code is to control the mazesolving algorithm for the Arduino Robot
 *
 * Main library: EE335
 *
 */
 
#include <MazeSolver.h>
 
 
MazeSolver::MazeSolver(RangeFinder* fRangeFinder, RangeFinder* lRangeFinder,
						Motor * new_rfMotor, Motor * new_rrMotor,Motor * new_lfMotor, Motor * new_lrMotor,
						float new_ftolerance, float new_ltolerance, float new_maxdistance)
{
	//RangeFinders
	frange = fRangeFinder;
	lrange = lRangeFinder;
		
	//Motors
	lfMotor = new_lfMotor;
	lrMotor = new_lfMotor;
	rfMotor = new_rfMotor;
	rrMotor = new_rrMotor; 
	 
	solved = false;								// True when we are done
	oldTolerancef = frange->getTolerance();    	// Saved tolerence of the first range finder (to be restored when ReportSolved is finished)
	oldTolerancel = frange->getTolerance();    	// Saved tolerence of the left range finder (to be restored when ReportSolved is finished)
	ftolerance = new_ftolerance;        		// Value to be checked
	ltolerance = new_ltolerance;
	
	distanceForward = 0.0;
	distanceLeft = 0.0;
	lookingforward = false;
	lookingleft = false;
	doneLookingForward=false;
	doneLookingLeft = false;
	goingforward = false;
	goingleft = false;
	goingright = false;
	maxdistance = new_maxdistance;
		
 }

void MazeSolver::StartSolving()
{
	solved=false;
} 
 
bool MazeSolver::SolvingStep()		// Main Control
{	
	LookForward();
	if(doneLookingForward)
	{
		distanceForward = frange->getDistance();
		
		if(frange->isPathClear() && distanceForward < maxdistance)		
		{
			GoForward();
		}
		else
		{
			Stop();
			LookLeft();
			if(doneLookingLeft)
			{
				distanceLeft = lrange->getDistance();
				
				if(distanceForward > maxdistance && distanceLeft > maxdistance)
				{
					ReportSolved();
				}
				else
				{
					if(lrange->isPathClear())
					{
						TurnLeft();
					}
					else
					{
						TurnRight();
					}
				}
			}
		}
	}
	return solved;
}

void MazeSolver::ReportSolved()	// Report that we are out of the maze (when both Forward facing and left facing see a clear path)
{
	frange->setTolerance(oldTolerancef);
	lrange->setTolerance(oldTolerancel);
	
	solved = true;
}
 
// Private Functions
void MazeSolver::LookForward()		// Calls the floward facing rangerfinder to tell if the path is clear
{
	if(lookingforward==false)
	{
		lookingforward = true;
		
		frange -> EnquireForDistance();
	}
	
	if(frange->HasData())
	{
		lookingforward=false;
		doneLookingForward=true;
	}
	
}

void MazeSolver::LookLeft()		// Calls the left facing rangefinder to tell if the path is clear
{
	if(lookingleft==false)
	{
		lookingleft = true;
		
		lrange -> EnquireForDistance();
	}
	
	if(lrange->HasData())
	{
		lookingleft=false;
		doneLookingLeft=true;
	}
	
}
void MazeSolver::TurnRight()		// Turn right till the path is clear 
{
	
	if(goingright == false)
	{
		goingright = true;
		lfMotor->setDrive(150,BACKWARD); 
		lrMotor->setDrive(150,BACKWARD); 
		rfMotor->setDrive(150,FORWARD); 
		rrMotor->setDrive(150,FORWARD); 
	}
	else
	{
		//do nothing different
	}

	
}
void MazeSolver::TurnLeft()		// Turn left till the path is clear
{
	if(goingleft == false)
	{
		lfMotor->setDrive(150,FORWARD); 
		lrMotor->setDrive(150,FORWARD); 
		rfMotor->setDrive(150,BACKWARD); 
		rrMotor->setDrive(150,BACKWARD);
	}
	else
	{
		//do nothing different
	}

}

void MazeSolver::GoForward()		// Drive Forward
{
	if(goingforward == false)
	{
		lfMotor->setDrive(150,FORWARD); 
		lrMotor->setDrive(150,FORWARD); 
		rfMotor->setDrive(150,FORWARD); 
		rrMotor->setDrive(150,FORWARD);
		goingforward = true;
	}
	else
	{
		//Do nothing different
	}
		
 
} 

void MazeSolver::Stop()			// Stop Moving
{
	lfMotor->setDrive(0,RELEASE); 
	lrMotor->setDrive(0,RELEASE); 
	rfMotor->setDrive(0,RELEASE); 
	rrMotor->setDrive(0,RELEASE);
	goingforward = false;
	goingleft = false;
	goingright = false;
}
