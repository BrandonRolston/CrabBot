/*
 * RoboDemo FullFeature
 * EE335
 * Authors: Brandon Rolston,William Pearson
 * 
 * Description: This schetch controls a robot car using an Arduino Mega 2560, in order 
 * to perform four obsticals for Oregon Instutite of Technology's robot compitition.
 * 
 * Website for resources: http://sites.ieee.org/sb-oit/2017/01/08/ee335-advanced-microcontrollers-class-component-kit/
 *
 */

/*
 * Resources taken up:
 * 
 * Normal Functions
 * Pins: 23, 27, 30, 31, 40, 41, 48, 49, 52, 53
 * Timers: Timer0, Timer1, Timer2, Timer3, Timer4, Timer5
 * Serial Ports: Serial0 (pc communication), Serial1 (BlueTooth)
 * 
 * By Groups:         : IN.COMP   : Pin(s)
 * Gripper            : Timer1    : 53
 * Front Range Finder : Timer2*   : 41 (Trigger),40 (Echo)
 * Left Range Finder  : Timer2*   : 30 (Trigger),31 (Echo)
 * BlueTooth Board    : Serial1   : 18 (TX1), 19 (RX1)
 * Left Encoder       : Timer4    : 48 (ICNC4)
 * Right Encoder      : Timer5    : 49 (ICNC5)
 * Left LineSensor    : NC        : 23 (OUT) 
 * Middle LineSensor  : NC        : 27 (OUT)
 * Right LineSensor   : NC        : 52 (OUT)
 * PC Communication   : Serial0   : USB peripheral
 * 
 * *Note: The RangeFinders are always called seperatly and can
 *        share timer2 without overriding each other.
 * 
 * Testing Functions (Pins) : Range(32- 38)
 * 
 */


#include <Encoder.h>
#include <MotorClass.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Gripper.h>
#include <MegaServo.h>
#include <LineSensor.h>
#include <LineFollower.h>
#include <MazeSolver.h>
#include <SpeedControl.h>

// Motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Motor *lfMotor;
Motor *rfMotor;
Motor *lrMotor;
Motor *rrMotor;

//Encoder
#define LEFT_ENCODER_INPUT_PIN 48
#define RIGHT_ENCODER_INPUT_PIN 49
Encoder *lEncoder;
Encoder *rEncoder;
float encoder_slots=20.0;
float wheel_diameter=65.0;

// Speed Controller
SpeedControl* rSpeedControl;
SpeedControl* lSpeedControl;
float sampleTime = 0.1; // 100 ms
bool update_control_flag = false;
bool first_time=true;

//Bluetooth
char message[256];
int index = 0;
int robotMode = 0; // Dictates whether in manual (0) or auto (1-3) modes.

//Servo
#define SERVO_PIN 53
Gripper *myGripper;

// Line Follower
#define SENS_PIN_L 23
#define SENS_PIN_M 27
#define SENS_PIN_R 52
LineFollower *lineFollower;
volatile bool changed = 0;

// =============RangeFinder=======================
//Inputs
float toleranceForward= 0.5; //In ft
float toleranceLeft= 0.5; //In ft
const byte ftrigger_pin = 41;
const byte fecho_pin = 19;
const byte ltrigger_pin = 30;
const byte lecho_pin = 18;
int id;

//Loop controls
int count=1;
volatile byte scope_state=LOW;
bool mode_toggle = false;
bool testing = false;

RangeFinder* forwardSensor = new RangeFinder(toleranceForward, ftrigger_pin, fecho_pin, testing);
RangeFinder* leftSensor = new RangeFinder(toleranceLeft, ltrigger_pin, lecho_pin, testing);

// Maze Solver
float toleranceForwardNew = 1;
float toleranceLeftNew = 1;
float maxDistanceNew = 5;
bool mazeSolved = 0;

MazeSolver* mazeSolver = new MazeSolver (forwardSensor, leftSensor, rfMotor, 
                         rrMotor,lfMotor,lrMotor, toleranceForwardNew, toleranceLeftNew, 
                         maxDistanceNew);

/* ===================================
                    SETUP
    =================================== */
void setup() {
  // Motor Startup
  AFMS.begin();
  rfMotor = new Motor (&AFMS, 3, 1);
  rrMotor = new Motor (&AFMS, 2, 0);
  lfMotor = new Motor (&AFMS, 4, 1);
  lrMotor = new Motor (&AFMS, 1, 1);

  // Gripper Startup
  myGripper = new Gripper(SERVO_PIN);

  // Range Finder Startup
  pinMode(fecho_pin,INPUT);
  attachInterrupt(digitalPinToInterrupt(fecho_pin), blink, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lecho_pin), blink, CHANGE);
  scope_state=0;

  // Encoder Startup
  lEncoder = new Encoder(encoder_slots, wheel_diameter, 4, (int)LEFT_ENCODER_INPUT_PIN);
  rEncoder = new Encoder(encoder_slots, wheel_diameter, 5, (int)RIGHT_ENCODER_INPUT_PIN);

  // Speed Controller Startup
  rSpeedControl = new SpeedControl(lEncoder, rfMotor, rrMotor, 3800, sampleTime,1,false);
  lSpeedControl = new SpeedControl(rEncoder, lfMotor, lrMotor, 3800, sampleTime,2,false);
  
  // Bluetooth Setup
  Serial2.begin(115200);

  // Line Follower Startup
  lineFollower = new LineFollower(SENS_PIN_L, SENS_PIN_M, SENS_PIN_R,
                  rfMotor, rrMotor, lfMotor,lrMotor, forwardSensor);

   // Maze Solver Startup

  // Set up timer 3
  TCCR3A = 0; 
  TCCR3B = 0x0D;    // Prescaler=1024 (101)
  OCR3A = 0x061C; // Set the Compare A register
  TIMSK3 = 0x02;    // Enable the Output Compare A Match interrupt
   

  // Uncomment if you require a serial reading for testing purposes
  Serial.begin(115200);
}

/* ===================================
                 MAIN LOOP
    =================================== */

void loop() 
{ 
/* ===================================
        Manual Robot Mode Control
    =================================== */
  if(Serial2.available())  // If the bluetooth sent any characters
    {
      message[index] = (char)Serial2.read();  // Add the character to the message buffer
      if (message[index] == 0x3F) {             // Check for the "?" character
        int button_comp = message[index - 1];
        switch(button_comp)
        {
          case 49: // (1) Slow Forward
                if (robotMode == 0)
                  {
                  lrMotor->setDrive(100,FORWARD);
                  rrMotor->setDrive(100,FORWARD);
                  lfMotor->setDrive(100,FORWARD);  
                  rfMotor->setDrive(100,FORWARD);
                  }
                  break;                 
          case 50: // (2)  Full Forward
                if (robotMode == 0)
                  {
                    lrMotor->setDrive(255,FORWARD);
                    rrMotor->setDrive(255,FORWARD);
                    lfMotor->setDrive(255,FORWARD);
                    rfMotor->setDrive(255,FORWARD);
                  }
                  break;
          case 51: // (3)  Medium Forward
                if (robotMode == 0)
                  {
                    lrMotor->setDrive(170,FORWARD);
                    rrMotor->setDrive(170,FORWARD);
                    lfMotor->setDrive(170,FORWARD);
                    rfMotor->setDrive(170,FORWARD);
                  }
                  break;
          case 52: // (4)  Full Left
                if (robotMode == 0)
                  {
                    rfMotor->setDrive(255,FORWARD);
                    rrMotor->setDrive(255,FORWARD);
                    lfMotor->setDrive(255,BACKWARD);
                    lrMotor->setDrive(255,BACKWARD);
                  }
                  break;
          case 53:  // (5) Stop, reset to manual mode.
                  lrMotor->stop();
                  rrMotor->stop();
                  lfMotor->stop();
                  rfMotor->stop();
                  robotMode = 0; 
                  break;
          case 54: // (6) Full Right
                if (robotMode == 0)
                  {
                  rfMotor->setDrive(255,BACKWARD);
                  rrMotor->setDrive(255,BACKWARD);
                  lfMotor->setDrive(255,FORWARD);
                  lrMotor->setDrive(255,FORWARD);
                  }
                  break;
          case 55: // (7) Open Gripper
                if (robotMode == 0)
                  {
                  myGripper->Open();
                  }
                  break;
          case 56: // (8) Full Reverse
                if (robotMode == 0)
                  {
                  lrMotor->setDrive(255,BACKWARD);
                  lfMotor->setDrive(255,BACKWARD);
                  rrMotor->setDrive(255,BACKWARD);
                  rfMotor->setDrive(255,BACKWARD);
                  }
                  break;
          case 57: // (9) Close Gripper
                if (robotMode == 0)
                  {
                  myGripper->Close();
                  }
                  break;
          case 65: // (A) Begin Line Follow
                  robotMode = 1;
                  rfMotor->setDrive(225,FORWARD);
                  rrMotor->setDrive(225,FORWARD);   
                  lfMotor->setDrive(225,FORWARD);
                  lrMotor->setDrive(225,FORWARD);
                  break;
          case 66: // (C) Begin PID Loop
                  mode_toggle == false;
                  robotMode = 2;  
                  break;
          
          case 67: // (B) Begin Maze Solve
                  
                  mode_toggle == true;
                  mazeSolved = 0;
                  mazeSolver->StartSolving();
                  robotMode = 3;
                  break;

          default: // Any other character results in the robot stopping
                if (robotMode == 0)
                  {
                  lrMotor->setDrive(0,RELEASE);
                  rrMotor->setDrive(0,RELEASE);
                  lfMotor->setDrive(0,RELEASE);
                  rfMotor->setDrive(0,RELEASE);
                  break;
                  }
                else break;
        }
        index = 0;                              // Reset the message buffer index
      }
      else 
      {
        index++;
      }
    }

    //*/
    //robotMode=1;
 /* ===================================
             Auto Robot Modes
    =================================== */
    if (robotMode == 1) // Line Following Mode
    {
      //count=count+1;
      //if(update_control_flag == true)

         //if(count == 1000)
      if(update_control_flag==true)
      {
         lineFollower->isPathClear();
         update_control_flag = false;  
      }
         
      if(lineFollower->change_test())
      {
       changed = 1;
       changed = (lineFollower->follow(changed)); //This needs to be placed into a proper holder
      }
    }
    else if (robotMode == 2) // PID mode
    {
      if(first_time)
      {
        lSpeedControl->setDesiredSpeed(150);
        rSpeedControl->setDesiredSpeed(150);
        first_time=false;
      }
  
      if(update_control_flag)
      {
        lSpeedControl->Update();
        rSpeedControl->Update();
        update_control_flag = false;
      }
    }
    else if (robotMode == 3) // Maze Solving Mode
    {
    mazeSolved = (mazeSolver->SolvingStep());
    if (mazeSolved == 1)
      {
        robotMode = 0;
      }
    }
    //Serial.println(robotMode);
}


/* ===================================
             INTERRUPTS
    =================================== */
void blink()
{
      if(!scope_state)
      {
        //Rising Edge
        //start timer
        TCCR2B = 0x02; // Prescale 8
        TIMSK2 &= ~(1 << OCIE2A); // desable timer compare interrupts
        scope_state=HIGH;
      }
      else
      {
        //Falling Edge
        if(forwardSensor->IsActive())
        {
          forwardSensor->UpdatePingTime((int)TCNT2);
        }
        else if(leftSensor->IsActive())
        {
          leftSensor->UpdatePingTime((int)TCNT2);
        }
        TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupts
        //Set flag that data has changed
        scope_state=LOW;
      }
}

/* ===================================
             TIMER 2
    =================================== */
ISR(TIMER2_OVF_vect)
{
   TIFR2 = 0x00; //Timer2 INT Flag Reg: clear timer overflow flag
   
   if( forwardSensor->IsActive())
   { 
      forwardSensor->IncrementOVFcount();
   }
   else if( leftSensor->IsActive())
   {
      leftSensor->IncrementOVFcount();
   }
}

ISR(TIMER2_COMPA_vect)
{
  if(forwardSensor->IsActive())
  {
      digitalWrite(ftrigger_pin,LOW);
  }
  
  else if(leftSensor->IsActive())
  {
      digitalWrite(ltrigger_pin,LOW);
  }
  TIMSK2 &= ~(1 << OCIE2A); // desable timer compare interrupts
  //Serial.println(TIMSK2);
  TCNT2=0;
  TCCR2B=0x00;
}

/* ===================================
             TIMER 3: Main Tick (100m period)
    =================================== */
ISR(TIMER3_COMPA_vect)
{
    
    update_control_flag = true;
}

/* ===================================
             TIMER 4
    =================================== */
ISR(TIMER4_CAPT_vect)
{
  lEncoder->timerCapture();
  
}

ISR(TIMER4_OVF_vect)
{
  lEncoder->OverflowHandle();
}

/* ===================================
             TIMER 5
    =================================== */
ISR(TIMER5_CAPT_vect)
{
  rEncoder->timerCapture();
}

ISR(TIMER5_OVF_vect)
{
  rEncoder->OverflowHandle();
}
