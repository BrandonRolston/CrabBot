/*
 * Lab 3: RangeFinder Testing
 * 
 *  Resources: Timer2 & Digital pins (2,3,4,5)
 *  Note: To increase accurecy, we can remove the Serial.print statements for the
 *        two testing pins. This would speed up those sections and make them more
 *        accurant.
 *        Currently within an 1 in of the correct measurment. 
 */

#include <RangeFinder.h>


//Inputs
float tolerance= 5.0; //In ft
int units_type=1; // Currently not used internally.
const byte trigger_pin = 2;
const byte echo_pin = 3;

//Testing Pins
const byte testpin= 4;  //Outputs the Arduino signal from the echo_pin
const byte countpin=5;  //Toggles when the Overflow Interrupt triggers for Timer2
bool testing=false;      // Toggle this to get more information

//Loop controls
int count=1;
volatile byte scope_state=LOW;
volatile byte count_state=1;
bool my_state;
float distance=0.0;

RangeFinder* myRanger = new RangeFinder(tolerance,units_type, trigger_pin,echo_pin, testing);


void setup() {
  // put your setup code here, to run once:
  
  interrupts();
  Serial.begin(115200);
  pinMode(countpin, OUTPUT);
  pinMode(echo_pin,INPUT);
  pinMode(testpin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(echo_pin), blink, CHANGE);
  scope_state=0;
}

void loop() {

     if(count ==1000)
     {
     myRanger->EnquireForDistance();  //Ask to get distance
     }
     
     if(myRanger->HasData()) //Data is ready to be used.
     {
        my_state = myRanger->isPathClear();   //Ask if the path is clear (using tolerance)
        distance = myRanger->ReportDistance();// Get the distance calulated
        
        if(!testing) // If we are in normal mode then print these
        {
          Serial.print("D=");
          Serial.println(distance);
          Serial.print("IsClear=");
          Serial.println(my_state);
        }
        count=0;  //Set count to zero to get ready to generate the pulse again.
     }
     
     count=count+1;
}

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
        myRanger->UpdatePingTime((int)TCNT2);
        TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupts
        //Set flag that data has changed
        scope_state=LOW;
      }
      digitalWrite(testpin,scope_state);
}

ISR(TIMER2_COMPA_vect)
{
  digitalWrite(trigger_pin,LOW);
  TIMSK2 &= ~(1 << OCIE2A); // desable timer compare interrupts
  //Serial.println(TIMSK2);
  TCNT2=0;
  TCCR2B=0x00;
}

ISR(TIMER2_OVF_vect)
{
   TIFR2 = 0x00; //Timer2 INT Flag Reg: clear timer overflow flag
   
   if(count_state == 1)
   { 
       digitalWrite(countpin,HIGH);
       count_state=0;
   }
   else
   {
       digitalWrite(countpin,LOW);
       count_state=1;
   }
   myRanger->IncrementOVFcount();
}
