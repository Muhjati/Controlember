/********************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************/

#include <PID_v1_bc.h> // https://github.com/drf5n/Arduino-PID-Library
#include <EasyUltrasonic.h>
#include <Servo.h>

#define TRIGPIN 8 // Digital pin connected to the trig pin of the ultrasonic sensor
#define ECHOPIN 7
#define PIN_OUTPUT 2

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=10, aggKi=1, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
EasyUltrasonic ultrasonic;
Servo myservo; 
int pos = 90; 
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  Serial.begin(9600);
  myservo.attach(PIN_OUTPUT);
  ultrasonic.attach(TRIGPIN, ECHOPIN);
  //initialize the variables we're linked to
  Input =  ultrasonic.getDistanceCM();
  Setpoint = 25;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input =  ultrasonic.getDistanceCM();
  Serial.println(Input);
  delay(100);
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();

  int iku = Output;
  if(iku >= 118){
    iku = 118;
  }
  //Serial.println(iku);
  myservo.write(iku); 
  Serial.println();
}