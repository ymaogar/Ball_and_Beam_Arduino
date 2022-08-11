#include<Servo.h>
#include<PID_v1.h>
#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int servoPin = 9;                                               //Servo Pin

float Kp = 1.8;                                                    //Initial Proportional Gain
float Ki = 0.6;                                                      //Initial Integral Gain
float Kd = 1.1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                                                                                      
Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600); //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-70,70);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
  Setpoint = 19;
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  
  ServoOutput=81+Output;                                            // 102 degrees is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
}
      
      

float readPosition() 
{
  delay(40);                                                            //Don't set too low or echos will run into eachother.      
  long cm;
  cm = sonar.convert_cm(sonar.ping_median(5));
  
  if(cm > 40)     // 40 cm is the maximum position for the ball
  {cm=40;}
  
  Serial.println(cm);
  return cm;                                          //Returns distance value.
}
