// By: Ryan Boyle
// Desc: Abstract a DC Motor driven by 1 half of an H-Bridge
// ------------------------------------------------------------
#include <Servo.h>

const int H1A = 4;
const int H2A = 7;
const int H12EN = 5;

const int H3A = 8;
const int H4A = 2;
const int H34EN = 6;


// define class Motor
//    properties: pin1, pin2, enable, speed
//    methods: constructors, getter/setter for speed, run
class Motor {
private:
  byte _pin1;
  byte _pin2;
  byte _enable;
  byte _speed;
  
public:
  enum MotorDir {
    MotorStop,
    MotorForward,
    MotorReverse
  }; // enum MotorDir
  
  Motor() : _pin1(0), _pin2(0), _enable(0), _speed(0) { }
  
  Motor(byte pin1, byte pin2, byte enable)
    : _pin1(pin1), _pin2(pin2), _enable(enable) {
      pinMode(_pin1, OUTPUT);
      pinMode(_pin2, OUTPUT);
      pinMode(_enable, OUTPUT);
    } // end Motor prop constructor
  
  byte getSpeed() { return _speed; }
  void setSpeed(byte speed) { _speed = speed; }
  
  void run(Motor::MotorDir dir) {
    switch(dir) {
     case MotorStop:
       digitalWrite(_pin1, LOW);
       digitalWrite(_pin2, LOW);
       digitalWrite(_enable, LOW);
       break;
     case MotorForward:
       digitalWrite(_pin1, HIGH);
       digitalWrite(_pin2, LOW);
       digitalWrite(_enable, _speed);
       break;
     case MotorReverse:
       digitalWrite(_pin1, LOW);
       digitalWrite(_pin2, HIGH);
       digitalWrite(_enable, _speed);
    } // switch dir
  } // end run
  
}; // end Motor
// left : left motor reverse, rightmotor forward
// right: left motor forward, rightmotor reverse

class UltrasonicSensor {
  private:
  	byte triggerPin;
  	byte echoPin;
 	const int MAX_DURATION = (14)*12*74*2; 
  	// max duration for the sensor, 14ft chosen as max
  
  public:
  	UltrasonicSensor() {
      triggerPin = 0;
      echoPin = 0;
  	} // default constructor
  
    UltrasonicSensor(byte signalPin) {
      triggerPin = echoPin = signalPin;
  	} // signal pin constructor  
  
    UltrasonicSensor(byte trigger, byte echo) {
      triggerPin = trigger;
      echoPin = echo;
  	} // separate trigger and echo pin constructor
  
 	unsigned long measure() {
      unsigned long duration = 0;
      noInterrupts(); // do not want interrupts when measuring
      // Setup 				  				  ---------------
      pinMode(echoPin, OUTPUT);
      digitalWrite(echoPin, LOW); 
      // ^make sure input signal starts low
      // Trigger a measurement 				  ---------------
      pinMode(triggerPin, OUTPUT);
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(5);
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
      // Wait for measurement complete signal ---------------
      pinMode(echoPin, INPUT);
      duration = pulseIn(echoPin, HIGH, MAX_DURATION);
      // ^ read a pulse on the echo pin
      //									  ---------------
      interrupts(); // turn interrupts on
      if(duration > 0)
        duration /= 2; // get 1-way duration
      return duration;
  }
  
   	double measureInches() {
      double inches = 0;
      unsigned long duration = measure();
      if(duration > 0)
        inches = duration / 74.0; // get inches from duration
      return inches;
  }
  
   	double measureCentimeters() {
      double centimeters = 0;
      unsigned long duration = measure();
      if(duration > 0)
        centimeters = duration / 29.1; 
      // ^ get inches from duration
      return centimeters;
  }
}; // end UltrasonicSensor



// declare 2 Motor variables
Motor leftMotor;
Motor rightMotor;

Servo servo; // servo object
byte servo_pin = A2;

byte trigger_pin = A1;
byte echo_pin = A0;
UltrasonicSensor sensor(trigger_pin, echo_pin); 
// object for the ultrasonic sensor, with a trigger pin of A1
// and an echo pin of A0

byte speed = 50;

void setup()
{
  Serial.begin(9600);
  
  Serial.println("hi");
  leftMotor = Motor(H2A,H1A,H12EN);
  rightMotor = Motor(H3A,H4A,H34EN);
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);
  Serial.println("cat");

  // set up servo (pin, min, and max)
  // min and max are pulse width values
  // used to correctly move the motor to
  // position 0 and 180.
  servo.attach(servo_pin, 500, 2500);  

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorForward);  
  // ^ start off by moving forward
}


// exercise motors by running through directions: 
//	- forward, left, revese, right, stop
//  - delay between each
//  - increase speed of both motors

const int DELAY = 7000;
bool auto_mode = true;

// automatic and manual mode
/*
void setup()
{
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, true);
  
  //pinMode(13, OUTPUT);
  //pinMode(IR_RECEIVE_PIN, INPUT);
  
  
  
}

// create switch statmement using the enums

// main application
void loop()
{
  if(IrReceiver.decode()){
    Serial.println(IrReceiver.decodedIRData.command);
    IrReceiver.resume();
    
  }
  
}*/

// have a switch method to stop when change happens.
void loop()
{

// getting the mode
  if(IrReceiver.decode()){
    if(IrReceiver.decodedIRData.command == change){
      auto_mode = (auto_mode == true) ? false : true; 
      // stop robot for a second
       leftMotor.run(MotorStop);
       rightMotor.run(MotorStop);
       delay(1200);
    }
    IrReceiver.resume();
  }


// leftMotor.run(MotorStop);
// rightMotor.run(MotorStop);

if(auto_mode){// then we are in auto_mode



}else{


    switch(btn){

      case forward: 
        leftMotor.run(MotorForward);
        rightMotor.run(MotorForward);

    }



}

  servo.write(90);
  double measurement = sensor.measureInches();
    Serial.println(measurement);
  if(measurement <= 5)
  {
	speed = 10;
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorForward);
    while(sensor.measureInches() < 10); 
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
    speed = 50; 
  }
}