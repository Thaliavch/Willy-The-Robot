// By: Ryan Boyle and Thalia Valle Chavez
// Desc: Write a program to control a robot using different states.
// The robot has an ATMega328p IC, DC Motor, Ultrasonic Sensor, 
// and Servo Motor.
// ------------------------------------------------------------
#include <Servo.h>
#include <IRremote.h>

const int H1A = 4;
const int H2A = 7;
const int H12EN = 5;

const int H3A = 8;
const int H4A = 2;
const int H34EN = 6;

const int IR_RECEIVE_PIN = 13;// have to check for pin in robot
byte SPEED = 95; // wheels speed
const int TM2_COUNT = 0x64;

 enum AutoState { // states when in auto mode
  MoveForward,
  TurnLeft,
  TurnRight,
  CorrectLeft,
  CorrectRight,
  Reverse,
  Stop
  }; // enum AutoState

  AutoState state; 

  byte forward_count = 0;
  byte left_count = 0;
  byte right_count = 0;
// count variables increment for consecutive times where
// forward/left/right is < safe value
// put in reverse when count is high enough



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
       analogWrite(_enable, LOW);
       break;
     case MotorForward:
       digitalWrite(_pin1, HIGH);
       digitalWrite(_pin2, LOW);
       analogWrite(_enable, _speed);
       break;
     case MotorReverse:
       digitalWrite(_pin1, LOW);
       digitalWrite(_pin2, HIGH);
       analogWrite(_enable, _speed);
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


class Wheels{
  Motor leftMotor;
  Motor rightMotor;
  byte speed;// = 95;
  // count variables increment for consecutive times where
  // forward/left/right is < safe value
  // put in reverse when count is high enough


  public:
   Wheels(){
    
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);


  }
  Wheels(Motor left, Motor right, byte s){

    leftMotor = left;
    rightMotor = right;
    speed = s;
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);


  }

void turnLeft() {
  speed = 95; // slow down when turning
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorReverse);
  rightMotor.run(Motor::MotorForward);
  // ^ set motors to turn left
  delay(500); // wait half a second
  state = MoveForward; // return to forward
}

void turnRight() {
  speed = 95; // slow down when turning
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorReverse);
  // ^ set motors to turn right
  delay(500); // wait half a second

  state = MoveForward; // return to forward
}

void correctLeft() {
  speed = 95; // slow down when correcting
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorReverse);
  rightMotor.run(Motor::MotorForward);
  // ^ set motors to turn left
  delay(300); // wait 300 ms

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorForward);
  delay(100); // move forward for 100 ms

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorReverse);
  // ^ set motors to turn right
  // in order to straighten out
  delay(100); // wait 100 ms

  state = MoveForward; // return to forward
}
void correctRight() {
  speed = 95; // slow down when correcting
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorReverse);
  // ^ set motors to turn right
  delay(300); // wait 300 ms

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorForward);
  delay(100); // move forward for 100 ms

  leftMotor.run(Motor::MotorReverse);
  rightMotor.run(Motor::MotorForward);
  // ^ set motors to turn left
  // in order to straighten out
  delay(100); // wait 100 ms

  state = MoveForward; // return to forward
}

void moveForward() {
  speed = 95; 
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorForward);
  // ^ return speed to 100 if slowed by a turn

  forward_count = 0;
  left_count = 0;
  right_count = 0;
  // clear counts, robot is not stuck
}

void reverse() {
  speed = 255;
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.run(Motor::MotorReverse);
  rightMotor.run(Motor::MotorForward);

  delay(300);

  leftMotor.run(Motor::MotorReverse);
  rightMotor.run(Motor::MotorReverse);

  delay(1500);

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorReverse);
  delay(300);

  state = MoveForward; // return to forward
}

void stop() {
  leftMotor.run(Motor::MotorStop);
  rightMotor.run(Motor::MotorStop);
}

};




class RemoteControl{

  enum RemoteState{
  SwitchMode,// assign given signal from remote control
  moveForward,
  correctLeft,
  correctRight,
  reverse,
  stop
  };
  RemoteState state;
  bool auto_mode;
  Wheels wheels;

  public:
  // default constructor
  RemoteControl(){
    IrReceiver.begin(IR_RECEIVE_PIN, true);
    auto_mode = true;
    state = stop;
  } 
  // Contructor to set pin
  RemoteControl(int pin, Wheels w){
    IrReceiver.begin(pin, true);
    auto_mode = true;
    state = stop;
    wheels = w;
  } 

  bool currentMode(){
  if(IrReceiver.decode()){
    if(IrReceiver.decodedIRData.command == SwitchMode){
      auto_mode = !auto_mode; 
      // stop robot for a second
       wheels.stop();
       delay(1200);
       return auto_mode; 
    }
    IrReceiver.resume();
  }

  return auto_mode;
  }


  // function to update state
  void updateState(){
  if(IrReceiver.decode()){
    state = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
  }
// Get the moveForward() and other methods from outside the class by inheritance
  switch(state) {
    case moveForward:
      wheels.moveForward();
      break;
    case correctLeft:
      wheels.correctLeft();
      break;
    case correctRight:
      wheels.correctRight();
      break;
    case reverse:
      wheels.reverse();
      break;
    case stop:
      wheels.stop();
      break;
    }

  }


};




// Instance of RemoteControl class



// declare 2 Motor variables
Motor leftMotor(H2A,H1A,H12EN);
Motor rightMotor(H3A,H4A,H34EN);

Wheels wheels(leftMotor, rightMotor, SPEED);

RemoteControl remote(IR_RECEIVE_PIN, wheels);

Servo servo; // servo object
byte servo_pin = A2;

byte trigger_pin = A1;
byte echo_pin = A0;
UltrasonicSensor sensor(trigger_pin, echo_pin); 
// object for the ultrasonic sensor, with a trigger pin of A1
// and an echo pin of A0




/*
void turnLeft();
void turnRight();
void correctLeft();
void correctRight();*/
// ^ functions to call when in corresponding state
/*
void setup()
{
  Serial.begin(9600);
  
  //leftMotor = Motor(H2A,H1A,H12EN);
  //rightMotor = Motor(H3A,H4A,H34EN);
  //leftMotor.setSpeed(speed);
  //rightMotor.setSpeed(speed);

  // set up servo (pin, min, and max)
  // min and max are pulse width values
  // used to correctly move the servo between
  // positions 0 and 180.
  servo.attach(servo_pin, 500, 2500);

  servo.write(90); // start with head straight forward

  // set up timer 2 for 9984 microseconds (~0.01 s)
  TCNT2 = TM2_COUNT;
  TCCR2A = 0x00; // set to normal mode
  TCCR2B = 0x07; // set 1024 prescaler
  TIMSK2 = (1 << TOIE2); // enable Timer Overflow Interrupt flag
  
  interrupts(); // turn interrupts on


  // IR Receiver for remote control
  //IrReceiver.begin(IR_RECEIVE_PIN, true); // this will be also initialized inside the remote class
  //auto_mode = true; // robot starts in auto mode // used inside RemoteControl class

  leftMotor.run(Motor::MotorForward);
  rightMotor.run(Motor::MotorForward);  
  // ^ start off by moving forward
}

*/
//***********************************************************************
//int btn; // should be enum

// automatic and manual mode

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
  
}
/*
// have a switch method to stop when change happens.
void loop()
{

  
  // getting the mode -> auto or manual (auto: 1, manual: 0)
  bool is_auto_mode = remote.currentMode();

if(is_auto_mode){// then we are in auto_mode
  switch(state) {
    case MoveForward:
      moveForward();
      break;
    case TurnLeft:
      turnLeft();
      break;
    case TurnRight:
      turnRight();
      break;
    case CorrectLeft:
      correctLeft();
      break;
    case CorrectRight:
      correctRight();
      break;
    case Reverse:
      reverse();
      break;
    case Stop:
      stop();
      break;
  }

}else{// manual mode

    remote.updateState();

  }



}
*/

byte currentAngle;
byte left; // left measurement
byte forward; // forward measurement

const byte WALL_DISTANCE = 6; // should always be 6 inches from wall
const byte RANGE = 1; // can be + or - 1 inch from WALL_DISTANCE
const byte COURSE_WIDTH = 28;

/*byte forward_count = 0;
byte left_count = 0;
byte right_count = 0;*/
// count variables increment for consecutive times where
// forward/left/right is < safe value
// put in reverse when count is high enough

float scan_time = 1.5;

// timer interrupt occurs every 0.01s
// decrement time variables to have 
ISR(TIMER2_OVF_vect) {
  if(scan_time < 0.01) // bring back to 1.5 s when 0
  {
    Serial.println("hi");
    scan_time = 1.5; 
    scan();
  }

  scan_time = scan_time - 0.01;
  TCNT2 = TM2_COUNT;
}

void scan() {
  currentAngle = servo.read();
  //Serial.println(currentAngle);
  if(currentAngle == 90)
  { // if sensor is currently facing forward,
    // scan forward then turn left and scan
    forward = sensor.measureInches();
    servo.write(180);
    delay(625); // wait for servo to turn
    left = sensor.measureInches();
  }
  else
  { // if sensor is currently facing left,
    // scan left then turn forward and scan
    left = sensor.measureInches();
    servo.write(90);
    delay(625); // wait for servo to turn
    forward = sensor.measureInches();
  }

  if(forward < 12) // forward < 12 inches
  {
    forward_count++;
    if(forward_count > 2) state = Reverse;
    // ^ put robot in reverse if it is stuck
    else if(left < COURSE_WIDTH) state = TurnRight;
    // ^ if approaching forward wall and left wall exists,
    // turn right
    else state = TurnLeft;
    // ^ if approaching forward wall and no close left wall,
    // turn left
  }

  else if(left < WALL_DISTANCE - RANGE)
  {
    left_count++;
    if(left_count > 2) state = Reverse;
    // ^ put robot in reverse if it is stuck
    else state = CorrectRight;
    // if too close to left wall, correct to right
  }

  else if(left > WALL_DISTANCE + RANGE)
  {
    right_count++;
    if(right_count > 2) state = Reverse;
    // ^ put robot in reverse if it is stuck
    else state = CorrectLeft;
    // if too far from left wall, correct to left
  }

  else if((left > COURSE_WIDTH && forward > COURSE_WIDTH) ||
          (left == 0 && forward == 0))
  {
    state = Stop;
    // ^ stop if both left and forward
    // are greater than the course width
    // or if both are too far (returns 0)
  }

  else state = MoveForward;
}




// Scratch Section ************************************
//  if(IrReceiver.decode()){
//    if(IrReceiver.decodedIRData.command == change){
//      auto_mode = (auto_mode == true) ? false : true; 
//      // stop robot for a second
//       leftMotor.run(MotorStop);
//       rightMotor.run(MotorStop);
//       delay(1200);
//    }
//    IrReceiver.resume();
//  }