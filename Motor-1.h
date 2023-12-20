// Motor.h
// By: Ryan Boyle and Thalia Valle Chavez
// Define class Motor
//    properties: pin1, pin2, enable, speed
//    methods: constructors, getter/setter for speed, run
// Desc: Class containing all the attributes and abilities of the DC motor
#ifndef _MOTOR_H
#define _MOTOR_H

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
  // ^ default constructor
  
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
    } // switch dir, sets spin direction of motor
  } // end run
  
}; // end Motor
// left: left motor reverse, right motor forward
// right: left motor forward, right motor reverse

#endif