// Wheels.h
// By: Ryan Boyle and Thalia Valle Chavez
// Define class Wheels
//    properties: leftMotor, rightMotor, speeds
//    methods: constructors, state functions that control wheels
// Desc: Class containing all the attributes and abilities of the wheels
#include "Motor.h"

class Wheels {
  Motor leftMotor;
  Motor rightMotor;
  byte leftSpeed;
  byte rightSpeed;

  public:
   Wheels(){
    
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

  } // end default constructor

  Wheels(Motor left, Motor right, byte left_speed, byte right_speed){

    leftMotor = left;
    rightMotor = right;
    leftSpeed = left_speed;
    rightSpeed = right_speed;
    leftMotor.setSpeed(left_speed);
    rightMotor.setSpeed(right_speed);

  } // end prop constructor

  void turnLeft() {
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorForward);
    // ^ set motors to turn left
    delay(125); // turn 90 degrees

    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop
    
  }

  void turnRight() {
    leftMotor.setSpeed(leftSpeed+100);
    rightMotor.setSpeed(rightSpeed-10);
    // ^ for more accurate turning

    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorReverse);
    // ^ set motors to turn right
    delay(125); // turn 90 degrees

    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop
    
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

  }

  void correctLeft() {
    // we will stop very briefly just for sake of stability of the robot
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // turn left
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorForward);
    delay(200);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // move forward
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
    delay(600);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // turn right to correct
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorReverse);
    delay(150);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop
    
    // move forward
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
    delay(400); 
    
  }

  void correctRight() {
    // we will stop very briefly just for sake of stability of the robot
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // turn right
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorReverse);
    delay(200);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // move forward
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
    delay(600);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // turn left to correct
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorForward);
    delay(150);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop
    
    // move forward
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
    delay(400);
    
  }

  void moveForward() {
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorForward);
  }

  void reverse() {

    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100); // stop

    // reverse
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorReverse);
    delay(400);

  }

  void reverseRight(){
    
    // stop
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100);

    // reverse
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorReverse);
    delay(300);

    //stop
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100);
    
    //reverse right
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorStop);
    delay(300);

  }

  void reverseLeft(){
    // stop
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(100);

    // reverse
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorReverse);
    delay(300);

    //stop
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    delay(200);
    
    //correct left reverse 
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorReverse);
    delay(300);

  }

  void remoteTurnLeft(){
    // turn left
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorForward);
  }
  
  void remoteTurnRight(){
    // turn right
    leftMotor.run(Motor::MotorForward);
    rightMotor.run(Motor::MotorReverse);
  }
      
  void remoteReverse(){
    leftMotor.run(Motor::MotorReverse);
    rightMotor.run(Motor::MotorReverse);
  }

  void stop() {
    // since the left motor stops first then the right, when stop it turns a bit
    // lower right motor speed to fix this
    rightMotor.setSpeed(80);
    
    leftMotor.run(Motor::MotorStop);
    rightMotor.run(Motor::MotorStop);
    
    // set back to original speed
    rightMotor.setSpeed(rightSpeed);
  }

}; // end Wheels