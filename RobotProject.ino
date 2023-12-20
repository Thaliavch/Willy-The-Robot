// RobotProject.ino
// By: Ryan Boyle and Thalia Valle Chavez
// Desc: Write a program to control a robot using different states.
// The robot has an ATMega328p IC, DC Motor, Ultrasonic Sensor, 
// IR Sensor, and Servo Motor.
// ------------------------------------------------------------
#include <IRremote.h>
#include <Servo.h>
#include "Motor.h"
#include "UltrasonicSensor.h"
#include "RemoteControl.h"

// left motor pins
const int H1A = 4;
const int H2A = 7;
const int H12EN = 5;

// right motor pins
const int H3A = 2;
const int H4A = 8;
const int H34EN = 6;

// pins for LEDs that turn on
// when turning or correcting left/right
// in automatic mode
const int LEFT_LED = 11; // red LED
const int RIGHT_LED = 9; // blue LED

const int IR_RECEIVE_PIN = 12;
byte SPEED_LEFT = 104; // left wheel moves slower than right wheel so we will give it more velocity
byte SPEED_RIGHT = 100;
const int TM2_COUNT = 0xF9; // for timer 2

 enum AutoState { // states when in auto mode
  Scan,
  MoveForward,
  TurnLeft,
  TurnRight,
  CorrectLeft,
  CorrectRight,
  Reverse,
  ReverseLeft,
  ReverseRight,
  Stop
  }; // enum AutoState

  AutoState state; 

// declare 2 Motor variables
Motor leftMotor(H2A,H1A,H12EN);
Motor rightMotor(H3A,H4A,H34EN);

Wheels wheels(leftMotor, rightMotor, SPEED_LEFT, SPEED_RIGHT);

// Instance of RemoteControl class 
RemoteControl remote(IR_RECEIVE_PIN, wheels);

Servo servo; // servo object
const int SERVO_PIN = A2;

// object for the ultrasonic sensor, with a trigger pin of A1
// and an echo pin of A0
const int TRIGGER_PIN = A1;
const int ECHO_PIN = A0;
UltrasonicSensor sensor(TRIGGER_PIN, ECHO_PIN); 

// getting the mode -> auto or manual (auto: 1, manual: 0)
bool is_auto_mode = 1;

void setup()
{
  Serial.begin(9600);

  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);

  setupTimer2();

  state = MoveForward; 
  // ^ start off by moving forward
}

// loop to use when utilizing both modes (auto and manual)
void loop()
{
  // test if remote switchMode button is pressed
  is_auto_mode = remote.currentMode();

  if(is_auto_mode) { // then we are in auto_mode
    // set up timer 2 again in case
    // last loop was in manual mode
    // which uses the remote
    setupTimer2();

    // choose correct function
    // for correct state
    switch(state) {
      case Scan:
        scan();
        break;
      case MoveForward:
        wheels.moveForward();
        break;
      case TurnLeft:
        digitalWrite(LEFT_LED, HIGH);
        wheels.turnLeft();
        digitalWrite(LEFT_LED, LOW);
        state = Scan;
        break;
      case TurnRight:
        digitalWrite(RIGHT_LED, HIGH);
        wheels.turnRight();
        digitalWrite(RIGHT_LED, LOW);
        state = Scan;
        break;
      case CorrectLeft:
        digitalWrite(LEFT_LED, HIGH);
        wheels.correctLeft();
        digitalWrite(LEFT_LED, LOW);
        state = Scan;
        break;
      case CorrectRight:
        digitalWrite(RIGHT_LED, HIGH);
        wheels.correctRight();
        digitalWrite(RIGHT_LED, LOW);
        state = Scan;
        break;
      case Reverse:
        wheels.reverse();
        state = Scan;
        break;
      case ReverseLeft:
        wheels.reverseLeft();
        state = Scan;
        break;
      case ReverseRight:
        wheels.reverseRight();
        state = Scan;
        break;
      case Stop:
        wheels.stop();
        break;
    }

  } else { // manual mode

    remote.updateState(); // switch to state of currently pressed button
    state = Scan; // have it scan when it goes back to auto mode
    }

}

// will scan every 20 interrupts, roughly 0.32s
float scanTime = 20;

// timer interrupt occurs every 0.016s
// decrement time variables to have
// longer increments before actions occur

// after adding the remote, sometimes the interrupt
// takes longer to do
ISR(TIMER2_COMPA_vect) {
  if(scanTime < 1) // bring back to 20 when 0
  {
    scanTime = 20; 
    state = Scan; // set state to scan
  }

  scanTime = scanTime - 1;

  // restart timer
  TCNT2 = 0;
  OCR2A = TM2_COUNT;
}


// variables for scan function
  
byte currentAngle;
byte left; // left measurement
byte left2; // second left measurement
byte forward; // forward measurement

const byte WALL_DISTANCE = 14; // robot should always be around 14 inches from left wall
const byte RANGE = 4; // can be + - 4 inches from WALL_DISTANCE
const byte COURSE_WIDTH = 28;


// count variable increments for consecutive times where
// forward measurement is < safe value
// put in reverse when count is high enough
byte forwardCount = 0;

void scan() {
  // set up servo (pin, min, and max)
  // min and max are pulse width values
  // used to correctly move the servo between
  // positions 0 and 180.
  servo.attach(SERVO_PIN, 500, 2500);
  currentAngle = servo.read();

  if(currentAngle == 90)
  { // if sensor is currently facing forward,
    // scan forward then turn left and scan
    forward = sensor.measureInches();
    servo.write(180);
    delay(625); // wait for servo to turn
    left = sensor.measureInches();
    delay(300); // wait to measure left again
    left2 = sensor.measureInches();
  }
  else
  { // if sensor is currently facing left,
    // scan left then turn forward and scan
    left = sensor.measureInches();
    delay(300); // wait to measure left again
    left2 = sensor.measureInches();
    servo.write(90);
    delay(625); // wait for servo to turn
    forward = sensor.measureInches();
  }
  servo.detach();
  // set up timer again after servo detach
  setupTimer2();


  if(forward < 20) // forward < 20 inches, any less may not be enough time
  {
    if(forward <= 3) forwardCount++;
    if(forwardCount > 3) state = Reverse;
    // ^ put robot in reverse if it is stuck/close to wall
    else if(left < COURSE_WIDTH) 
    {
      state = TurnRight;
      // turn right since forward and left are both close
    }
    else state = TurnLeft;
    // ^ if approaching forward wall and no close left wall,
    // turn left
  }

  else if(left < WALL_DISTANCE - RANGE || left2 - left < -3)
  {
    forwardCount = 0;
    // ^ should not be stuck in forward direction
    state = CorrectRight;
    // if too close to left wall, or if 2nd left measurement is much less than first
    // correct to right
  }

  else if(left > WALL_DISTANCE + RANGE || left2 - left > 3)
  {
    forwardCount = 0;
    // ^ should not be stuck in forward direction
    state = CorrectLeft;
    // if too far from left wall, or if 2nd measurement is much greater than first
    // correct to left
  }

  else if((left > COURSE_WIDTH && forward > COURSE_WIDTH) ||
          (left == 0 && forward == 0))
  {
    state = Stop;
    // ^ stop if both left and forward
    // are greater than the course width
    // or if both are too far (returns 0)
  }

  else 
  {
    forwardCount = 0; // not stuck
    state = MoveForward;
  }

}

void setupTimer2(){

  // set up timer 2 for 16000 microseconds
  // using CTC mode
  TCNT2 = 0;
  OCR2A = TM2_COUNT;
  TCCR2A = 0x02; // set to CTC mode
  TCCR2B = 0x07; // set 1024 prescaler
  TIMSK2 = (1 << OCIE2A); // enable Timer Overflow Interrupt flag

  interrupts();
}