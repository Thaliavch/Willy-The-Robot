// RemoteControl.h
// By: Ryan Boyle and Thalia Valle Chavez
// Define class RemoteControl
//    properties: RemoteState enum, state, auto_mode, receive_pin, wheels
//    methods: constructors, currentMode, updateState
// Desc: Class containing all the attributes and abilities of the remote control
#include "Wheels.h"

class RemoteControl {

  enum RemoteState{
  switchMode = 66, // * button
  forward = 70, // up arrow
  left = 68, // left arrow
  right = 67, // right arrow
  reverse = 21, // down arrow
  stop = 64  // OK button
  };
  RemoteState state; // enum
  bool auto_mode; // this is the var we return in the update function each time
  int receive_pin;
  Wheels wheels;

  public:
    // default constructor
    // when object is initialized, it is initialized to auto mode
    RemoteControl(){
      auto_mode = true;
      state = stop;
      receive_pin = 0;
    } 

    // Constructor to set pin
    RemoteControl(int pin, Wheels w){
      auto_mode = true;
      state = stop;
      wheels = w;
      receive_pin = pin;
    } 

    bool currentMode(){ // switch mode if specific button pressed
      IrReceiver.begin(receive_pin, true);
      delay(500);
    if(IrReceiver.decode()) {
      if(IrReceiver.decodedIRData.command == RemoteState::switchMode) {
        auto_mode = !auto_mode; 
        // stop robot for a bit
        
        wheels.stop();
        delay(500);
          
      }
      IrReceiver.resume();
      
    }
    IrReceiver.end();
    return auto_mode;
    }

    // function to update state based on button press
    void updateState(){
      IrReceiver.begin(receive_pin, true);
      delay(500);
    if(IrReceiver.decode()) {
      state = IrReceiver.decodedIRData.command;
      IrReceiver.resume();
    }
      IrReceiver.end(); 
    // Get the moveForward() and other methods from outside the class through Wheels object
    switch(state) {
      case forward:
        Serial.println("Move Forward");
        wheels.moveForward();
        break;
      case left:
        Serial.println("Move Left");
        wheels.remoteTurnLeft();
        break;
      case right:
        Serial.println("Move Right");
        wheels.remoteTurnRight();
        break;
      case reverse:
        Serial.println("Reverse");
        wheels.remoteReverse();
        break;
      case stop:
        Serial.println("Stop");
        wheels.stop();
        break;
      }

    }

}; // end RemoteControl