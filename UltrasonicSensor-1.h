// UltrasonicSensor.h
// By: Ryan Boyle and Thalia Valle Chavez
// Define class UltrasonicSensor
//    properties: triggerPin, echoPin, MAX_DURATION
//    methods: constructors, measure, measure in inches/cm
// Desc: Class containing all the attributes and abilities of the ultrasonic sensor
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
      // Setup 				  				              ---------------
      pinMode(echoPin, OUTPUT);
      digitalWrite(echoPin, LOW); 
      // ^make sure input signal starts low
      // Trigger a measurement 				        ---------------
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
      //									                    ---------------
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
      // ^ get cm from duration
      return centimeters;
  }
}; // end UltrasonicSensor