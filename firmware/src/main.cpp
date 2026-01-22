#include <main.h>

void setup() {
	initSteppers();

	pinMode(2, OUTPUT); // LED pin
}

void loop() {
  mandrelStepper.runSpeed();
  carriageStepper.runSpeed();
}

// ---------------
// ---------------

// Below is simpler code that is confirmed to work

// ---------------
// ---------------

// #include <AccelStepper.h>

// // Define pin numbers
// const int stepPin = 25;
// const int directionPin = 26;
// const int enablePin = 27; // Optional enable pin

// // Define the stepper interface type (DRIVER for step/dir pins) and the pins
// AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);

// void setup() {
//   Serial.begin(9600);
//   // Set the enable pin and invert its logic (if the driver needs a LOW signal to be enabled)
  
//   stepper.setPinsInverted(false, false, true); // Invert enable pin logic
//   stepper.setEnablePin(enablePin);
//   stepper.enableOutputs(); // Enable the motor driver outputs

//   // set pin 2 to output and turn it on with arduino gpio
//   pinMode(2, OUTPUT); // LED pin
//   digitalWrite(2, HIGH); // Turn on LED

//   // Set the maximum speed in steps per second
//   stepper.setMaxSpeed(1000); 
//   // Set the target speed in steps per second
//   stepper.setSpeed(200); 
// }

// void loop() {
//   // Must be called repetitively to make the motor move
//   stepper.runSpeed();
// }
