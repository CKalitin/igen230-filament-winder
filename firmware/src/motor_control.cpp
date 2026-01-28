// Include the motor control header
#include "motor_control.h"

// Global stepper instances bound to their configured pins
AccelStepper mandrelStepper(AccelStepper::DRIVER, MANDREL_MOTOR_PARAMS.step_pin, MANDREL_MOTOR_PARAMS.dir_pin);
AccelStepper carriageStepper(AccelStepper::DRIVER, CARRIAGE_MOTOR_PARAMS.step_pin, CARRIAGE_MOTOR_PARAMS.dir_pin);

void initSteppers() {
    mandrelStepper.setCurrentPosition(0);
    carriageStepper.setCurrentPosition(0);

    mandrelStepper.setPinsInverted(false, false, true); // Make EN pin active low
    mandrelStepper.setEnablePin(MANDREL_MOTOR_PARAMS.enable_pin);
    mandrelStepper.enableOutputs();

    carriageStepper.setPinsInverted(false, false, true); // Make EN pin active low
    carriageStepper.setEnablePin(CARRIAGE_MOTOR_PARAMS.enable_pin);
    carriageStepper.enableOutputs();

    mandrelStepper.setMaxSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * 5);
    carriageStepper.setMaxSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * 5);

    // Above 2 rps they vibrate pretty badly, so limit to 2 rps
    mandrelStepper.setSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * 2);
    carriageStepper.setSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * 2);
}