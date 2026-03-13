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

    // Above 2 rps they vibrate pretty badly, so limit to 2 rps
    mandrelStepper.setMaxSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * MAX_REV_PER_SEC);
    carriageStepper.setMaxSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * MAX_REV_PER_SEC);

    mandrelStepper.setSpeed(0);
    carriageStepper.setSpeed(0);

    mandrelStepper.runSpeed();
    carriageStepper.runSpeed();
}

void setRps(StepperMotorParams motorParams, float revPerSec) {
    if (revPerSec > MAX_REV_PER_SEC) {
        revPerSec = MAX_REV_PER_SEC; // Cap to max speed
    }
    
    float stepsPerSec = (motorParams.microStepsPerRev * revPerSec);

    // If address of motorParams matches the motor params for a given motor, set that motor's speed
    if (&motorParams == &MANDREL_MOTOR_PARAMS) {
        mandrelStepper.setSpeed(stepsPerSec);
    } else if (&motorParams == &CARRIAGE_MOTOR_PARAMS) {
        carriageStepper.setSpeed(stepsPerSec);
    }
}

void runMotorsMaxSpeed(int dir) {
    mandrelStepper.setSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * MAX_REV_PER_SEC * dir);
    carriageStepper.setSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * MAX_REV_PER_SEC * dir);
    mandrelStepper.runSpeed();
    carriageStepper.runSpeed();
}