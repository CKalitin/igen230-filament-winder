#pragma once

#include <stdint.h>
#include <AccelStepper.h>
#include "config.h"

#define MANDREL_MOTOR_STEP_PIN 14
#define MANDREL_MOTOR_DIR_PIN 17
#define MANDREL_MOTOR_ENABLE_PIN 13

#define CARRIAGE_MOTOR_STEP_PIN 25
#define CARRIAGE_MOTOR_DIR_PIN 26
#define CARRIAGE_MOTOR_ENABLE_PIN 27

// We can't control number of microsteps, define defaults here
#define TMCS2209_MICROSTEPS 8
#define TMC2225_MICROSTEPS 4

#define MOTOR_STEPS_PER_REV 200

#define MAX_REV_PER_SEC 2.0f

struct StepperMotorParams {
	uint8_t step_pin;           // step signal pin
	uint8_t dir_pin;            // direction signal pin
	uint8_t enable_pin;         // enable signal pin (active low typically)
	uint16_t stepsPerRev;       // mechanical full steps per revolution
	uint16_t microsteps;        // microsteps configured per full step
	uint16_t microStepsPerRev;  // derived: total microsteps per revolution
	uint8_t forward_dir;        // 1 for normal, -1 for reversed direction, in case wiring is inconsistent

	StepperMotorParams(uint8_t step, uint8_t dir, uint8_t enable = 1, uint16_t steps = 200, uint16_t micro = 8, uint8_t forward = 1) {
		step_pin = step;
		dir_pin = dir;
		enable_pin = enable;
		stepsPerRev = steps;
		microsteps = micro;
		microStepsPerRev = stepsPerRev * microsteps;
		forward_dir = forward;
	}
};

const StepperMotorParams MANDREL_MOTOR_PARAMS(MANDREL_MOTOR_STEP_PIN, MANDREL_MOTOR_DIR_PIN, MANDREL_MOTOR_ENABLE_PIN, MOTOR_STEPS_PER_REV, TMCS2209_MICROSTEPS, 1);
const StepperMotorParams CARRIAGE_MOTOR_PARAMS(CARRIAGE_MOTOR_STEP_PIN, CARRIAGE_MOTOR_DIR_PIN, CARRIAGE_MOTOR_ENABLE_PIN, MOTOR_STEPS_PER_REV, TMCS2209_MICROSTEPS, 1);

// Global stepper objects (defined in motor_control.cpp)
extern AccelStepper mandrelStepper;
extern AccelStepper carriageStepper;

// Initialize stepper instances with the configured pins/params
void initSteppers();

void setRps(StepperMotorParams motorParams, float rpm);

// Run both motors at max speed (call every loop iteration while active)
void runMotorsMaxSpeed(int dir = 1);