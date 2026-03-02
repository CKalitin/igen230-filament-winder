// Parameters describing a stepper motor's resolution and pinning
#pragma once

#include <stdint.h>
#include <AccelStepper.h>

struct StepperMotorParams {
	uint8_t step_pin;           // step signal pin
	uint8_t dir_pin;            // direction signal pin
	uint8_t enable_pin;         // enable signal pin (active low typically)
	uint16_t stepsPerRev;       // mechanical full steps per revolution
	uint16_t microsteps;        // microsteps configured per full step
	uint16_t microStepsPerRev;  // derived: total microsteps per revolution

	StepperMotorParams(uint8_t step, uint8_t dir, uint8_t enable = 1, uint16_t steps = 200, uint16_t micro = 8) {
		step_pin = step;
		dir_pin = dir;
		enable_pin = enable;
		stepsPerRev = steps;
		microsteps = micro;
		microStepsPerRev = stepsPerRev * microsteps;
	}
};

// step dir enable
// 25 26 27
// 14 17 13

// Default parameters per motor
// step, dir, enable
const StepperMotorParams MANDREL_MOTOR_PARAMS(25, 26, 27, 200, 8); // TMCS2209 (8 microsteps default)
const StepperMotorParams CARRIAGE_MOTOR_PARAMS(14, 17, 13, 200, 4); // TMC2225 (4 microsteps default)

// Global stepper objects (defined in motor_control.cpp)
extern AccelStepper mandrelStepper;
extern AccelStepper carriageStepper;

// Initialize stepper instances with the configured pins/params
void initSteppers();