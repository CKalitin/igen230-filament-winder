#pragma once

// mandrel motor pins
// carriage motor pins
// limit switch pin
// led pin
// led blink period ms

// belt pitch mm
// motor-shaft pulley teeth
// mandrel pulley teeth
// carriage pulley teeth

// mandrel gear ratio = mandrel_pulley_teeth / motor_shaft_pulley_teeth

// carriage mm per revolution = belt_pitch * carriage_pulley_teeth

// speed

#define LIMIT_SWITCH_PIN 4
#define LED_PIN 2

#define LED_BLINK_INTERVAL_MS 500.0f

#define BELT_PITCH_MM 2.0f
#define MOTOR_PULLEY_TEETH 20.0f
#define MANDREL_PULLEY_TEETH 48.0f
#define CARRIAGE_PULLEY_TEETH 20.0f

// Mandrel gear ratio  (driven / driver).
inline float getMandrelGearRatio() {
    return (float)MANDREL_PULLEY_TEETH / MOTOR_PULLEY_TEETH;
}

// getter for carriage mm per revolution
inline float getCarriageMMPerRev() {
    return (float)CARRIAGE_PULLEY_TEETH * BELT_PITCH_MM / MOTOR_PULLEY_TEETH;
}

float getStepRatio(float mandrel_diameter);