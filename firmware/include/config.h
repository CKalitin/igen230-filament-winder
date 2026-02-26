/// @file config.h
/// @brief Central hardware configuration for the filament winder.
///
/// All pin assignments, mechanical drive-train constants, and default motion
/// parameters live here so they can be tuned in one place.

#pragma once

#include <stdint.h>

// ============================================================================
//  Limit-Switch Pins
// ============================================================================

/// Carriage home / limit switch (active LOW with internal pull-up).
constexpr uint8_t CARRIAGE_LIMIT_PIN = 16;

// ============================================================================
//  Drive-Train Mechanical Constants
// ============================================================================

constexpr float BELT_PITCH_MM         = 2.0f;  ///< GT2 belt tooth pitch (mm).
constexpr int   MOTOR_PULLEY_TEETH    = 20;    ///< Motor-shaft pulley tooth count.
constexpr int   MANDREL_PULLEY_TEETH  = 48;    ///< Mandrel driven-pulley tooth count.
constexpr int   CARRIAGE_PULLEY_TEETH = 20;    ///< Carriage driven-pulley tooth count.

/// Mandrel gear ratio  (driven / driver).
constexpr float MANDREL_GEAR_RATIO =
    static_cast<float>(MANDREL_PULLEY_TEETH) / MOTOR_PULLEY_TEETH;

/// Carriage linear travel per motor revolution (mm).
constexpr float CARRIAGE_MM_PER_MOTOR_REV =
    static_cast<float>(CARRIAGE_PULLEY_TEETH) * BELT_PITCH_MM;

// ============================================================================
//  Derived-Ratio Helper Functions
// ============================================================================

/// Compute carriage motor microsteps per millimetre of linear travel.
/// @param microStepsPerRev  Carriage motor total microsteps per revolution.
inline float computeCarriageStepsPerMM(uint16_t microStepsPerRev) {
    return static_cast<float>(microStepsPerRev) / CARRIAGE_MM_PER_MOTOR_REV;
}

/// Compute total mandrel motor microsteps per full mandrel revolution
/// (accounting for the belt/pulley gear ratio).
/// @param microStepsPerRev  Mandrel motor total microsteps per revolution.
inline float computeMandrelStepsPerRev(uint16_t microStepsPerRev) {
    return static_cast<float>(microStepsPerRev) * MANDREL_GEAR_RATIO;
}

// ============================================================================
//  Default Motion Parameters
// ============================================================================

constexpr float DEFAULT_MANDREL_SPEED      = 600.0f;   ///< Mandrel constant speed (steps/s).
constexpr float DEFAULT_MANDREL_MAX_SPEED  = 1000.0f;  ///< Mandrel maximum speed  (steps/s).
constexpr float DEFAULT_CARRIAGE_MAX_SPEED = 3000.0f;  ///< Carriage maximum speed (steps/s).
constexpr float DEFAULT_CARRIAGE_ACCEL     = 5000.0f;  ///< Carriage acceleration  (steps/s²).
constexpr float ZEROING_SPEED              = 400.0f;   ///< Carriage homing speed  (steps/s).
