#pragma once

#include "motor_control.h"

/// @brief Parameters for one winding layer.
///
/// Store one instance per layer in an array to build a full winding profile.
struct Layer {
	float length_mm;    // axial travel length for this layer
	float angle_deg;    // target winding angle (+/- for direction)
	float offset_mm;    // start offset from reference origin
	float stepover_mm;  // lateral shift relative to the previous layer

	Layer() : length_mm(0.0f), angle_deg(0.0f), offset_mm(0.0f), stepover_mm(0.0f) {}

	Layer(float length, float angle, float offset, float stepover)
		: length_mm(length), angle_deg(angle), offset_mm(offset), stepover_mm(stepover) {}
};

float current_carriage_pos_mm = 0.0f; // Track current carriage position for limit switch and layer transitions

// Winding Profile Definitions:

float mandrel_diameter_mm = 50.0f; // example default, set as needed
float dwell_deg = 180.0f; // example default, set as needed
float fiber_width_mm = 3.0f; // example default, set as needed
float motor_speed = 1.0f; // rev/s

int currentLayer = 0;
Layer layers[] = {
    // length_mm, angle_deg, offset_mm, stepover_mm
    Layer(100.0f, 45.0f, 0.0f, 2.0f),
    Layer(100.0f, -45.0f, 1.0f, 2.0f),
    // Add more layers as needed
};

enum FSMState {
    PAUSED,
    ZEROING,
    MOVING,
    DWELLING,
    FINISHED
};

FSMState fsm_state = PAUSED;

void FSM_Run();

void FSM_SwitchState(FSMState newState);

void FSM_Paused();
void FSM_Zeroing();
void FSM_Moving();
void FSM_Dwelling();
void FSM_Finished();

