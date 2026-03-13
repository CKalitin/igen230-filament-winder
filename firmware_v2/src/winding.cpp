#include "winding.h"

void FSM_Run() {
    switch (fsm_state) {
        case PAUSED:
            FSM_Paused();
            break;
        case ZEROING:
            FSM_Zeroing();
            break;
        case MOVING:
            FSM_Moving();
            break;
        case DWELLING:
            FSM_Dwelling();
            break;
        case FINISHED:
            FSM_Finished();
            break;
    }
}

void FSM_SwitchState(FSMState newState) {
    fsm_state = newState;
    Serial.print("Switched to state ");
    Serial.print(static_cast<int>(fsm_state));
    Serial.print(": ");
    switch (fsm_state) {
        case PAUSED:
            Serial.println("PAUSED");
            break;
        case ZEROING:
            Serial.println("ZEROING");
            break;
        case MOVING:
            Serial.println("MOVING");
            break;
        case DWELLING:
            Serial.println("DWELLING");
            break;
        case FINISHED:
            Serial.println("FINISHED");
            break;
    }
}

void FSM_Paused() {
    // Do nothing, wait for start command(?)
}

void FSM_Zeroing() {
    // Move backard at low speed
    setRps(MANDREL_MOTOR_PARAMS, -1 * motor_speed * 0.1);

    // if GPIO for carriage limit switch is triggered, transition to DWELLING
    if (digitalRead(CARRIAGE_LIMIT_SWITCH_PIN) == LOW) {
        current_carriage_pos_mm = 0.0f; // Reset carriage position tracking on zeroing complete
        
        setRps(MANDREL_MOTOR_PARAMS, 0); // Stop mandrel

        FSM_SwitchState(DWELLING);
    }
}

void FSM_Moving() {
    // Move according to current layer parameters, track progress, transition to DWELLING when done

    // On the forward pass, carriage moves forward, mandrel spins
    // Define speed
    // Related carriage steps to mandrel steps for mm to deg and vice versa (StepRatio)
    // Define mm per step for carriage
    // Define deg per step for mandrel
    // When going backward, keep spinning in same direction but make angle negative?

    setRps(MANDREL_MOTOR_PARAMS, motor_speed);
}

void FSM_Dwelling() {
    // Rotate mandrel in place for the specified dwell angle, then transition to next layer or FINISHED
}

void FSM_Finished() {
    // Stop all motion, maybe signal completion with an LED or serial message
}