/// @file main.cpp
/// @brief Filament-winder firmware entry point.
///
/// Initialises hardware, then runs the winding state machine every loop().
/// A minimal serial command interface is provided for testing — replace with
/// full UI/comms integration as needed.

#include "main.h"

static bool debugMode = true;
static int debugDir = 1;

static unsigned long lastLedToggle = 0;
static bool ledState = false;

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    initSteppers();
}

void loop() {
    if (debugMode) {
        // if limit switch is triggered, reverse direction
        if (digitalRead(CARRIAGE_LIMIT_SWITCH_PIN) == LOW) {
            debugDir = -debugDir; // Reverse direction
        }
        
        // In debug mode, run motors at max speed to test wiring and basic functionality.
        runMotorsMaxSpeed(debugDir);
        
        return;
    }
    
    // blink LED
    unsigned long now = millis();
    if (now - lastLedToggle >= LED_BLINK_INTERVAL_MS) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    FSM_Run();
}
