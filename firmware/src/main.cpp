/// @file main.cpp
/// @brief Filament-winder firmware entry point.
///
/// Initialises hardware, then runs the winding state machine every loop().
/// A minimal serial command interface is provided for testing — replace with
/// full UI/comms integration as needed.

#include "main.h"

void setup() {
    Serial.begin(115200);

    initSteppers();
    Winding::init();

    Winding::start();

    Serial.println(F("=== Filament Winder Ready ==="));
    Serial.println(F("Commands: profile, start, pause, resume, status"));
}

void loop() {
    Winding::update();

    // ── Minimal serial command interface (placeholder for full UI) ───────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "start") {
            Winding::start();

        } else if (cmd == "pause") {
            Winding::pause();

        } else if (cmd == "resume") {
            Winding::resume();

        } else if (cmd == "status") {
            const char* names[] = {
                "IDLE", "PAUSED", "ZEROING", "WINDING", "DWELLING", "COMPLETE"
            };
            Serial.print(F("State: "));
            Serial.print(names[static_cast<int>(Winding::getState())]);
            Serial.print(F("  Layer: "));
            Serial.print(Winding::getActiveLayerIndex());
            Serial.print(F("/"));
            Serial.println(Winding::getProfile().layerCount);

        } else if (cmd == "profile") {
            // Load a test profile — replace with real UI data in production.
            WindProfile& p = Winding::getProfile();
            p.clear();
            p.mandrelDiameter = 50.0f;                           // 50 mm mandrel
            p.addLayer(200.0f, 45.0f, 0.0f, 4.0f, 10.0f);       // Layer 0
            Serial.println(F("Test profile loaded (50 mm dia, 1 layer @ 45 deg)."));
        }
    }
}
