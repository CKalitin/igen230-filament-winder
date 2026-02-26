/// @file winding.cpp
/// @brief Winding state-machine and wind-profile implementation.
///
/// Translates the electronic-gearing algorithm from elias_script.cpp into a
/// clean state machine that operates on a WindProfile.

#include "winding.h"
#include "config.h"
#include "motor_control.h"

// ============================================================================
//  Internal (file-scoped) State
// ============================================================================

static WindProfile  s_profile;
static WindingState s_state            = WindingState::IDLE;
static int          s_activeLayerIdx   = 0;

// Electronic-gearing runtime variables.
static float s_carAccumulator   = 0.0f;   // Fractional carriage-step accumulator.
static long  s_lastMandrelStep  = 0;       // Previous mandrel position (steps).
static long  s_dwellTargetStep  = 0;       // Mandrel step count to end dwell.

// State to resume to after un-pausing.
static WindingState s_stateBeforePause = WindingState::IDLE;

// Derived ratios (computed once in init() from motor params + drive train).
static float s_carriageStepsPerMM = 0.0f;
static float s_mandrelStepsPerRev = 0.0f;

// ============================================================================
//  WindProfile Implementation
// ============================================================================

bool WindProfile::addLayer(float length, float angle, float offset,
                           float stepover, float dwell) {
    if (layerCount >= MAX_LAYERS) return false;
    layers[layerCount] = Layer(length, angle, offset, stepover,
                               dwell, mandrelDiameter);
    layerCount++;
    return true;
}

void WindProfile::clear() {
    for (int i = 0; i < MAX_LAYERS; i++) {
        layers[i] = Layer();
    }
    layerCount      = 0;
    mandrelDiameter = 0.0f;
}

bool WindProfile::isValid() const {
    return (layerCount > 0) && (mandrelDiameter > 0.0f);
}

// ============================================================================
//  Winding Controller — Public API
// ============================================================================

void Winding::init() {
    // Compute derived ratios from actual motor configuration.
    s_carriageStepsPerMM = computeCarriageStepsPerMM(CARRIAGE_MOTOR_PARAMS.microStepsPerRev);
    s_mandrelStepsPerRev = computeMandrelStepsPerRev(MANDREL_MOTOR_PARAMS.microStepsPerRev);

    // Configure limit-switch input.
    pinMode(CARRIAGE_LIMIT_PIN, INPUT_PULLUP);

    s_state = WindingState::IDLE;
}

void Winding::start() {
    if (!s_profile.isValid()) {
        Serial.println(F("[WINDING] Cannot start — no valid profile loaded."));
        return;
    }

    // Reset runtime variables.
    s_activeLayerIdx = 0;
    s_carAccumulator = 0.0f;

    // Reset progress on every layer.
    for (int i = 0; i < s_profile.layerCount; i++) {
        s_profile.layers[i].resetProgress();
    }

    // Apply winding motion parameters.
    mandrelStepper.setMaxSpeed(DEFAULT_MANDREL_MAX_SPEED);
    mandrelStepper.setSpeed(DEFAULT_MANDREL_SPEED);
    carriageStepper.setMaxSpeed(DEFAULT_CARRIAGE_MAX_SPEED);
    carriageStepper.setAcceleration(DEFAULT_CARRIAGE_ACCEL);

    // Begin with a homing sequence.
    s_state = WindingState::ZEROING;
    Serial.println(F("[WINDING] Zeroing started..."));
}

void Winding::pause() {
    if (s_state == WindingState::ZEROING ||
        s_state == WindingState::WINDING ||
        s_state == WindingState::DWELLING) {
        s_stateBeforePause = s_state;
        s_state = WindingState::PAUSED;
        Serial.println(F("[WINDING] Paused."));
    }
}

void Winding::resume() {
    if (s_state == WindingState::PAUSED) {
        s_state = s_stateBeforePause;
        Serial.println(F("[WINDING] Resumed."));
    }
}

WindProfile& Winding::getProfile() {
    return s_profile;
}

WindingState Winding::getState() {
    return s_state;
}

int Winding::getActiveLayerIndex() {
    return s_activeLayerIdx;
}

// ============================================================================
//  Winding Controller — State Machine (called every loop())
// ============================================================================

void Winding::update() {
    switch (s_state) {

    // ── Nothing to do in these states ────────────────────────────────────────
    case WindingState::IDLE:
    case WindingState::PAUSED:
    case WindingState::COMPLETE:
        return;

    // ── ZEROING: drive carriage toward the home limit switch ─────────────────
    case WindingState::ZEROING: {
        carriageStepper.setSpeed(-ZEROING_SPEED);
        carriageStepper.runSpeed();

        if (digitalRead(CARRIAGE_LIMIT_PIN) == LOW) {
            carriageStepper.stop();
            carriageStepper.setCurrentPosition(0);
            s_lastMandrelStep = mandrelStepper.currentPosition();
            s_carAccumulator  = 0.0f;
            s_state = WindingState::WINDING;
            Serial.println(F("[WINDING] Zeroing complete. Winding layer 0..."));
        }
        break;
    }

    // ── WINDING: electronic gearing — sync carriage to mandrel ──────────────
    case WindingState::WINDING: {
        Layer& active = s_profile.layers[s_activeLayerIdx];

        const float ratio  = active.getStepRatio(s_carriageStepsPerMM, s_mandrelStepsPerRev);
        const float target = active.getTargetEndpoint();

        // 1. Spin mandrel at constant speed.
        mandrelStepper.runSpeed();

        // 2. Synchronise carriage to mandrel via fractional-step accumulator.
        long stepNow = mandrelStepper.currentPosition();

        if (stepNow != s_lastMandrelStep) {
            long delta = stepNow - s_lastMandrelStep;
            s_lastMandrelStep = stepNow;

            float sign = active.isGoingForward() ? 1.0f : -1.0f;
            s_carAccumulator += delta * ratio * sign;

            if (fabsf(s_carAccumulator) >= 1.0f) {
                long steps = static_cast<long>(s_carAccumulator);
                carriageStepper.move(steps);
                s_carAccumulator -= steps;
            }
        }

        carriageStepper.run();

        // 3. Detect end of pass.
        float posMM = carriageStepper.currentPosition() / s_carriageStepsPerMM;

        bool reached = active.isGoingForward()
                     ? (posMM >= target)
                     : (posMM <= target);

        if (reached) {
            // Compute dwell: fibre-placement rotation + stepover shift.
            float totalDeg    = active.getDwell() + active.getStepoverDegrees();
            long  dwellSteps  = static_cast<long>(
                (totalDeg / 360.0f) * s_mandrelStepsPerRev);
            s_dwellTargetStep = mandrelStepper.currentPosition() + dwellSteps;

            s_state = WindingState::DWELLING;
        }
        break;
    }

    // ── DWELLING: extra mandrel rotation while carriage is stationary ────────
    case WindingState::DWELLING: {
        mandrelStepper.runSpeed();

        if (mandrelStepper.currentPosition() >= s_dwellTargetStep) {
            Layer& active = s_profile.layers[s_activeLayerIdx];
            active.countPass();

            if (active.isDone()) {
                // Try to advance to the next layer.
                if (s_activeLayerIdx < s_profile.layerCount - 1) {
                    s_activeLayerIdx++;
                    s_carAccumulator  = 0.0f;
                    s_lastMandrelStep = mandrelStepper.currentPosition();
                    s_state = WindingState::WINDING;

                    Serial.print(F("[WINDING] Layer "));
                    Serial.print(s_activeLayerIdx);
                    Serial.println(F(" started."));
                } else {
                    // All layers complete — stop motors.
                    mandrelStepper.setSpeed(0);
                    carriageStepper.setSpeed(0);
                    s_state = WindingState::COMPLETE;
                    Serial.println(F("[WINDING] All layers complete."));
                }
            } else {
                // Continue with the next pass of the current layer.
                s_lastMandrelStep = mandrelStepper.currentPosition();
                s_state = WindingState::WINDING;
            }
        }
        break;
    }

    }  // switch
}
