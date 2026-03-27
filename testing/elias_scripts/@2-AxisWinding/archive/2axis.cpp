#include <Arduino.h>
#include <AccelStepper.h>

// Carriage pins (driver slot 1)
#define CARRIAGE_STEP  16
#define CARRIAGE_DIR   17
#define CARRIAGE_EN    5
#define CARRIAGE_LIMIT 35

// Toolhead pins (using mandrel driver slot, pins 18/19/21)
#define TOOLHEAD_STEP  18
#define TOOLHEAD_DIR   19
#define TOOLHEAD_EN    21
#define TOOLHEAD_LIMIT 32

// Emergency shut off pin
#define E_STOP 13

// Hardware Variables
const float Pitch = 2.0;
const int motorSteps = 200;
const int microsteps = 16;
const int motorTeeth = 20;
const int carTeeth = 20;
const int toolheadTeeth = 60;

const float stepsPerMM = (motorSteps * microsteps) / (carTeeth * Pitch);
const float toolheadStepsPerRev = motorSteps * microsteps * ((float)toolheadTeeth / motorTeeth);

// Stepper Objects
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);
AccelStepper toolhead(AccelStepper::DRIVER, TOOLHEAD_STEP, TOOLHEAD_DIR);

// Winding parameters
float layerLength = 50.0;  // mm
bool goingForward = true;

// State machine
enum State {
    ZEROING_TOOLHEAD,
    POSITIONING_TOOLHEAD,
    WINDING,
    FINISHED
};

State currentState = ZEROING_TOOLHEAD;

void setup() {
    Serial.begin(115200);

    // Motor driver pins
    pinMode(CARRIAGE_STEP, OUTPUT);
    pinMode(CARRIAGE_DIR, OUTPUT);
    pinMode(CARRIAGE_EN, OUTPUT);
    pinMode(TOOLHEAD_STEP, OUTPUT);
    pinMode(TOOLHEAD_DIR, OUTPUT);
    pinMode(TOOLHEAD_EN, OUTPUT);

    // Limit switches and E-Stop
    pinMode(CARRIAGE_LIMIT, INPUT_PULLUP);
    pinMode(TOOLHEAD_LIMIT, INPUT_PULLUP);
    pinMode(E_STOP, INPUT);

    // Enable drivers
    digitalWrite(CARRIAGE_EN, LOW);
    digitalWrite(TOOLHEAD_EN, LOW);

    // Carriage: same speeds as working 2-axis code
    carriage.setMaxSpeed(2000);
    carriage.setSpeed(1000);

    // Toolhead
    toolhead.setMaxSpeed(2000);
    toolhead.setAcceleration(3000);

    Serial.println("Starting zeroing sequence...");
}

void loop() {
    switch (currentState) {

        case ZEROING_TOOLHEAD: {
            toolhead.setSpeed(-400);
            toolhead.runSpeed();

            if (digitalRead(TOOLHEAD_LIMIT) == HIGH) {
                toolhead.setSpeed(0);
                delay(200);
                toolhead.setCurrentPosition(0);

                Serial.println("Toolhead zeroed.");
                currentState = POSITIONING_TOOLHEAD;
            }
            break;
        }

        case POSITIONING_TOOLHEAD: {
            long firstTarget = (long)((90.0 / 360.0) * toolheadStepsPerRev);
            toolhead.moveTo(firstTarget);
            toolhead.run();

            if (toolhead.distanceToGo() == 0) {
                Serial.println("Toolhead positioned at +90 deg. Starting in 2 seconds...");
                delay(2000);
                currentState = WINDING;
            }
            break;
        }

        case WINDING: {
            // Drive carriage directly at constant speed
            float direction = goingForward ? 1.0 : -1.0;
            carriage.setSpeed(1000 * direction);
            carriage.runSpeed();

            // Check end of pass
            float currentPosMM = carriage.currentPosition() / stepsPerMM;
            float target = goingForward ? layerLength : 0.0;

            bool passComplete = false;
            if (goingForward && currentPosMM >= target) passComplete = true;
            if (!goingForward && currentPosMM <= target) passComplete = true;

            if (passComplete) {
                goingForward = !goingForward;

                // Flip toolhead 180 degrees: +90 when forward, -90 when reverse
                float toolheadAngle = goingForward ? 90.0 : -90.0;
                long toolheadTarget = (long)((toolheadAngle / 360.0) * toolheadStepsPerRev);
                toolhead.moveTo(toolheadTarget);
                while (toolhead.distanceToGo() != 0) {
                    toolhead.run();
                }

                Serial.printf("Pass complete. CarPos: %.2f mm  Direction: %s\n",
                    currentPosMM, goingForward ? "FWD" : "REV");
            }
            break;
        }

        case FINISHED: {
            carriage.setSpeed(0);
            Serial.println("Winding complete.");
            while (true) { delay(1000); }
            break;
        }
    }
}
