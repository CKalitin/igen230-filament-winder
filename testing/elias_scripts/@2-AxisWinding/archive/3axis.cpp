#include <Arduino.h>
#include <AccelStepper.h>

// Mandrel pins
#define MANDREL_DIR    4
#define MANDREL_STEP   23
#define MANDREL_EN     22

// Carriage pins
#define CARRIAGE_DIR   17
#define CARRIAGE_STEP  16
#define CARRIAGE_EN    5
#define CARRIAGE_LIMIT 35

// Toolhead pins
#define TOOLHEAD_DIR   19
#define TOOLHEAD_STEP  18
#define TOOLHEAD_EN    21
#define TOOLHEAD_LIMIT 32

// Toolarm pins
#define TOOLARM_DIR    25
#define TOOLARM_STEP   26
#define TOOLARM_EN     27
#define TOOLARM_LIMIT  2

// Emergency shut off pin
#define E_STOP 15

// Hardware Variables
const float Pitch = 2.0;
const int motorSteps = 200;
const int microsteps = 16;
const int motorTeeth = 20;
const int carTeeth = 20;
const int manTeeth = 40;
const int toolheadTeeth = 60;

const float stepsPerMM  = (motorSteps * microsteps) / (carTeeth * Pitch);
const float stepsPerRev = (motorSteps * microsteps) * ((float)manTeeth / motorTeeth);
const float toolheadStepsPerRev = motorSteps * microsteps * ((float)toolheadTeeth / motorTeeth);

// Stepper Objects
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);
AccelStepper toolhead(AccelStepper::DRIVER, TOOLHEAD_STEP, TOOLHEAD_DIR);

// Gearing variables
float carAccumulator = 0;
long lastManStep = 0;

// Test layer parameters
float manD = 55.0;
float windAngle = 45.0;
float layerLength = 50.0;
bool goingForward = true;

// State machine
enum State {
    ZEROING_CARRIAGE,
    ZEROING_TOOLHEAD,
    POSITIONING_TOOLHEAD,
    WINDING,
    FINISHED
};

State currentState = ZEROING_TOOLHEAD;

float getStepRatio() {
    float safeAngle = windAngle;
    if (safeAngle > 89.0) safeAngle = 89.9;
    if (safeAngle < 1.0) safeAngle = 1.0;
    float angleRad = radians(safeAngle);
    float mmPerRev = PI * manD / tan(angleRad);
    return (mmPerRev * stepsPerMM) / stepsPerRev;
}

void setup() {
    Serial.begin(115200);

    // Motor driver pins
    pinMode(MANDREL_STEP, OUTPUT);
    pinMode(MANDREL_DIR, OUTPUT);
    pinMode(MANDREL_EN, OUTPUT);
    pinMode(CARRIAGE_STEP, OUTPUT);
    pinMode(CARRIAGE_DIR, OUTPUT);
    pinMode(CARRIAGE_EN, OUTPUT);
    pinMode(TOOLHEAD_STEP, OUTPUT);
    pinMode(TOOLHEAD_DIR, OUTPUT);
    pinMode(TOOLHEAD_EN, OUTPUT);
    pinMode(TOOLARM_STEP, OUTPUT);
    pinMode(TOOLARM_DIR, OUTPUT);
    pinMode(TOOLARM_EN, OUTPUT);

    // Limit switches and E-Stop
    pinMode(CARRIAGE_LIMIT, INPUT_PULLUP);
    pinMode(TOOLHEAD_LIMIT, INPUT_PULLUP);
    pinMode(TOOLARM_LIMIT, INPUT_PULLUP);
    pinMode(E_STOP, INPUT);

    // Enable all drivers
    digitalWrite(MANDREL_EN, LOW);
    digitalWrite(CARRIAGE_EN, LOW);
    digitalWrite(TOOLHEAD_EN, LOW);
    digitalWrite(TOOLARM_EN, LOW);

    // Mandrel: 2.5x speed (was 1000, now 2500)
    mandrel.setMaxSpeed(5000);
    mandrel.setSpeed(2500);

    // Carriage: raised max to keep up with faster mandrel
    carriage.setMaxSpeed(5000);
    carriage.setAcceleration(5000);

    // Toolhead
    toolhead.setMaxSpeed(2000);
    toolhead.setAcceleration(3000);

    Serial.println("Starting zeroing sequence...");
}

void loop() {
    switch (currentState) {

        case ZEROING_CARRIAGE: {
            carriage.setSpeed(-600);
            carriage.runSpeed();

            if (digitalRead(CARRIAGE_LIMIT) == LOW) {
                carriage.setSpeed(0);
                delay(200);

                // Back off the switch
                carriage.setCurrentPosition(0);
                carriage.moveTo(200);
                while (carriage.distanceToGo() != 0) carriage.run();
                carriage.setCurrentPosition(0);

                Serial.println("Carriage zeroed.");
                currentState = ZEROING_TOOLHEAD;
            }
            break;
        }

        case ZEROING_TOOLHEAD: {
            // Slowly rotate toolhead toward limit switch
            toolhead.setSpeed(-400);
            toolhead.runSpeed();

            if (digitalRead(TOOLHEAD_LIMIT) == LOW) {
                toolhead.setSpeed(0);
                delay(200);

                // Set this as the toolhead zero reference
                toolhead.setCurrentPosition(0);

                Serial.println("Toolhead zeroed.");
                currentState = POSITIONING_TOOLHEAD;
            }
            break;
        }

        case POSITIONING_TOOLHEAD: {
            // Move toolhead to the starting winding angle
            long firstTarget = (long)((windAngle / 360.0) * toolheadStepsPerRev);
            toolhead.moveTo(firstTarget);
            toolhead.run();

            if (toolhead.distanceToGo() == 0) {
                lastManStep = mandrel.currentPosition();
                Serial.println("Toolhead positioned. Starting winding in 2 seconds...");
                Serial.printf("Step ratio: %.4f\n", getStepRatio());
                Serial.printf("StepsPerMM: %.2f  StepsPerRev: %.2f\n", stepsPerMM, stepsPerRev);
                Serial.printf("Toolhead target: %ld steps\n", firstTarget);
                delay(2000);
                currentState = WINDING;
            }
            break;
        }

        case WINDING: {
            float ratio = getStepRatio();

            // Step the mandrel at constant speed
            mandrel.runSpeed();

            // Electronic gearing: sync carriage to mandrel
            long stepNow = mandrel.currentPosition();

            if (stepNow != lastManStep) {
                long delta = stepNow - lastManStep;
                lastManStep = stepNow;

                float moveSign = goingForward ? 1.0 : -1.0;
                carAccumulator += (delta * ratio * moveSign);

                if (abs(carAccumulator) >= 1.0) {
                    long numStep = (long)carAccumulator;
                    carriage.move(numStep);
                    carAccumulator -= numStep;
                }
            }

            carriage.run();

            // Check end of pass
            float currentPosMM = carriage.currentPosition() / stepsPerMM;
            float target = goingForward ? layerLength : 0.0;

            bool passComplete = false;
            if (goingForward && currentPosMM >= target) passComplete = true;
            if (!goingForward && currentPosMM <= target) passComplete = true;

            if (passComplete) {
                goingForward = !goingForward;

                // Flip toolhead to new angle
                float toolheadAngle = goingForward ? windAngle : -windAngle;
                long toolheadTarget = (long)((toolheadAngle / 360.0) * toolheadStepsPerRev);
                toolhead.moveTo(toolheadTarget);
                while (toolhead.distanceToGo() != 0) {
                    toolhead.run();
                    mandrel.runSpeed();  // Keep mandrel spinning during flip
                }

                lastManStep = mandrel.currentPosition();
                Serial.printf("Pass complete. CarPos: %.2f mm  Direction: %s\n",
                    currentPosMM, goingForward ? "FWD" : "REV");
            }

            // Debug output every 5000 mandrel steps
            static long lastDebug = 0;
            if (stepNow - lastDebug > 5000) {
                lastDebug = stepNow;
                Serial.printf("manPos: %ld  carPos: %ld  carMM: %.2f  dir: %s\n",
                    stepNow, carriage.currentPosition(), currentPosMM,
                    goingForward ? "FWD" : "REV");
            }
            break;
        }

        case FINISHED: {
            mandrel.setSpeed(0);
            carriage.setSpeed(0);
            Serial.println("Winding complete.");
            while (true) { delay(1000); }
            break;
        }
    }
}
