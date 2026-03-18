#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduinoJson.h>

// Mandrel pins
#define MANDREL_DIR    12
#define MANDREL_STEP   13
#define MANDREL_EN     14

// Carriage pins
#define CARRIAGE_DIR   15
#define CARRIAGE_STEP  16
#define CARRIAGE_EN    17
#define CARRIAGE_LIMIT 18

// Toolhead pins
#define TOOLHEAD_DIR   21
#define TOOLHEAD_STEP  22
#define TOOLHEAD_EN    23
#define TOOLHEAD_LIMIT 19

// Toolarm pins
#define TOOLARM_DIR    25
#define TOOLARM_STEP   26
#define TOOLARM_EN     27
#define TOOLARM_LIMIT  32

// Emergency stop pin
#define E_STOP 34

// ─── Spline Profile ───────────────────────────────────────────────────────────

class SplineProfile {
    private:
        static const int maxPoints = 50;
        float _x[maxPoints];
        float _y[maxPoints];
        float _a[maxPoints];
        float _b[maxPoints];
        float _c[maxPoints];
        float _d[maxPoints];
        int   _n;
        float _standoff;

    public:
        SplineProfile() : _n(0), _standoff(0) {}

        void reset() { _n = 0; _standoff = 0; }

        void setStandoff(float standoff) { _standoff = standoff; }

        void addPoint(float x, float y) {
            if (_n < maxPoints) {
                _x[_n] = x;
                _y[_n] = y;
                _n++;
            }
        }

        void compute() {
            if (_n < 2) return;
            int n = _n - 1;

            float h[maxPoints], alpha[maxPoints];
            float l[maxPoints], mu[maxPoints], z[maxPoints];

            for (int i = 0; i < n; i++)
                h[i] = _x[i+1] - _x[i];

            for (int i = 1; i < n; i++)
                alpha[i] = (3.0/h[i])*(_y[i+1]-_y[i]) - (3.0/h[i-1])*(_y[i]-_y[i-1]);

            l[0] = 1; mu[0] = 0; z[0] = 0;

            for (int i = 1; i < n; i++) {
                l[i]  = 2*(_x[i+1]-_x[i-1]) - h[i-1]*mu[i-1];
                mu[i] = h[i] / l[i];
                z[i]  = (alpha[i] - h[i-1]*z[i-1]) / l[i];
            }

            l[n] = 1; z[n] = 0; _c[n] = 0;

            for (int j = n-1; j >= 0; j--) {
                _c[j] = z[j] - mu[j]*_c[j+1];
                _b[j] = (_y[j+1]-_y[j])/h[j] - h[j]*(_c[j+1]+2*_c[j])/3.0;
                _d[j] = (_c[j+1]-_c[j]) / (3.0*h[j]);
                _a[j] = _y[j];
            }
        }

        float getTarget(float x) const {
            if (_n < 2) return _standoff;
            if (x <= _x[0])    return _y[0]    + _standoff;
            if (x >= _x[_n-1]) return _y[_n-1] + _standoff;

            int i = 0;
            for (int j = 0; j < _n-1; j++) {
                if (x >= _x[j] && x <= _x[j+1]) { i = j; break; }
            }

            float dx = x - _x[i];
            return _a[i] + _b[i]*dx + _c[i]*dx*dx + _d[i]*dx*dx*dx + _standoff;
        }

        bool isReady() const { return _n >= 2; }
};

// ─── Layer ────────────────────────────────────────────────────────────────────

class Layer {
    private:
        float _length;
        float _angle;
        float _offset;
        float _stepover;
        float _dwell;
        float _diameter;
        int   _pass;
        int   _passDone;
        bool  _goForward;

    public:
        Layer(float l, float a, float off, float stpo, float d, float manD)
            : _length(l), _angle(a), _offset(off), _stepover(stpo), _dwell(d),
              _diameter(manD), _passDone(0), _goForward(true) {
                float safeAngle = constrain(_angle, 1.0, 89.9);
                float angleRad  = radians(safeAngle);
                float circ      = PI * manD;
                float calcPasses = (circ * cos(angleRad)) / _stepover;
                _pass = (int)ceil(calcPasses);
                if (_pass % 2 != 0) _pass++;
              }

        float getLength() const { return _length; }
        float getAngle()  const { return _angle; }
        float getDwell()  const { return _dwell; }

        float getStepRatio(float manD, float stepsPerMM, float stepsPerRev) const {
            float safeAngle = constrain(_angle, 1.0, 89.9);
            float angleRad  = radians(safeAngle);
            float mmPerRev  = PI * manD / tan(angleRad);
            return (mmPerRev * stepsPerMM) / stepsPerRev;
        }

        float getStepoverDeg() const {
            float angleRad = radians(_angle);
            float circ     = PI * _diameter;
            return (_stepover * 360.0) / (circ * cos(angleRad));
        }

        float getTargetEndpoint() const {
            return _goForward ? _offset + _length : _offset;
        }

        long getToolheadTarget(bool goingForward, float stepsPerRev) const {
            float toolheadAngle = goingForward ? _angle : -_angle;
            return (long)((toolheadAngle / 360.0) * stepsPerRev);
        }

        bool isGoingForward() const { return _goForward; }

        void countPass() {
            _passDone++;
            _goForward = !_goForward;
        }

        bool isDone() const { return _passDone >= _pass; }
};

// ─── Globals ──────────────────────────────────────────────────────────────────

const int maxLayers = 10;
int totalLayers     = 0;
int activeLayerIndex = 0;
Layer* layers[maxLayers];

SplineProfile toolarmProfile;

// Hardware constants
const float Pitch       = 2.0;
const int motorSteps    = 200;
const int microsteps    = 16;
const int motorTeeth    = 20;
const int carTeeth      = 20;
const int manTeeth      = 40;
const int toolheadTeeth = 60;
const int toolarmPitch  = 4;

const float stepsPerMM         = (float)(motorSteps * microsteps) / (carTeeth * Pitch);
const float stepsPerRev        = (float)(motorSteps * microsteps) * ((float)manTeeth / motorTeeth);
const float toolheadStepsPerRev = (float)(motorSteps * microsteps) * ((float)toolheadTeeth / motorTeeth);
const float toolarmStepsPerMM  = (float)(motorSteps * microsteps) / toolarmPitch;

float manD = 0.0;

// Control variables
float carAccumulator  = 0;
long  lastManStep     = 0;
long  dwellTargetStep = 0;
bool  toolheadFlipped = false;
bool  carriageZeroed  = false;
bool  toolheadZeroed  = false;
bool  toolarmZeroed   = false;
bool  windingDone     = false;

// Stepper objects
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);
AccelStepper toolhead(AccelStepper::DRIVER, TOOLHEAD_STEP, TOOLHEAD_DIR);
AccelStepper toolarm(AccelStepper::DRIVER, TOOLARM_STEP, TOOLARM_DIR);

// ─── State Machine ────────────────────────────────────────────────────────────

enum windingState {
    WAITING,    // Waiting for JSON job from UI
    PAUSED,     // E-Stop or pause — all motion stopped
    ZEROING,    // Homing all axes
    MOVING,     // Active winding pass
    DWELLING,   // End-of-pass mandrel rotation + toolhead flip
    FINISHED    // Layer or job complete
};

windingState currentState  = WAITING;
windingState previousState = WAITING;

// ─── JSON Parser ──────────────────────────────────────────────────────────────

void clearJob() {
    for (int i = 0; i < totalLayers; i++) {
        delete layers[i];
        layers[i] = nullptr;
    }
    totalLayers      = 0;
    activeLayerIndex = 0;
    carAccumulator   = 0;
    toolheadFlipped  = false;
    carriageZeroed   = false;
    toolheadZeroed   = false;
    toolarmZeroed    = false;
    windingDone      = false;
    toolarmProfile.reset();
}

bool parseJob(const String& json) {
    // Allocate JSON document — 4KB should handle most jobs
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err) {
        Serial.print("JSON parse error: ");
        Serial.println(err.c_str());
        return false;
    }

    // Validate required fields
    if (!doc["mandrel_diameter"] || !doc["layers"] || !doc["profile"]) {
        Serial.println("ERROR: Missing required fields (mandrel_diameter, layers, profile)");
        return false;
    }

    clearJob();

    // Mandrel diameter
    manD = doc["mandrel_diameter"].as<float>();

    // Standoff (optional, defaults to 0)
    float standoff = doc["standoff"] | 0.0f;
    toolarmProfile.setStandoff(standoff);

    // Profile points
    JsonArray profile = doc["profile"].as<JsonArray>();
    for (JsonObject point : profile) {
        float x = point["x"].as<float>();
        float r = point["r"].as<float>();
        toolarmProfile.addPoint(x, r);
    }
    toolarmProfile.compute();

    // Layers
    JsonArray jsonLayers = doc["layers"].as<JsonArray>();
    for (JsonObject l : jsonLayers) {
        if (totalLayers >= maxLayers) {
            Serial.println("WARNING: Max layers reached, ignoring remaining layers");
            break;
        }
        float length   = l["length"].as<float>();
        float angle    = l["angle"].as<float>();
        float offset   = l["offset"]   | 0.0f;
        float stepover = l["stepover"].as<float>();
        float dwell    = l["dwell"].as<float>();

        layers[totalLayers] = new Layer(length, angle, offset, stepover, dwell, manD);
        totalLayers++;
    }

    if (totalLayers == 0) {
        Serial.println("ERROR: No valid layers in job");
        return false;
    }

    return true;
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    pinMode(MANDREL_STEP,  OUTPUT);
    pinMode(MANDREL_DIR,   OUTPUT);
    pinMode(MANDREL_EN,    OUTPUT);
    pinMode(CARRIAGE_STEP, OUTPUT);
    pinMode(CARRIAGE_DIR,  OUTPUT);
    pinMode(CARRIAGE_EN,   OUTPUT);
    pinMode(TOOLHEAD_STEP, OUTPUT);
    pinMode(TOOLHEAD_DIR,  OUTPUT);
    pinMode(TOOLHEAD_EN,   OUTPUT);
    pinMode(TOOLARM_STEP,  OUTPUT);
    pinMode(TOOLARM_DIR,   OUTPUT);
    pinMode(TOOLARM_EN,    OUTPUT);

    pinMode(CARRIAGE_LIMIT, INPUT_PULLUP);
    pinMode(TOOLHEAD_LIMIT, INPUT_PULLUP);
    pinMode(TOOLARM_LIMIT,  INPUT_PULLUP);
    pinMode(E_STOP,         INPUT);

    digitalWrite(MANDREL_EN,  LOW);
    digitalWrite(CARRIAGE_EN, LOW);
    digitalWrite(TOOLHEAD_EN, LOW);
    digitalWrite(TOOLARM_EN,  LOW);

    mandrel.setMaxSpeed(2000);
    mandrel.setSpeed(1000);
    carriage.setMaxSpeed(2000);
    carriage.setAcceleration(5000);
    toolhead.setMaxSpeed(2000);
    toolhead.setAcceleration(3000);
    toolarm.setMaxSpeed(3000);
    toolarm.setAcceleration(4000);

    Serial.println("READY");  // Signal to UI that ESP32 is ready for a job
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {

    // ── E-Stop ──────────────────────────────────────────────────────────────
    if (currentState != WAITING) {
        if (digitalRead(E_STOP) == HIGH) {
            if (currentState != PAUSED) {
                previousState = currentState;
                currentState  = PAUSED;
            }
        }
        else {
            if (currentState == PAUSED) {
                currentState = previousState;
            }
        }
    }

    // ── State Machine ────────────────────────────────────────────────────────
    switch (currentState) {

        case WAITING: {
            // Block and read serial until a full JSON object is received
            if (Serial.available()) {
                String json = Serial.readStringUntil('\n');
                json.trim();

                if (json.length() == 0) break;

                if (parseJob(json)) {
                    Serial.println("OK");   // Confirm receipt to UI
                    currentState = ZEROING;
                    Serial.println("STATE: ZEROING");
                }
                else {
                    Serial.println("ERROR");  // Tell UI parsing failed
                }
            }
            break;
        }

        case PAUSED: {
            mandrel.setSpeed(0);
            mandrel.runSpeed();
            carriage.stop();
            carriage.run();
            toolarm.stop();
            toolarm.run();
            break;
        }

        case ZEROING: {
            previousState = currentState;

            if (!carriageZeroed) {
                carriage.setSpeed(-600);
                carriage.runSpeed();
                if (digitalRead(CARRIAGE_LIMIT) == LOW) {
                    carriage.stop();
                    carriage.setCurrentPosition(0);
                    carriage.moveTo(200);
                    while (carriage.distanceToGo() != 0) carriage.run();
                    carriage.setCurrentPosition(0);
                    carriageZeroed = true;
                }
            }
            else if (!toolheadZeroed) {
                toolhead.setSpeed(-400);
                toolhead.runSpeed();
                if (digitalRead(TOOLHEAD_LIMIT) == LOW) {
                    toolhead.stop();
                    toolhead.setCurrentPosition(0);
                    toolheadZeroed = true;
                }
            }
            else if (!toolarmZeroed) {
                toolarm.setSpeed(-400);
                toolarm.runSpeed();
                if (digitalRead(TOOLARM_LIMIT) == LOW) {
                    toolarm.stop();
                    toolarm.setCurrentPosition(0);
                    toolarmZeroed = true;
                }
            }
            else {
                // Move toolhead to first position
                Layer* firstLayer = layers[0];
                long firstTarget  = firstLayer->getToolheadTarget(true, toolheadStepsPerRev);
                toolhead.moveTo(firstTarget);
                toolhead.run();

                if (toolhead.distanceToGo() == 0) {
                    lastManStep  = mandrel.currentPosition();
                    currentState = MOVING;
                    Serial.println("STATE: MOVING");
                }
            }
            break;
        }

        case MOVING: {
            previousState = currentState;

            Layer* activeLayer = layers[activeLayerIndex];
            float ratio  = activeLayer->getStepRatio(manD, stepsPerMM, stepsPerRev);
            float target = activeLayer->getTargetEndpoint();

            // Electronic gearing
            mandrel.runSpeed();
            long stepNow = mandrel.currentPosition();

            if (stepNow != lastManStep) {
                long delta    = stepNow - lastManStep;
                lastManStep   = stepNow;
                float moveSign = activeLayer->isGoingForward() ? 1.0 : -1.0;
                carAccumulator += delta * ratio * moveSign;

                if (abs(carAccumulator) >= 1.0) {
                    long numStep    = (long)carAccumulator;
                    carriage.move(numStep);
                    carAccumulator -= numStep;
                }
            }
            carriage.run();

            // Toolarm tracking
            float currentPosMM = carriage.currentPosition() / stepsPerMM;
            if (toolarmProfile.isReady()) {
                float targetMM    = toolarmProfile.getTarget(currentPosMM);
                long  targetSteps = (long)(targetMM * toolarmStepsPerMM);
                toolarm.moveTo(targetSteps);
            }
            toolarm.run();

            // End of pass check
            if (activeLayer->isGoingForward()  && currentPosMM >= target) currentState = DWELLING;
            if (!activeLayer->isGoingForward() && currentPosMM <= target) currentState = DWELLING;

            if (currentState == DWELLING) {
                float totalDeg    = activeLayer->getDwell() + activeLayer->getStepoverDeg();
                long stepsToDwell = (long)((totalDeg / 360.0) * stepsPerRev);
                dwellTargetStep   = mandrel.currentPosition() + stepsToDwell;
                toolheadFlipped   = false;
            }
            break;
        }

        case DWELLING: {
            previousState = currentState;

            Layer* activeLayer = layers[activeLayerIndex];
            mandrel.runSpeed();

            if (!toolheadFlipped) {
                bool nextDirection  = !activeLayer->isGoingForward();
                long toolheadTarget = activeLayer->getToolheadTarget(nextDirection, toolheadStepsPerRev);
                toolhead.moveTo(toolheadTarget);
                toolhead.run();
                if (toolhead.distanceToGo() == 0) toolheadFlipped = true;
            }

            if (toolheadFlipped && mandrel.currentPosition() >= dwellTargetStep) {
                activeLayer->countPass();

                if (activeLayer->isDone()) {
                    currentState = FINISHED;
                    Serial.printf("STATE: LAYER %d COMPLETE\n", activeLayerIndex);
                }
                else {
                    lastManStep  = mandrel.currentPosition();
                    currentState = MOVING;
                }
            }
            break;
        }

        case FINISHED: {
            previousState = currentState;

            if (activeLayerIndex < totalLayers - 1) {
                activeLayerIndex++;
                carAccumulator  = 0;
                lastManStep     = mandrel.currentPosition();
                toolheadFlipped = false;
                currentState    = MOVING;
            }
            else {
                if (!windingDone) {
                    mandrel.setSpeed(0);
                    carriage.setSpeed(0);
                    windingDone  = true;
                    currentState = WAITING;   // Return to waiting for next job
                    Serial.println("STATE: COMPLETE");
                    Serial.println("READY");  // Signal UI that a new job can be sent
                }
            }
            break;
        }
    }
}