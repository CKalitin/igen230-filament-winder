#include <Arduino.h>
#include <AccelStepper.h> // this is the library that allows arduino ide to talk to motor drivers
#include <ArduinoJson.h>

// Carriage pins (Motor 1 on PCB)
#define CARRIAGE_DIR   17
#define CARRIAGE_STEP  16
#define CARRIAGE_EN    5
#define CARRIAGE_LIMIT 35   // Limit switch 1 (on PCB)

// Mandrel pins (Motor 2 on PCB)
#define MANDREL_DIR    19   
#define MANDREL_STEP   18
#define MANDREL_EN     21

// Toolhead pins (Motor 3 on PCB)
#define TOOLHEAD_DIR   14
#define TOOLHEAD_STEP  12
#define TOOLHEAD_EN    27

// Toolarm pins (Motor 4 on PCB)
#define TOOLARM_DIR    25 
#define TOOLARM_STEP   26
#define TOOLARM_EN     33
#define TOOLARM_LIMIT  32   // Limit switch 2 (on PCB)

// Emergency stop pin
#define E_STOP         13

class SplineProfile {
    private:
        static const int maxPoints = 50;  // Maximum number of profile points
        float _standoff;      // Fixed standoff distance from surface (mm)
        float _x[maxPoints];  // Carriage positions (mm)
        float _y[maxPoints];  // Radius values (mm)
        float _a[maxPoints];  // Spline coefficients
        float _b[maxPoints];  // 
        float _c[maxPoints];  //
        float _d[maxPoints];  //
        int   _n;               // Total number of points

    public:
        SplineProfile() : _n(0), _standoff(20) {}

        // Set standoff distance
        void setStandoff(float standoff) { _standoff = standoff; }

        // Add a (position, radius) point — call these in order of increasing x
        void addPoint(float x, float y) {
            if (_n < maxPoints) {
                _x[_n] = x;
                _y[_n] = y;
                _n++;
            }
        }

        // Compute spline coefficients — call once after all points are added
        void compute() {
            if (_n < 2) return;
            int n = _n - 1;  // Number of segments

            float h[maxPoints];
            float alpha[maxPoints];
            float l[maxPoints];
            float mu[maxPoints];
            float z[maxPoints];

            for (int i = 0; i < n; i++)
                h[i] = _x[i + 1] - _x[i];

            for (int i = 1; i < n; i++)
                alpha[i] = (3.0 / h[i]) * (_y[i + 1] - _y[i]) - (3.0 / h[i - 1]) * (_y[i] - _y[i - 1]);

            l[0] = 1; mu[0] = 0; z[0] = 0;

            for (int i = 1; i < n; i++) {
                l[i] = 2 * (_x[i + 1] - _x[i - 1]) - h[i - 1] * mu[i - 1];
                mu[i] = h[i] / l[i];
                z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
            }

            l[n] = 1; z[n] = 0; _c[n] = 0;

            for (int j = n - 1; j >= 0; j--) {
                _c[j] = z[j] - mu[j] * _c[j + 1];
                _b[j] = (_y[j + 1] - _y[j]) / h[j] - h[j] * (_c[j + 1] + 2 *_c[j]) / 3.0;
                _d[j] = (_c[j + 1] - _c[j]) / (3.0 * h[j]);
                _a[j] = _y[j];
            }
        }

        // Query the target toolarm position (radius + standoff) at a given carriage position
        float getToolarmTarget(float x) const {
            if (_n < 2) return _standoff;

            // Clamp to profile range
            if (x <= _x[0])      return _y[0] + _standoff;
            if (x >= _x[_n - 1]) return _y[_n - 1] + _standoff;

            // Find the segment
            int i = 0;
            for (int j = 0; j < _n - 1; j++) {
                if (x >= _x[j] && x <= _x[j + 1]) { i = j; break; }
            }

            float dx = x - _x[i];
            float radius = _a[i] + (_b[i] * dx) + (_c[i] * dx * dx) + (_d[i] * dx * dx * dx); // Calculate the radius between known points
            if (radius < 0.0) radius = 0.0;  // prevent NaN from downstream sqrt operations
            return radius + _standoff;  // Return total required distance from mandrel surface (mm)
        }

        bool isReady() const { return _n >= 2; }
};

class Layer{
    // Information not accessible outside the Layer class
    private:
        float _diameter;  // Mandrel diameter (mm)
        float _stepover;  // Distance moved per pass (to control fibre overlap in each layer) (mm)
        int   _length;    // Length of the layer (mm)
        int   _angle;     // Fibre angle for the layer (degrees)
        int   _offset;    // Where the first pass starts on mandrel (mm)
        int   _dwell;     // Extra mandrel rotation at the end of each pass (degrees)
        int   _pass;      // Number of passes to complete (a pass is from left to right, not there and back)
        int   _passDone;  // Number of passes Completed
        bool  _goForward; // Track direction of motion

    // Information accesible to the greater program
    public:
        // Tansfer repective parameters into private storage
        Layer(int l, int a, int off, float stpo, int d, float manD)
            : _length(l), _angle(a), _offset(off), _stepover(stpo), _dwell(d), _diameter(manD), _passDone(0), _goForward(true) { 
                int safeAngle = _angle;
                if (safeAngle > 89.0) safeAngle = 89.9;    // Prevent divide by zero
                if (safeAngle < 1.0) safeAngle = 1.0;      // Prevent divide by zero
                float angleRad = radians(safeAngle);       // Convert to radians for calculations

                float circ = PI * manD; // Calculate cicumference
                float calcPasses = (circ * cos(angleRad)) / _stepover;  // Calculate passes needed for 100% coverage
                _pass = ((int)ceil(calcPasses) * 2);
            }

        // GETTERS: Read-only access
        float getLength() const { return _length; }
        float getAngle()  const { return _angle; }
        float getOffset() const { return _offset; }
        float getDwell()  const { return _dwell; }
        int getPassDone() const { return _passDone; }

        // MATH: Calculates step Ratio for the layer
        float getStepRatio(float manD, float stepsPerMM, float stepsPerRev) const {
            float safeAngle = _angle;
            if (safeAngle > 89.0) safeAngle = 89.9;    // Prevent divide by zero
            if (safeAngle < 1.0) safeAngle = 1.0;      // Prevent divide by zero

            float angleRad = radians(safeAngle);            // Convert to radians for calculations
            float mmPerRev = PI * manD / tan(angleRad);     // mm carriage needs to move per mandrel rotation for a given winding angle
            return (mmPerRev * stepsPerMM) / stepsPerRev;   // Calculate required carriage steps per mandrel step
        }

        // Calculates the extra rotation (in degrees) needed to shift the fiber by a given stepover distance
        float getStepoverDeg() const {
            float angleRad = radians(_angle);
            float circ = PI * _diameter;
            return (_stepover * 360.0) / (circ * cos(angleRad)); // Shift needed to move the mandrel by the stepover distance
        }

        // Tells the loop which coordinate (in mm) to stop at
        float getTargetEndpoint() const {
            if (_goForward) return _offset + _length;
            else return _offset;
        }

         // Returns the toolhead position in steps for a given direction
        // Forward pass: fiber feeds at (90 - angle), reverse pass: fiber feeds at -(90 - angle)
        long getToolheadTarget(bool goingForward, float stepsPerRev) const {
            float toolheadAngle = (90.0 - _angle);                          // Toolhead angle based on winding angle
            toolheadAngle = goingForward ? -toolheadAngle : toolheadAngle;  // Flip sign on direction change
            return (long)((toolheadAngle / 360.0) * stepsPerRev);
        }

        // Lets the loop know which way the carriage is moving
        bool isGoingForward() const { return _goForward; }

        // PROGRESS: Logic for the loop to interact with  
        void countPass() {
            _passDone++;              // Track one completed pass along the mandrel
            _goForward = !_goForward; // Flips direction for the return stroke
        }                
        
         // Returns true when layer is complete
        bool isDone() const { return _passDone >= _pass; } 
};

SplineProfile toolarmProfile;  // Global profile instance

// Array of pointers to store the data for each layer from the UI
const int maxLayers  = 5;   // Maximum layers the machine can handle (subject to change)
int totalLayers      = 0;   // Number of layers recieved from the UI
int activeLayerIndex = 0;   // Layer currently winding
Layer* layers[maxLayers];   // An array of pointers to Layer objects

// Possible Winding/Operation States
enum windingState {
    PAUSED,     // Stops all motion
    ZEROING,    // Zeroing the carriage before winding
    MOVING,     // Carriage and mandrel motion during winding
    DWELLING,   // Carriage stopped, mandrel rotating during dwell period
    FINISHED    // Carriage and mandrel stopped
};

windingState currentState  = PAUSED;    // Paused on statup, no motion
windingState previousState = PAUSED;    // Tracks last active state in case of E-Stop or pause

// Harware Constants
const float Pitch       = 2.0;  // Belt pitch (in mm)
const int motorSteps    = 200;  // Number of steps motor makes per revolution
const int microsteps    = 16;   // Not Sure about this, ask Loki
const int motorTeeth    = 20;   // Number of pulley teeth on motor pulleys
const int carTeeth      = 20;   // Number of pulley teeth on carriage pulley
const int manTeeth      = 20;   // Number of pulley teeth on mandrel pulley
const int toolheadTeeth = 60;   // Number of pulley teeth on toolhead pulley
const int toolarmPitch  = 3;    // mm per revolution
const int toolarmZero   = 115;  // Physical distance from mandrel axis to the toolarm at position 0 (mm)

const float stepsPerMM          = (motorSteps * microsteps) / (carTeeth * Pitch);                   // Carriage steps per mm moved
const float stepsPerRev         = (motorSteps * microsteps) * ((float)manTeeth / motorTeeth);       // Required carriage steps per mandrel step
const float toolheadStepsPerRev = (motorSteps * microsteps) * ((float)toolheadTeeth / motorTeeth);  // Steps for one full toolhead rotation
const float toolarmStepsPerMM   = (motorSteps * microsteps / 2) / toolarmPitch;                         // Toolarm steps per mm

// Winding Parameters (from UI)
float manD;   // Mandrel Diameter (mm)

// Global Control Variables
long lastManStep;              // Stores previous loop's mandrel position
long dwellTargetStep;          // Number of extra steps mandrel must move at end of a pass
bool toolheadFlipped = false;  // Tracks whether toolhead has finished flipping
bool carriageZeroed  = false;  // Tracks if the carriage has zeroed
bool toolheadZeroed  = false;  // Tracks if the toolhead has zeroed
bool toolarmZeroed   = false;  // Tracks if the toolarm has zeroed

// Stepper Objects
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);     // Creates object called mandrel to store current step position, target position, speed, timing
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);  // Same as above but for the carriage
AccelStepper toolhead(AccelStepper::DRIVER, TOOLHEAD_STEP, TOOLHEAD_DIR);  // Same but for the toolhead
AccelStepper toolarm(AccelStepper::DRIVER, TOOLARM_STEP, TOOLARM_DIR);     // Same but for toolarm

void setup() {
    Serial.begin(115200);

    // Define Motor Driver pin directions
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

    // Define Limit Switch and E-Stop Directions
    pinMode(CARRIAGE_LIMIT, INPUT_PULLUP);
    pinMode(TOOLARM_LIMIT, INPUT_PULLUP);
    pinMode(E_STOP, INPUT);

    // Initialize Motors
    digitalWrite(MANDREL_EN, LOW);
    digitalWrite(MANDREL_DIR, LOW);
    digitalWrite(CARRIAGE_EN, LOW);
    digitalWrite(CARRIAGE_DIR, HIGH);
    digitalWrite(TOOLHEAD_EN, LOW);
    digitalWrite(TOOLHEAD_DIR, LOW);
    digitalWrite(TOOLARM_EN, LOW);
    digitalWrite(TOOLARM_DIR, LOW);

    // Set speeds and accelerations
    mandrel.setMaxSpeed(2000);
    mandrel.setSpeed(1000);
    carriage.setMaxSpeed(2000);
    carriage.setAcceleration(5000);
    carriage.setSpeed(1000);
    toolhead.setMaxSpeed(8000);
    toolhead.setAcceleration(10000);
    toolarm.setMaxSpeed(8000);
    toolarm.setAcceleration(10000);

    // Layer(length, angle, offset, stepover, dwell, diameter)
    layers[0] = new Layer(509, 30, 0, 4.0, 180, 79.0);
    
    // Tell the program there is 1 layer to process
    totalLayers = 1;

    manD = 79.0;

    // Define mandrel profile — cylinder with hemispherical ends
    // Total length: 150mm (34.5mm dome + 100mm cylinder + 34.5mm dome)
    // Radius: 50mm throughout, hemisphere described by r(x) = sqrt(R^2 - (R-x)^2)
    float R = manD / 2.0;  // Hemisphere radius = cylinder radius for tangent transition

    // Left hemisphere (using 10 points)
    for (int i = 0; i <= 10; i++) {
        float x = (R / 10.0) * i;
        float y = sqrt(R * R - (R - x) * (R - x));
        toolarmProfile.addPoint(x, y);
    }

    // Cylinder section (using 2 points)
    //toolarmProfile.addPoint(R,  R);   // just inside cylinder start
    toolarmProfile.addPoint(R + 215.0, R);   // mid cylinder
    toolarmProfile.addPoint(R + 400.0, R);   // cylinder end

    // Right hemisphere (using 10 points)
    for (int i = 1; i <= 10; i++) {
        float x = R + 430.0 + (R / 10.0) * i;
        float t = (R / 10.0) * i;          // distance from cylinder end
        float y = sqrt(R * R - t * t);         // same formula as left dome
        toolarmProfile.addPoint(x, y);
    }

    toolarmProfile.setStandoff(55.0);  
    toolarmProfile.compute();

    // Enter Zeroing state on startup
    delay(1000);
    currentState = ZEROING;
    Serial.println("STATE = ZEROING");
}

void loop() {
    // If no layers exist, keep motors stopped
    //if (totalLayers == 0) return;

    // Pointer to the current active layer for clarity
    Layer* activeLayer = layers[activeLayerIndex];

    switch (currentState) {

        // Stop all motors if the emergency stop is pressed
        case PAUSED: {

            // Stop all motion
            mandrel.setSpeed(0);
            mandrel.runSpeed();   
            carriage.stop();
            carriage.run();
            toolarm.stop();
            toolarm.run();

            break;  // Motors held in position
        }

        // Move the carriage to limit switch to set it's home "zero" position and prepare for winding
        case ZEROING: {
            previousState = currentState;

            toolhead.setCurrentPosition(0);
            toolheadZeroed = true;

            if (!carriageZeroed) { // Zero the carriage
                carriage.setSpeed(-1000);
                carriage.runSpeed();

                if (digitalRead(CARRIAGE_LIMIT) == HIGH) {
                    carriage.stop();
                    delay(500);
                    carriage.setCurrentPosition(0);
                    carriage.moveTo(1000);
                    while (carriage.distanceToGo() != 0) carriage.run();
                    carriage.setCurrentPosition(0);
                    carriageZeroed = true;
                    Serial.println("Carriage Zero Set");
                }
            }
            else if (!toolarmZeroed) { // Zero the toolarm
                toolarm.setSpeed(-6000);
                toolarm.runSpeed();

                if (digitalRead(TOOLARM_LIMIT) == LOW) {
                    toolarm.stop();
                    delay(500);
                    toolarm.setCurrentPosition(0);
                    toolarm.moveTo(2000);
                    while (toolarm.distanceToGo() != 0) toolarm.run();
                    toolarmZeroed = true;
                    toolarm.setCurrentPosition(0);
                    Serial.println("Toolarm Zero Set");
                }
            }
            if (carriageZeroed && toolarmZeroed && toolheadZeroed) { // Move to the starting positions and start winding

                long offsetSteps = (long)(activeLayer->getOffset() * stepsPerMM);
                carriage.moveTo(offsetSteps);
                while (carriage.distanceToGo() != 0) carriage.run();

                float firstArmTarget = toolarmProfile.getToolarmTarget(0.0);
                float firstTravel    = toolarmZero - firstArmTarget;
                if (firstTravel < 0.0) firstTravel = 0.0;
                toolarm.moveTo((long)(firstTravel * toolarmStepsPerMM));
                while (toolarm.distanceToGo() != 0) toolarm.run();

                long firstTarget = activeLayer->getToolheadTarget(true, toolheadStepsPerRev);
                toolhead.moveTo(firstTarget);
                while (toolhead.distanceToGo() != 0) { toolhead.run(); }

                lastManStep = mandrel.currentPosition();
                delay(2000);
                currentState = MOVING;
                Serial.println("Zeroing Complete");
            }
            break;
        }

        // Move the carriage and mandrel at speeds relative to the given winding angle
        case MOVING: {
            //Increment pass
            if (previousState == DWELLING) {
                activeLayer->countPass();
            }

            previousState = currentState;

            // Get needed information from the Layer class
            float currentPosMM = carriage.currentPosition() / stepsPerMM;             // Convert position in steps to mm
            float ratio = activeLayer->getStepRatio(manD, stepsPerMM, stepsPerRev);   // Get step ratio
            float target = activeLayer->getTargetEndpoint();                          // Get end point for layer
            float moveSign = activeLayer->isGoingForward() ? 1.0 : -1.0;              // Direction of carriage motion
            const float maxCarriageSteps = 4000.0;                                    // Safe carriage step limit

            // Scale mandrel speed down if carriage would exceed limit
            float safeMandelSpeed = min(1000.0f, maxCarriageSteps / ratio);
            mandrel.setSpeed(safeMandelSpeed);

            // Set carriage speed proportional to mandrel speed
            float carriageSpeed = safeMandelSpeed * ratio * moveSign;
            carriage.setSpeed(carriageSpeed);
            
            // Run both motors
            mandrel.runSpeed();
            carriage.runSpeed();

            // Calculate Toolarm Target
            float toolarmTarget = toolarmProfile.getToolarmTarget(currentPosMM);  // radius + standoff in mm
            float travel = toolarmZero - toolarmTarget;
            if (travel < 0.0) travel = 0.0;
            long toolarmSteps = (long)(travel * toolarmStepsPerMM);
            toolarm.moveTo(toolarmSteps);
            toolarm.run();

            // Enter Dwell state if the carriage has reached the end of a pass
            if (activeLayer->isGoingForward() && currentPosMM >= target) currentState = DWELLING;  
            if (!activeLayer->isGoingForward() && currentPosMM <= target) currentState = DWELLING;

            // Prepare Dwell if direction just switched
            if (currentState == DWELLING) {

                toolarm.stop();
                toolarm.setCurrentPosition(toolarm.currentPosition());

                // Calculate total needed rotation
                float totalDeg = activeLayer->getDwell() + activeLayer->getStepoverDeg();   // Total mandrel rotation needed
                long stepsToDwell = (totalDeg / 360.0) * stepsPerRev;           // Calculate how many steps to turn required angle
                dwellTargetStep = mandrel.currentPosition() + stepsToDwell;     // Calculates which step number to end Dwell state
            }
            break;  // Exit Moving state
        }

        // Spin the mandrel to align the fiber for the next pass, no carriage motion
        case DWELLING: {
            previousState = currentState;

            // Cancel any queued carriage motion
            carriage.stop();                                         
            carriage.setCurrentPosition(carriage.currentPosition());

            mandrel.runSpeed(); // Rotate mandrel at prior defined constant speed

            if (!toolheadFlipped) {
                // isGoingForward() still holds the OLD direction here (countPass hasn't been called yet)
                // so we target the NEW direction by flipping the sign
                bool nextDirection = !activeLayer->isGoingForward();
                long toolheadTarget = activeLayer->getToolheadTarget(nextDirection, toolheadStepsPerRev);
                toolhead.moveTo(toolheadTarget);
                toolhead.run();

                if (toolhead.distanceToGo() == 0) {
                    toolheadFlipped = true;  // Toolhead is in position
                }
            }
            if (toolheadFlipped && mandrel.currentPosition() >= dwellTargetStep) {     // Check if current position has reached end of dwell
                if (activeLayer->isDone()) { // Check if layer is finished and update current state
                    currentState = FINISHED;
                }
                else {
                    lastManStep = mandrel.currentPosition(); // Update mandrel position before resuming
                    currentState = MOVING;
                }
                toolheadFlipped = false;
            }
            break;  // Exit Dwell state
        }

        // Move on to the next layer or end the wind if there are none
        case FINISHED: {
            previousState = currentState;

            // Move on to the next layer in the array if there is one
            if (activeLayerIndex < totalLayers - 1) {
                Serial.printf("Layer %d complete\n", activeLayerIndex);

                // Reset system variables for the next layer
                activeLayerIndex++;
                lastManStep = mandrel.currentPosition();
                toolheadFlipped = false;  
                currentState = MOVING;
            }
            else {
                // All layers from the UI are done
                mandrel.setSpeed(0);
                carriage.setSpeed(0);
                // Need to send a signal back to the UI

                Serial.println("All layers complete. Winding over");
            }
            break;  // Exit Finished state
        }
    }
}