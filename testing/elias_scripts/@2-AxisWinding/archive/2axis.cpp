#include <Arduino.h>
#include <AccelStepper.h> // this is the library that allows arduino ide to talk to motor drivers
#include <ArduinoJson.h>

// Carriage pins (Motor 1 on PCB)
#define CARRIAGE_DIR   17
#define CARRIAGE_STEP  16
#define CARRIAGE_EN    5
#define CARRIAGE_LIMIT 35   // Limit switch 1 on PCB

// Mandrel pins (Motor 2 on PCB)
#define MANDREL_DIR    19   
#define MANDREL_STEP   18
#define MANDREL_EN     21

// Emergency shut off pin
#define E_STOP         13

class Layer{
    // Information not accessible outside the Layer class
    private:
        float _length;         // Length of the layer (mm)
        float _angle;          // Fibre angle for the layer (degrees)
        float _offset;         // Where the first pass starts on mandrel (mm)
        float _stepover;       // Distance moved per pass (to control fibre overlap in each layer) (mm)
        float _dwell;          // Extra mandrel rotation at the end of each pass (degrees)
        float _diameter;       // Mandrel diameter (mm)
        int   _pass;           // Number of passes to complete (a pass is from left to right, not there and back)
        int   _passDone;       // Number of passes Completed
        bool  _goForward;      // Track direction of motion


    // Information accesible to the greater program
    public:
        // Tansfer repective parameters into private storage
        Layer(float l, float a, float off, float stpo, float d, float manD)
            : _length(l), _angle(a), _offset(off), _stepover(stpo), _dwell(d), _diameter(manD), _passDone(0), _goForward(true) { 
                float safeAngle = _angle;
                if (safeAngle > 89.0) safeAngle = 89.9;    // Prevent divide by zero
                if (safeAngle < 1.0) safeAngle = 1.0;      // Prevent divide by zero
                float angleRad = radians(safeAngle);       // Convert to radians for calculations

                float circ = PI * manD; // Calculate cicumference
                float calcPasses = (_length * cos(angleRad)) / _stepover;  // Calculate passes needed for 100% coverage
                _pass = ((int)ceil(calcPasses) * 2) + 1;
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

        // Lets the loop know which way the carriage is moving
        bool isGoingForward() const { return _goForward; }

        // Logic for the loop to use
        void countPass() {
            _passDone++;              // Track one completed pass along the mandrel
            _goForward = !_goForward; // Flips direction for the return stroke
        }                
        bool isDone() const { return _passDone >= _pass; }  // Returns true when layer is complete

        // Inside class Layer public section:
        float getEstimatedTimeSeconds(float mandrelSpeedRPM) {
            if (mandrelSpeedRPM <= 0) return 0;
            
            float safeAngle = _angle;
            if (safeAngle > 89.0) safeAngle = 89.9;
            float angleRad = radians(safeAngle);

            // Mandrel revolutions per single pass
            float revsPerPass = (_length / tan(angleRad)) / (PI * _diameter);
            float minutesPerPass = revsPerPass / mandrelSpeedRPM;
            
            // Add time for the dwell/stepover rotation
            float dwellRevs = (_dwell + getStepoverDeg()) / 360.0;
            float dwellMinutes = dwellRevs / mandrelSpeedRPM;

            return (minutesPerPass + dwellMinutes) * _pass * 60.0; // Total seconds
        }
};

// Array of pointers to store the data for each layer from the UI
const int maxLayers  = 5;   // Maximum layers the machine can handle (subject to change)
int totalLayers      = 0;   // Number of layers recieved from the UI
int activeLayerIndex = 0;   // Layer currently winding
Layer* layers[maxLayers];   // An array of pointers to Layer objects (chat gave me this idk how pointers work)

// Stores layer data received from UI in the layer pointer array
void LayerFromUI(float length, float angle, float offset, float stepover, float dwell, float diameter) {
    
    if (totalLayers < maxLayers) {  // Make sure data can fit in the array
        layers[totalLayers] = new Layer(length, angle, offset, stepover, dwell, diameter);  // Create a new layer object
        totalLayers++;
    }
}

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
const float Pitch    = 2.0; // Belt pitch (in mm)
const int motorSteps = 200; // Number of steps motor makes per revolution
const int microsteps = 16;  // Not Sure about this, ask Loki
const int motorTeeth = 20;  // Number of pulley teeth on motor pulleys
const int carTeeth   = 20;  // Number of pulley teeth on carriage pulley
const int manTeeth   = 20;  // Number of pulley teeth on mandrel pulley

const float stepsPerMM  = (motorSteps * microsteps) / (carTeeth * Pitch);               // Carriage steps per mm moved
const float stepsPerRev = (motorSteps * microsteps) * ((float)manTeeth / motorTeeth);   // Required carriage steps per mandrel step

// Winding Parameters (from UI)
float manD;   // Mandrel Diameter (mm)

// Global Control Variables
long lastManStep;              // Stores previous loop's mandrel position
long dwellTargetStep;          // Number of extra steps mandrel must move at end of a pass
bool carriageZeroed  = false;  // Tracks if the carriage has zeroed
bool offsetReached   = false;  // Tracks if carriage has reached its required offset


// Stepper Objects
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);     // Creates object called mandrel to store current step position, target position, speed, timing
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);  // Same as above but for the carriage

void setup() {
    Serial.begin(115200);

    // Define Motor Driver pin directions
    pinMode(MANDREL_STEP, OUTPUT);
    pinMode(MANDREL_DIR, OUTPUT);
    pinMode(MANDREL_EN, OUTPUT);
    pinMode(CARRIAGE_STEP, OUTPUT);
    pinMode(CARRIAGE_DIR, OUTPUT);
    pinMode(CARRIAGE_EN, OUTPUT);

    // Define Limit Switch and E-Stop Directions
    pinMode(CARRIAGE_LIMIT, INPUT_PULLUP);
    pinMode(E_STOP, INPUT);

    // Initialize Motors and Switches
    digitalWrite(MANDREL_EN, LOW);
    digitalWrite(CARRIAGE_EN, LOW);
    digitalWrite(MANDREL_DIR, LOW);
    digitalWrite(CARRIAGE_DIR, HIGH);

    // Set speeds and accelerations
    mandrel.setMaxSpeed(3000);
    mandrel.setSpeed(1000);
    carriage.setMaxSpeed(5000);
    carriage.setAcceleration(6000);

    // Manually add a test layer (since UI isn't connected yet)
    // Parameters: length (mm), angle (deg), offset (mm), stepover (mm), dwell (deg), diameter (mm)
    LayerFromUI(230.0, 60.0, 0.0, 3.0, 100.0, 55.0);
    
    // Set global mandrel diameter (mm)
    manD = 55.0;

    // Enter Zeroing state on startup
    currentState = ZEROING;
}

void loop() {
    
    if (totalLayers == 0) return;                   // If no layers exist, keep motors stopped
    Layer* activeLayer = layers[activeLayerIndex];  // Pointer to the current active layer

        // Emergency shut off logic
    if (digitalRead(E_STOP) == HIGH) {
        if (currentState != PAUSED) {
            previousState = currentState;  // Save state before pausing
            currentState = PAUSED;
        }
    }
    else {
        if (currentState == PAUSED) {
            currentState = previousState;
        }
    }

    if (previousState != currentState) {
        switch (currentState) {
            case PAUSED:    Serial.println("STATE = PAUSED");   break;
            case ZEROING:   Serial.println("STATE = ZEROING");  break;
            case MOVING:    Serial.println("STATE = MOVING");   break;
            case DWELLING:  Serial.println("STATE = DWELLING"); break;
            case FINISHED:  Serial.println("STATE = FINISHED"); break;
        }
    }

    // State Machine
    switch (currentState) {

        // Stop all motors if the emergency stop is pressed
        case PAUSED: {

            // Stop all motion
            mandrel.setSpeed(0);
            mandrel.runSpeed();   
            carriage.stop();
            carriage.run();

            break;  // Motors held in position, waiting for command to start or zero
        }

        // Move the carriage to limit switch to set it's home "zero" position and prepare for winding
        case ZEROING: {
            previousState = currentState;

            carriage.setSpeed(-800); // Slowly move to limit switch
            carriage.runSpeed();

            if (digitalRead(CARRIAGE_LIMIT) == HIGH) {   // Check if limit switch is active
                carriage.stop();                         // Stop motion
                delay(500);                              // Wait half a second

                // Temporarily set the carriage zero/home and move away from the limit switch a bit
                carriage.setCurrentPosition(0);
                carriage.moveTo(1000);
                while (carriage.distanceToGo() != 0) carriage.run();
                carriage.setCurrentPosition(0);
                carriageZeroed = true;
                Serial.println("Carriage Zero Set");
            }
            if (carriageZeroed) {   // Move to the starting positions and start winding
                
                long offsetSteps = (long)(activeLayer->getOffset() * stepsPerMM);
                carriage.moveTo(offsetSteps);
                while (carriage.distanceToGo() != 0) carriage.run();
                if (carriage.distanceToGo() == 0) offsetReached = true;
                
                if (offsetReached) {
                    carriage.setCurrentPosition(0);           // Update carriage position to final zero/home position
                    lastManStep = mandrel.currentPosition();  // Update mandrel postion to zero
                    delay(2000);                              // Wait two seconds
                    currentState = MOVING;                    // Initialize next state
                    Serial.println("Zeroing Complete");       // Print zeroing confirmation to screen
                }
            }
            break;  // Exit Zeroing state
        }

        // Move the carriage and mandrel at speeds relative to the given winding angle
        case MOVING: {

            //Increment pass
             if (previousState == DWELLING) {
                activeLayer->countPass();
            }

            previousState = currentState;

            // Get needed information from the Layer class
            float ratio = activeLayer->getStepRatio(manD, stepsPerMM, stepsPerRev);   // Get step ratio
            float target = activeLayer->getTargetEndpoint();                          // Get end point for layer
            float moveSign = activeLayer->isGoingForward() ? 1.0 : -1.0;              // Safe carriage step limit
            
            const float maxCarriageSteps = 4000.0;  // Safe carriage step limit

            // Scale mandrel speed down if carriage would exceed limit
            float safeMandelSpeed = min(1000.0f, maxCarriageSteps / ratio);
            mandrel.setSpeed(safeMandelSpeed);

            // Set carriage speed proportional to mandrel speed
            float carriageSpeed = safeMandelSpeed * ratio * moveSign;
            carriage.setSpeed(carriageSpeed);
            
            // Run both motors
            mandrel.runSpeed();
            carriage.runSpeed();

            // Check for End of Pass
            float currentPosMM = carriage.currentPosition() / stepsPerMM;   // Convert position in steps to mm

            // Enter Dwell state if the carriage has reached the end of a pass
            if (activeLayer->isGoingForward() && currentPosMM >= target) currentState = DWELLING;  
            if (!activeLayer->isGoingForward() && currentPosMM <= target) currentState = DWELLING;

            // Prepare Dwell if direction just switched
            if (currentState == DWELLING) {

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

            if (mandrel.currentPosition() >= dwellTargetStep) {     // Check if current position has reached end of dwell
                if (activeLayer->isDone()) {  // Check if layer is finished and update current state
                    currentState = FINISHED;
                }
                else {
                    lastManStep = mandrel.currentPosition(); // Update mandrel position before resuming
                    currentState = MOVING;
                }
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
