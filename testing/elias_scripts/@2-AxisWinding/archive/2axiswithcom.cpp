#include <Arduino.h>
#include <AccelStepper.h> // Library that allows arduino ide to talk to motor drivers
#include <ArduinoJson.h>  // Include the ArduinoJson library (Install via Library Manager)

// Mandrel pins
#define MANDREL_DIR    19 // Mandrel driver Direction
#define MANDREL_STEP   18 // Mandrel driver Step
#define MANDREL_EN     21 // Mandrel driver Enable

// Carriage pins
#define CARRIAGE_DIR   17 // Carriage driver Direction
#define CARRIAGE_STEP  16 // Carriage driver Step
#define CARRIAGE_EN    5  // Carriage driver Enable
#define CARRIAGE_LIMIT 35 // Carriage limit switch

// Emergency shut off pin
#define E_STOP         13 // Emengency shut off

class Layer {
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
                float calcPasses = (circ * cos(angleRad)) / _stepover;  // Calculate passes needed for 100% coverage
                _pass = (int)ceil(calcPasses);
                if (_pass % 2 != 0) _pass++;    // Round up to the nearest EVEN integer so carriage returns to start
            }

        // GETTERS: Read-only access
        float getLength() const { return _length; }
        float getAngle()  const { return _angle; }
        float getDwell()  const { return _dwell; }

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
const int maxLayers = 10;   // Updated to match UI layer limits
int totalLayers = 0;        // Number of layers recieved from the UI
int activeLayerIndex = 0;   // Layer currently winding
Layer* layers[maxLayers];   // An array of pointers to Layer objects

// Stores layer data received from UI in the layer pointer array
void LayerFromUI(float length, float angle, float offset, float stepover, float dwell, float diameter) {
    if (totalLayers < maxLayers) {  // Make sure data can fit in the array
        layers[totalLayers] = new Layer(length, angle, offset, stepover, dwell, diameter);  // Create a new layer object
        totalLayers++;
    }
}

// Possible Winding/Operation States
enum windingState {
    WAITING_FOR_JSON, // Waits for serial input from UI
    PAUSED,           // Stops all motion
    ZEROING,          // Zeroing the carriage before winding
    MOVING,           // Carriage and mandrel motion during winding
    DWELLING,         // Carriage stopped, mandrel rotating during dwell period
    FINISHED          // Carriage and mandrel stopped
};

// Start in WAITING_FOR_JSON
windingState currentState = WAITING_FOR_JSON; 
windingState previousSate;          

// Harware Variables (Subject to change)
const float Pitch = 2.0;      
const int motorSteps = 200.0; 
const int microsteps = 16;    
const int motorTeeth = 20;    
const int carTeeth = 20;      
const int manTeeth = 40;      

const float stepsPerMM  = (motorSteps * microsteps) / (carTeeth * Pitch);               
const float stepsPerRev = (motorSteps * microsteps) * ((float)manTeeth / motorTeeth);   

// Winding Parameters (from UI)
float manD = 0;   // Global Mandrel Diameter (mm)

// Global Control Variables
float carAccumulator = 0;   
long lastManStep = 0;       
long dwellTargetStep = 0;   

// Stepper Objects
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);     
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);  

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
    mandrel.setMaxSpeed(2000);
    mandrel.setSpeed(1000);
    carriage.setMaxSpeed(2000);
    carriage.setAcceleration(5000);
    carriage.setSpeed(1000);

    // Send READY signal to the UI to update the connection pill status
    Serial.println("READY");
}

void loop() {
    switch (currentState) {

        case WAITING_FOR_JSON: {
            if (Serial.available()) {
                // 1. Wait a brief moment for the full buffer to fill
                delay(100); 

                // 2. Use a larger buffer for complex JSON
                // Increase 2048 if you have more than 10-15 layers
                JsonDocument doc; 
                DeserializationError error = deserializeJson(doc, Serial);

                if (error) {
                    Serial.print("JSON Parse failed: ");
                    Serial.println(error.c_str());
                    // Clear buffer to prevent repeat errors
                    while(Serial.available()) Serial.read(); 
                    break;
                }

                // Clean up previous job
                for(int i = 0; i < totalLayers; i++) {
                    delete layers[i];
                }
                totalLayers = 0;
                activeLayerIndex = 0;

                // 3. Match the WinderUI.html structure exactly
                manD = doc["mandrel_diameter"] | 0.0;
                JsonArray jsonLayers = doc["layers"];
        
                for (JsonObject l : jsonLayers) {
                    // Note: The UI uses these specific keys
                    float length   = l["length"] | 0.0;
                    float angle    = l["angle"] | 0.0;
                    float offset   = l["offset"] | 0.0;
                    float stepover = l["stepover"] | 0.0;
                    float dwell    = l["dwell"] | 0.0;
            
                    if (length > 0) {
                        LayerFromUI(length, angle, offset, stepover, dwell, manD);
                    }
                }

                if (totalLayers > 0) {
                    Serial.print("Success! Layers loaded: ");
                    Serial.println(totalLayers);
                    currentState = ZEROING;
                } else {
                    Serial.println("Error: JSON parsed but no valid layers found.");
                }
            }
            break;
        }

        case PAUSED: {
            if (digitalRead(E_STOP) == HIGH) {
                currentState = previousSate;
            }
            break;  
        }

        case ZEROING: {
            // Safety check: ensure layers exist before zeroing
            if (totalLayers == 0) {
                Serial.println("Error: No layers to process");
                currentState = WAITING_FOR_JSON;
                break;
            }

            carriage.setSpeed(-600); // Slowly move to limit switch
            carriage.runSpeed();

            if (digitalRead(CARRIAGE_LIMIT) == LOW) {    
                carriage.stop();                         
                delay(200);                              

                carriage.setCurrentPosition(0);
                carriage.moveTo(200); 
                while (carriage.distanceToGo() != 0) {
                    carriage.run();   
                }

                carriage.setCurrentPosition(0);           
                lastManStep = mandrel.currentPosition();  
                delay(2000);                              
                
                Serial.println("STATE: WINDING...");       
                currentState = MOVING;                    
            }
            break; 
        }

        case MOVING: {
            Layer* activeLayer = layers[activeLayerIndex];

            float ratio = activeLayer->getStepRatio(manD, stepsPerMM, stepsPerRev);   
            float target = activeLayer->getTargetEndpoint();                          

            mandrel.runSpeed();                         
            long stepNow = mandrel.currentPosition();   
            
            if (stepNow != lastManStep) {           
                long delta = stepNow - lastManStep; 
                lastManStep = stepNow;              

                float moveSign = (activeLayer->isGoingForward()) ? 1.0 : -1.0;  
                carAccumulator = carAccumulator + (delta * ratio * moveSign);   

                if (abs(carAccumulator) >= 1.0) {  
                    long numStep = (long)carAccumulator;        
                    carriage.move(numStep);                     
                    carAccumulator = carAccumulator - numStep;  
                }
            }
            carriage.run(); 

            float currentPosMM = carriage.currentPosition() / stepsPerMM;   

            if (activeLayer->isGoingForward() && currentPosMM >= target) currentState = DWELLING;  
            if (!activeLayer->isGoingForward() && currentPosMM <= target) currentState = DWELLING;  

            if (currentState == DWELLING) {
                float totalDeg = activeLayer->getDwell() + activeLayer->getStepoverDeg();   
                long stepsToDwell = (totalDeg / 360.0) * stepsPerRev;           
                dwellTargetStep = mandrel.currentPosition() + stepsToDwell;     
            }
            break; 
        }

        case DWELLING: {    
            Layer* activeLayer = layers[activeLayerIndex];

            mandrel.runSpeed(); 

            if (mandrel.currentPosition() >= dwellTargetStep) {     
                activeLayer->countPass();     

                if (activeLayer->isDone()) {  
                    currentState = FINISHED;
                }
                else {
                    lastManStep = mandrel.currentPosition(); 
                    currentState = MOVING;
                }
            }
            break; 
        }

        case FINISHED: {
            if (activeLayerIndex < totalLayers - 1) {
                Serial.printf("Layer %d complete\n", activeLayerIndex);
                activeLayerIndex++;
                carAccumulator = 0;
                lastManStep = mandrel.currentPosition();
                currentState = MOVING;
            }
            else {
                mandrel.setSpeed(0);
                carriage.setSpeed(0);
                Serial.println("STATE: COMPLETE");
                
                // Job done! Ready for a new job.
                currentState = WAITING_FOR_JSON; 
            }
            break; 
        }
    }
}
