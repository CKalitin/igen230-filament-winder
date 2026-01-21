#include <AccelStepper.h> // this is the library that allows arduino ide to talk to motor drivers

// -------------------- Pins --------------------
#define MANDREL_STEP 26
#define MANDREL_DIR  27

#define CARRIAGE_STEP 14
#define CARRIAGE_DIR  12

// -------------------- Steppers --------------------
AccelStepper mandrel(AccelStepper::DRIVER, MANDREL_STEP, MANDREL_DIR);// creating an object called mandrel that will store the curennt step position, target position, speed, timing
AccelStepper carriage(AccelStepper::DRIVER, CARRIAGE_STEP, CARRIAGE_DIR);// same as above but for the carriage

// -------------------- Stepper parameters --------------------
const int STEPS_PER_REV = 200;// this number is up to change
const int MICROSTEPS = 16;
const float STEPS_PER_REV_EFF = STEPS_PER_REV * MICROSTEPS; //3200 hundred total steps ?

// -------------------- GT2 belt --------------------
const int GT2_TEETH = 20;      // pulley teeth // will need to change these when we get pulleys
const float GT2_PITCH = 2.0;   // mm

const float MM_PER_REV = GT2_TEETH * GT2_PITCH; // distance per revolution accounting for the belt
const float CARRIAGE_STEPS_PER_MM = STEPS_PER_REV_EFF / MM_PER_REV; // main conversion

// -------------------- Geometry --------------------
float pipeDiameter = 50.0;     // mm MAIN USER INPUTS
float pipeLength   = 500.0;    // mm
float windAngleDeg = 30.0;

// -------------------- Derived --------------------
float carriageStepsPerMandrelStep;// how many carriage steps should happen for each mandrel step
float carriageAccumulator = 0.0; //because the ration is fractional, we need to emit integer values

bool movingForward = true;

void setup() {
  float windAngleRad = radians(windAngleDeg);

  float mmPerMandrelRev = PI * pipeDiameter * tan(windAngleRad); //how fare should the carriage move for one pipe rotation

  float carriageStepsPerMandrelRev = mmPerMandrelRev * CARRIAGE_STEPS_PER_MM; //conver mm to number of steps
      
  carriageStepsPerMandrelStep = carriageStepsPerMandrelRev / STEPS_PER_REV_EFF;

  mandrel.setSpeed(600); // steps/sec halfed the speeds here
  carriage.setMaxSpeed(2000);
}

void loop() {
  mandrel.runSpeed();

  // Step-coupling logic
  static long lastMandrelStep = 0;
  long stepNow = mandrel.currentPosition();

  if (stepNow != lastMandrelStep) {
    lastMandrelStep = stepNow;

    carriageAccumulator += carriageStepsPerMandrelStep;

    if (fabs(carriageAccumulator) >= 1.0) {
      long s = (long)carriageAccumulator;
      carriage.move(s);
      carriageAccumulator -= s;
    }
  }

  carriage.run();

  // --- End-of-travel logic LAST ---
  float carriagePosMM =
      carriage.currentPosition() / CARRIAGE_STEPS_PER_MM; // current position keeps track of the total # of steps beleived to
      // have been taken since the power up or last reset

  if (movingForward && carriagePosMM >= pipeLength) {
    movingForward = false;
    mandrel.setSpeed(-fabs(mandrel.speed()));
  }

  if (!movingForward && carriagePosMM <= 0.0) {
    movingForward = true;
    mandrel.setSpeed(fabs(mandrel.speed()));
  }
}