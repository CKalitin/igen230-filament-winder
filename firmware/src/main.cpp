#include <main.h>

#define LIMIT_SWITCH_PIN 5

int carriage_dir = 1;

int blinks = 0;

void setup() {
	pinMode(2, OUTPUT); // LED pin
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

	initSteppers();

  carriageStepper.setSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * 0.5 * carriage_dir);
  mandrelStepper.setSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * 0.5 * carriage_dir);
}

void loop() {
  carriageStepper.runSpeed();
  mandrelStepper.runSpeed();

  if (digitalRead(LIMIT_SWITCH_PIN) == HIGH && false==true) {
    carriage_dir = -carriage_dir;
    carriageStepper.setSpeed(CARRIAGE_MOTOR_PARAMS.microStepsPerRev * 0.5 * carriage_dir);
    mandrelStepper.setSpeed(MANDREL_MOTOR_PARAMS.microStepsPerRev * 0.5 * carriage_dir);

    // sleep 0.5s
    delay(500);
    // toggle LED
    digitalWrite(2, !digitalRead(2));
  }

  if (blinks * 500 < millis()) {
    blinks++;
    digitalWrite(2, !digitalRead(2));
  }
}
