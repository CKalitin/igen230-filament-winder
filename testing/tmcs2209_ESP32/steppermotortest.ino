const int stepPin = 25;
const int dirPin  = 26;
const int enPin   = 27;

// Speed control:
// Smaller = faster, larger = slower
// Try 2000 (slow) down to ~200 (fast). Don’t go too low or it’ll skip/stall.
int stepDelayUs = 800;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(enPin, LOW);      // enable (many drivers are LOW=enabled)
  digitalWrite(dirPin, HIGH);    // set ONE direction and keep it
}

void loop() {
  // Continuous spinning at chosen speed:
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelayUs);
}

