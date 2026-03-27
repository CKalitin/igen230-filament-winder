#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    JsonDocument doc;

    // Attempt to parse
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) return;

    // Extract global job settings
    int diameter = doc["mandrel_diameter"] | 0;
    int axesCount = doc["axes"] | 2; // Default to 2 if not specified
    
    Serial.print("STATE: RECEIVED JOB | DIAMETER: ");
    Serial.print(diameter);
    Serial.print("MM | AXES: ");
    Serial.println(axesCount);

    // (Optional) Logic based on axes count
    if (axesCount == 2) {
      Serial.println("STATE: 2-Axis Winding");
    } else if (axesCount == 3) {
      Serial.println("STATE: 3-Axis Winding");
    } else if (axesCount == 4) {
      Serial.println("STATE: 4-Axis Winding");
    }

    // Iterate through the layers
    JsonArray layers = doc["layers"];
    int layerCount = 0;

    for (JsonObject layer : layers) {
      layerCount++;
      
      // Extract the parameters
      float angle = layer["angle"];
      float length = layer["length"];
      float stepover = layer["stepover"];
      int dwell = layer["dwell"] | 0; // Default to 0 if not provided

      // Print a detailed breakdown to the UI Serial Log
      Serial.print("STATE: > LAYER ");
      Serial.print(layerCount);
      Serial.print(" | Angle: ");
      Serial.print(angle);
      Serial.print(" | Len: ");
      Serial.print(length);
      Serial.print("mm | Step: ");
      Serial.print(stepover);
      Serial.print("mm | Dwell: ");
      Serial.println(dwell);

      // --- Motor Logic Goes Here ---
      // Example: moveAxis(length, angle);
      
      delay(500); // Simulation delay
    }

    // Finalize the job
    Serial.println("STATE: JOB FINISHED");
    Serial.println("READY"); // Tells the UI the machine is idle and ok to send more
  }
}