/// @file layer.cpp
/// @brief Layer class implementation.

#include "layer.h"

// ============================================================================
//  Constructors
// ============================================================================

Layer::Layer()
    : length_(0.0f),
      angle_(45.0f),
      offset_(0.0f),
      stepover_(1.0f),
      dwell_(0.0f),
      diameter_(0.0f),
      totalPasses_(0),
      passesCompleted_(0),
      goingForward_(true) {}

Layer::Layer(float length, float angle, float offset, float stepover,
             float dwell, float diameter)
    : length_(length),
      angle_(angle),
      offset_(offset),
      stepover_(stepover),
      dwell_(dwell),
      diameter_(diameter),
      totalPasses_(0),
      passesCompleted_(0),
      goingForward_(true) {
    recalcPasses();
}

// ============================================================================
//  Winding Calculations
// ============================================================================

float Layer::getStepRatio(float carriageStepsPerMM, float mandrelStepsPerRev) const {
    if (diameter_ <= 0.0f || mandrelStepsPerRev <= 0.0f) return 0.0f;

    float safe     = clampAngle(angle_);
    float rad      = radians(safe);
    // mm of carriage travel per full mandrel revolution at this fibre angle.
    float mmPerRev = PI * diameter_ / tan(rad);
    // Convert to: carriage microsteps per mandrel microstep.
    return (mmPerRev * carriageStepsPerMM) / mandrelStepsPerRev;
}

float Layer::getStepoverDegrees() const {
    if (diameter_ <= 0.0f) return 0.0f;

    float safe = clampAngle(angle_);
    float rad  = radians(safe);
    float circ = PI * diameter_;
    // Mandrel rotation (degrees) to shift fibre by one stepover width.
    return (stepover_ * 360.0f) / (circ * cos(rad));
}

// Returns the carriage target position (mm from home) for the current pass.
// Forward pass: target is the far end of the winding zone (offset + length).
// Return pass:  target is the start of the winding zone (offset).
float Layer::getTargetEndpoint() const {
    return goingForward_ ? (offset_ + length_) : offset_;
}

// ============================================================================
//  Progress Tracking
// ============================================================================

void Layer::countPass() {
    passesCompleted_++;
    goingForward_ = !goingForward_;
}

void Layer::resetProgress() {
    passesCompleted_ = 0;
    goingForward_    = true;
}

// ============================================================================
//  Private Helpers
// ============================================================================

float Layer::clampAngle(float angle) {
    if (angle > 89.0f) return 89.9f;
    if (angle <  1.0f) return  1.0f;
    return angle;
}

// Compute how many passes the carriage needs to fully cover the mandrel circumference.
void Layer::recalcPasses() {
    if (diameter_ <= 0.0f || stepover_ <= 0.0f) {
        totalPasses_ = 0;
        return;
    }

    float safe = clampAngle(angle_);
    float rad  = radians(safe);
    float circ = PI * diameter_;                  // Mandrel circumference (mm).

    // Project the circumference onto the fibre-perpendicular direction.
    // This gives the "width" that needs to be filled by stepover-sized shifts.
    // passes = (circ * cos(angle)) / stepover
    //   - At low angles (axial):  cos ≈ 1  → many passes needed.
    //   - At high angles (hoop):  cos ≈ 0  → few passes needed.
    float calc   = (circ * cos(rad)) / stepover_;
    totalPasses_ = static_cast<int>(ceil(calc));  // Round up so there are no gaps.

    // Force even so the carriage finishes on the same side it started.
    if (totalPasses_ % 2 != 0) {
        totalPasses_++;
    }
}
