/// @file layer.h
/// @brief Winding-layer definition — geometry, gearing math, and progress tracking.
///
/// A Layer represents one set of helical passes at a given fibre angle across
/// a defined length of the mandrel.  It computes the electronic-gearing ratio,
/// pass count, and per-pass target positions used by the winding state machine.

#pragma once

#include <Arduino.h>

/// Maximum number of layers in a single wind profile.
constexpr int MAX_LAYERS = 10;

/// @class Layer
/// @brief Describes one winding layer's geometry and tracks pass progress.
class Layer {
public:
    // ── Constructors ─────────────────────────────────────────────────────────

    /// Default constructor — creates an uninitialised (empty) layer.
    Layer();

    /// Construct a fully specified winding layer.
    /// @param length     Winding-zone length along the mandrel (mm).
    /// @param angle      Fibre angle relative to the mandrel axis (degrees, 1–89).
    /// @param offset     Start of winding zone measured from home (mm).
    /// @param stepover   Circumferential fibre shift per pass (mm).
    /// @param dwell      Extra mandrel rotation at each turn-around (degrees).
    /// @param diameter   Mandrel outer diameter at this layer (mm).
    Layer(float length, float angle, float offset, float stepover,
          float dwell, float diameter);

    // ── Read-only access ─────────────────────────────────────────────────────

    float getLength()          const { return length_; }
    float getAngle()           const { return angle_; }
    float getOffset()          const { return offset_; }
    float getStepover()        const { return stepover_; }
    float getDwell()           const { return dwell_; }
    float getDiameter()        const { return diameter_; }
    int   getTotalPasses()     const { return totalPasses_; }
    int   getPassesCompleted() const { return passesCompleted_; }
    bool  isGoingForward()     const { return goingForward_; }

    // ── Mutable access (for UI / serial configuration) ───────────────────────

    void setLength(float v)   { length_   = v; recalcPasses(); }
    void setAngle(float v)    { angle_    = v; recalcPasses(); }
    void setOffset(float v)   { offset_   = v; }
    void setStepover(float v) { stepover_ = v; recalcPasses(); }
    void setDwell(float v)    { dwell_    = v; }
    void setDiameter(float v) { diameter_ = v; recalcPasses(); }

    // ── Winding calculations ─────────────────────────────────────────────────

    /// Compute the electronic-gearing ratio (carriage microsteps per mandrel
    /// microstep) for the fibre angle and mandrel diameter of this layer.
    /// @param carriageStepsPerMM  Carriage motor microsteps per mm of travel.
    /// @param mandrelStepsPerRev   Mandrel motor microsteps per mandrel revolution.
    float getStepRatio(float carriageStepsPerMM, float mandrelStepsPerRev) const;

    /// Mandrel rotation (degrees) required to shift the fibre by one stepover
    /// width around the circumference.
    float getStepoverDegrees() const;

    /// Carriage target position for the current pass direction (mm from home).
    float getTargetEndpoint() const;

    // ── Progress tracking ────────────────────────────────────────────────────

    /// Record one completed pass and reverse the travel direction.
    void countPass();

    /// @return true when every pass for this layer has been completed.
    bool isDone() const { return passesCompleted_ >= totalPasses_; }

    /// Reset runtime state (passes completed, direction) for re-winding.
    void resetProgress();

private:
    // ── Configuration (set once, or mutated via setters) ─────────────────────

    float length_   = 0.0f;
    float angle_    = 45.0f;
    float offset_   = 0.0f;
    float stepover_ = 1.0f;
    float dwell_    = 0.0f;
    float diameter_ = 0.0f;

    // ── Runtime state ────────────────────────────────────────────────────────

    int  totalPasses_     = 0;
    int  passesCompleted_ = 0;
    bool goingForward_    = true;

    /// Clamp angle to [1, 89] degrees to prevent divide-by-zero in trig.
    static float clampAngle(float angle);

    /// Recompute totalPasses_ from current geometry parameters.
    void recalcPasses();
};
