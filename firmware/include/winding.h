/// @file winding.h
/// @brief Wind-profile storage and winding state-machine controller.
///
/// A WindProfile is the single place that stores every parameter defining a
/// complete winding job (mandrel diameter + ordered list of Layers).  It is
/// designed to be populated from a UI or serial interface before winding
/// begins.  All fields are directly readable and writable.
///
/// The Winding namespace exposes the public API for the state machine that
/// executes the profile.  Call Winding::init() once in setup() and
/// Winding::update() every loop() iteration.

#pragma once

#include "layer.h"

// ============================================================================
//  Winding States
// ============================================================================

/// Possible states of the winding controller.
enum class WindingState : uint8_t {
    IDLE,       ///< No wind profile loaded or machine freshly powered on.
    PAUSED,     ///< Motors held in position; awaiting resume or zero command.
    ZEROING,    ///< Homing carriage toward the limit switch.
    WINDING,    ///< Active winding — carriage electronically geared to mandrel.
    DWELLING,   ///< Extra mandrel rotation at the end of a pass.
    COMPLETE    ///< All layers finished; motors stopped.
};

// ============================================================================
//  Wind Profile
// ============================================================================

/// @struct WindProfile
/// @brief All parameters that define a complete winding job.
///
/// Populate mandrelDiameter, then call addLayer() for each layer in order.
/// The profile can be cleared and re-used between jobs.
struct WindProfile {
    float mandrelDiameter            = 0.0f;   ///< Mandrel OD (mm).
    int   layerCount                 = 0;       ///< Number of active layers.
    Layer layers[MAX_LAYERS];                   ///< Layer storage (0 … layerCount-1).

    /// Append a new layer using the stored mandrelDiameter.
    /// @return true on success, false if the profile is full.
    bool addLayer(float length, float angle, float offset,
                  float stepover, float dwell);

    /// Remove all layers and reset the profile.
    void clear();

    /// @return true if the profile contains at least one layer and a valid diameter.
    bool isValid() const;
};

// ============================================================================
//  Winding Controller
// ============================================================================

/// @namespace Winding
/// @brief Public API for the winding state machine.
namespace Winding {

    /// Initialise internal state (call once from setup()).
    void init();

    /// Run one iteration of the state machine (call every loop()).
    void update();

    // ── Commands ─────────────────────────────────────────────────────────────

    /// Begin a zeroing (homing) sequence, then start winding.
    void start();

    /// Pause all motion immediately.
    void pause();

    /// Resume from a paused state.
    void resume();

    // ── Profile access ───────────────────────────────────────────────────────

    /// Get a mutable reference to the active wind profile.
    WindProfile& getProfile();

    // ── Status queries ───────────────────────────────────────────────────────

    /// Current state of the winding controller.
    WindingState getState();

    /// Index of the layer currently being wound (0-based).
    int getActiveLayerIndex();

}  // namespace Winding
