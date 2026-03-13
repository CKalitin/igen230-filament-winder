#include "config.h"

float getStepRatio(float mandrelDiameter, float angle) {
    float safeAngle = angle;
    if (safeAngle > 89.0f) safeAngle = 89.0f; // Prevent tan(90) singularity
    if (safeAngle < 1.0f) safeAngle = 1.0f; // Prevent too small angles that cause very high step ratios

    float angleRad = safeAngle * 3.14159f / 180.0f; // Convert to radians
    float mmPerRev = 3.14159f * mandrelDiameter / tan(angleRad); // Calculate mm per revolution based on the winding angle
    return (mmPerRev * )
}