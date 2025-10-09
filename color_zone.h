#ifndef COLOR_ZONE_H
#define COLOR_ZONE_H

#include "sensor_processing.h"
#include <stdbool.h>

// State for color zone detection
typedef struct {
    bool active; // True if red zone is detected
} ColorZoneState;

// Initialize color zone state
void ColorZone_Init(ColorZoneState *state);

// Run color zone detection based on calibrated redness
// Returns true if red zone is detected (absolute priority)
bool ColorZone_Run(ColorZoneState *state, const SensorProcessor *sp);

// Compute red-based line position/strength using calibration
// Position is in [-1..1] (same convention as SENSOR_WEIGHTS)
// Strength is average redness [0..1]
bool ColorZone_ComputeRedPosition(const uint16_t *raw,
                                  const CalibrationData *cal,
                                  float *position,
                                  float *strength);

#endif // COLOR_ZONE_H
