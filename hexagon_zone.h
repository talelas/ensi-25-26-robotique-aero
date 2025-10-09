#ifndef HEXAGON_ZONE_H
#define HEXAGON_ZONE_H

#include "sensor_processing.h"
#include <stdbool.h>

// State for hexagon zone detection
typedef struct {
    bool active; // True if intersection detected
    bool right_chosen; // True if right path was chosen
} HexagonZoneState;

// Initialize hexagon zone state
void HexagonZone_Init(HexagonZoneState *state);

// Run hexagon zone detection and right-path selection
// Returns true if intersection detected and right path chosen
bool HexagonZone_Run(HexagonZoneState *state, const SensorProcessor *sp);

#endif // HEXAGON_ZONE_H
