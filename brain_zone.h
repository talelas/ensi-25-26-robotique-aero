#ifndef BRAIN_ZONE_H
#define BRAIN_ZONE_H

#include <stdbool.h>
#include <stdint.h>
#include "sensor_processing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool active;
    unsigned char enter_count;
    unsigned char exit_count;
} BrainZoneState;

// Tunables for detection stability
#define BRAIN_UNIFORM_EPS      0.20f  // max-min uniformity threshold on filtered intensity
#define BRAIN_AVG_BLACK_MIN    0.65f  // minimum average intensity to consider background black
#define BRAIN_ENTER_EPOCHS     5      // consecutive frames required to enter
#define BRAIN_EXIT_EPOCHS      5      // consecutive frames required to exit
// Frames of continuous uniform black after brain exit to consider final stop
#define BRAIN_FINAL_BLACK_FRAMES  40   // ~200ms at 200Hz

void BrainZone_Init(BrainZoneState* s);

// Update detection counters using filtered intensity pattern
// Returns true when activation condition is met this call (sets s->active)
bool BrainZone_ShouldActivate(BrainZoneState* s, const SensorProcessor* sp);

// Returns true when exit condition is met this call (clears s->active)
bool BrainZone_ShouldExit(BrainZoneState* s, const SensorProcessor* sp);

// Utility: check if current filtered intensities look like uniform black field
bool BrainZone_IsUniformBlack(const SensorProcessor* sp);

// Compute white-line position on black background using calibration
// Returns true if a valid white position is computed
bool BrainZone_ComputeWhitePosition(const uint16_t* raw,
                                    const CalibrationData* cal,
                                    float* position,
                                    float* strength);

#ifdef __cplusplus
}
#endif

#endif // BRAIN_ZONE_H
