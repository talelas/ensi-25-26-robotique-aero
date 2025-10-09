#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "calibration.h"

// Filtering and detection thresholds
// Adaptive EMA: use BASE for normal, FAST for rapid changes (lower alpha = faster response)
#define FILTER_ALPHA_BASE    0.60f  // prior weight for normal updates
#define FILTER_ALPHA_FAST    0.40f  // prior weight when a rapid jump is detected
#define FILTER_JUMP_THRESH   0.20f  // if |instant-filtered| > this, use FAST
#define MIN_SIGNAL_STRENGTH  0.10f  // Minimum average intensity to be valid
#define RED_THRESHOLD        0.40f  // Avg red similarity to flag red zone

// Normalized sensor weights [-1..+1], left -> right (defined in .c)
extern const float SENSOR_WEIGHTS[NUM_SENSORS];

typedef struct {
    float filtered[NUM_SENSORS]; // EMA-filtered line intensities [0..1]
    float instant[NUM_SENSORS];  // Latest unfiltered intensities [0..1]
    float last_position;         // Latest line position [-1..+1]
    float last_strength;         // Latest signal strength [0..1]
    bool  red_detected;          // Heuristic red-zone flag
} SensorProcessor;

// Initialize processor state (zeros filters; clears flags)
void init_sensor_processor(SensorProcessor* proc);

// Compute line position/strength from raw ADCs and calibration.
// Returns true if strength >= MIN_SIGNAL_STRENGTH (valid reading).
// Always updates proc->last_strength; updates last_position only if valid.
bool compute_line_position(const uint16_t* raw,
                           const CalibrationData* cal,
                           SensorProcessor* proc,
                           float* position,
                           float* strength);

// Compute position using whiteness similarity (for white line on black background)
bool sp_compute_position_white(const uint16_t* raw,
                               const CalibrationData* cal,
                               float* position,
                               float* strength);

// Compute position using redness similarity (for following red line)
bool sp_compute_position_red(const uint16_t* raw,
                             const CalibrationData* cal,
                             float* position,
                             float* strength);

// Per-sensor color similarity helpers (0..1: higher = better match)
float sp_whiteness(uint16_t raw, const CalibrationData* cal, uint8_t index);
float sp_blackness(uint16_t raw, const CalibrationData* cal, uint8_t index);
float sp_redness(uint16_t raw, const CalibrationData* cal, uint8_t index);

// Accessors for diagnostics/logic
static inline void sp_get_filtered(const SensorProcessor* p, float out[NUM_SENSORS]) {
    if (!p || !out) return; memcpy(out, p->filtered, sizeof(p->filtered));
}
static inline void sp_get_instant(const SensorProcessor* p, float out[NUM_SENSORS]) {
    if (!p || !out) return; memcpy(out, p->instant, sizeof(p->instant));
}
static inline float sp_get_strength(const SensorProcessor* p) { return p ? p->last_strength : 0.0f; }
static inline float sp_get_loss(const SensorProcessor* p) { return p ? (1.0f - p->last_strength) : 1.0f; }

// ---------------- Compatibility shims (temporary) ----------------
// Legacy helper expected by existing modules; will be removed later.
static inline float compute_position(SensorProcessor* proc,
                                     const uint16_t* raw,
                                     const CalibrationData* cal) {
    float pos = proc ? proc->last_position : 0.0f;
    float str = 0.0f;
    (void)compute_line_position(raw, cal, proc, &pos, &str);
    return pos;
}

static inline float get_signal_strength(const SensorProcessor* proc) {
    return proc ? proc->last_strength : 0.0f;
}

static inline bool is_red_detected(const SensorProcessor* proc) {
    return proc ? proc->red_detected : false;
}

#endif // SENSOR_PROCESSING_H