/* Black Box Navigation Module */

#ifndef BLACK_BOX_NAV_H
#define BLACK_BOX_NAV_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "calibration.h"
#include "sensor_processing.h"

typedef struct {
    uint32_t start_time;         // Time when we entered black box mode
    uint32_t white_detect_time;  // Time when we first detected white (optional)
    bool white_detected_left;    // White detected on left edge (hysteresis applied)
    bool white_detected_right;   // White detected on right edge (hysteresis applied)
    uint8_t left_stable;         // consecutive frames above threshold on left
    uint8_t right_stable;        // consecutive frames above threshold on right
} BlackBoxState;

typedef struct {
    float   white_thr;      // whiteness threshold [0..1]
    uint8_t stable_frames;  // frames to confirm edge white
    uint32_t accel_ms;      // ramp-up duration
    float   speed;          // cruise speed in box [0..1]
    float   steer_k;        // slight bias using center sensors
} BlackBoxParams;

static const BlackBoxParams BLACK_BOX_DEFAULT = {
    .white_thr = 0.70f,
    .stable_frames = 10,
    .accel_ms = 1000,
    .speed = 0.40f,
    .steer_k = 0.10f
};

/* Initialize black box navigation state */
void init_black_box_nav(BlackBoxState *state) {
    state->start_time = HAL_GetTick();
    state->white_detect_time = 0;
    state->white_detected_left = false;
    state->white_detected_right = false;
    state->left_stable = 0;
    state->right_stable = 0;
}

/* Check if sensor indicates white (using calibrated values) */
// Whiteness helper averaging two-edge sensors for robustness
static inline float edge_whiteness_avg(const uint16_t* raw, const CalibrationData* cal, bool left_edge){
    if(left_edge){
        float w0 = sp_whiteness(raw[0], cal, 0);
        float w1 = sp_whiteness(raw[1], cal, 1);
        return 0.5f * (w0 + w1);
    } else {
        int n = NUM_SENSORS;
        float w8 = sp_whiteness(raw[n-2], cal, (uint8_t)(n-2));
        float w9 = sp_whiteness(raw[n-1], cal, (uint8_t)(n-1));
        return 0.5f * (w8 + w9);
    }
}

/* Process black box navigation and detect exit conditions */
bool process_black_box(BlackBoxState *state, const uint16_t *sensor_values, 
                      const CalibrationData *cal_data, float *left_speed, float *right_speed) {
    const BlackBoxParams* p = &BLACK_BOX_DEFAULT;

    // Edge whiteness with simple hysteresis via stable counters
    float wl = edge_whiteness_avg(sensor_values, cal_data, true);
    float wr = edge_whiteness_avg(sensor_values, cal_data, false);

    if (wl >= p->white_thr) { if (state->left_stable  < 255) state->left_stable++;  }
    else                    { state->left_stable = 0; }
    if (wr >= p->white_thr) { if (state->right_stable < 255) state->right_stable++; }
    else                    { state->right_stable = 0; }

    if (!state->white_detected_left && state->left_stable  >= p->stable_frames)  {
        state->white_detected_left = true;
        if (!state->white_detect_time) state->white_detect_time = HAL_GetTick();
    }
    if (!state->white_detected_right && state->right_stable >= p->stable_frames) {
        state->white_detected_right = true;
        if (!state->white_detect_time) state->white_detect_time = HAL_GetTick();
    }

    // Speed planning
    uint32_t elapsed = HAL_GetTick() - state->start_time;
    float ramp = (elapsed >= p->accel_ms) ? 1.0f : ((float)elapsed / (float)p->accel_ms);
    float base = p->speed * ramp;

    // Tiny center bias using middle two sensors (raw compare is OK here; it's minor)
    int cl = NUM_SENSORS/2 - 1;
    int cr = NUM_SENSORS/2;
    float bias = 0.0f;
    if (sensor_values[cl] > sensor_values[cr]) bias = -p->steer_k;
    else if (sensor_values[cr] > sensor_values[cl]) bias = p->steer_k;

    float L = base + (-bias);
    float R = base + (+bias);
    if (L < 0.f) L = 0.f; if (L > 1.f) L = 1.f;
    if (R < 0.f) R = 0.f; if (R > 1.f) R = 1.f;
    *left_speed = L; *right_speed = R;

    // Exit condition: both edges stably white
    if (state->white_detected_left && state->white_detected_right) {
        *left_speed *= 0.7f; *right_speed *= 0.7f; // graceful handoff
        return true;
    }
    return false;
}

#endif // BLACK_BOX_NAV_H