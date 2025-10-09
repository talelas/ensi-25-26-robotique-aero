// checkerboard_zone.h
#ifndef CHECKERBOARD_ZONE_H
#define CHECKERBOARD_ZONE_H

#include "stm32f1xx_hal.h"
#include "calibration.h"
#include "sensor_processing.h"
#include <math.h>
#include <string.h>

#ifndef CONTROL_HZ
#define CONTROL_HZ 200.0f
#endif
#ifndef DT
#define DT (1.0f/CONTROL_HZ)
#endif

typedef struct {
    float kp;
    float kd;
    float base_speed;
    float min_speed;
    float max_speed;
    float speed_recover_ms;
    float confidence_enter;
    float confidence_exit;
    float gap_hold_decay;
    float d_limit;
    uint16_t enter_frames;
    uint16_t exit_frames;
} CheckerboardParams;

typedef struct {
    float last_error;
    float d_filt;
    float last_good_error;
    float current_speed;
    uint16_t enter_count;
    uint16_t exit_count;
    uint32_t straight_since_ms;
    uint8_t active;
} CheckerboardState;

static inline float cb_clipf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static const CheckerboardParams CHECKERBOARD_DEFAULT = {
    .kp = 1.0f,
    .kd = 0.10f,
    .base_speed = 0.45f,
    .min_speed = 0.30f,
    .max_speed = 0.65f,
    .speed_recover_ms = 500.0f,
    .confidence_enter = 0.45f,
    .confidence_exit  = 0.55f,
    .gap_hold_decay = 0.92f,
    .d_limit = 8.0f,
    .enter_frames = 4,
    .exit_frames  = 5
};

static inline void init_checkerboard(CheckerboardState* s) {
    memset(s, 0, sizeof(*s));
    s->last_error = 0.0f;
    s->last_good_error = 0.0f;
    s->d_filt = 0.0f;
    s->current_speed = CHECKERBOARD_DEFAULT.base_speed;
    s->straight_since_ms = HAL_GetTick();
    s->active = 0;
}

static inline void checkerboard_enter_logic(CheckerboardState* s,
                                            const SensorProcessor* proc,
                                            const CheckerboardParams* p) {
    float conf = proc->last_strength;
    if (conf < p->confidence_enter) {
        s->enter_count++;
        if (s->enter_count >= p->enter_frames) {
            s->active = 1;
            s->enter_count = 0;
            s->exit_count = 0;
        }
    } else {
        s->enter_count = 0;
    }
}

static inline void checkerboard_exit_logic(CheckerboardState* s,
                                           const SensorProcessor* proc,
                                           const CheckerboardParams* p) {
    float conf = proc->last_strength;
    if (conf > p->confidence_exit) {
        s->exit_count++;
        if (s->exit_count >= p->exit_frames) {
            s->active = 0;
            s->exit_count = 0;
            s->enter_count = 0;
        }
    } else {
        s->exit_count = 0;
    }
}

static inline uint8_t checkerboard_should_activate(CheckerboardState* s,
                                                   const SensorProcessor* proc,
                                                   const CheckerboardParams* p) {
    checkerboard_enter_logic(s, proc, p);
    return s->active;
}

static inline uint8_t checkerboard_should_exit(CheckerboardState* s,
                                               const SensorProcessor* proc,
                                               const CheckerboardParams* p) {
    checkerboard_exit_logic(s, proc, p);
    return (s->active == 0);
}

static inline void process_checkerboard(CheckerboardState* s,
                                        const CheckerboardParams* p,
                                        const SensorProcessor* proc,
                                        float* left_speed,
                                        float* right_speed) {
    const uint32_t now = HAL_GetTick();
    const float conf = proc->last_strength;
    const float pos = proc->last_position; // assumed [-1..+1]

    float e;
    if (conf < p->confidence_enter) {
        s->last_good_error = p->gap_hold_decay * s->last_good_error;
        e = s->last_good_error;
    } else {
        e = pos; // PD tuned for this scale
        s->last_good_error = e;
        s->straight_since_ms = now;
    }

    const float d_raw = (e - s->last_error) / DT;
    s->d_filt = 0.7f * s->d_filt + 0.3f * d_raw;
    s->d_filt = cb_clipf(s->d_filt, -p->d_limit, p->d_limit);

    float u = p->kp * e + p->kd * s->d_filt;
    u = cb_clipf(u, -1.0f, 1.0f);

    float v;
    if (conf < p->confidence_enter) {
        v = p->min_speed;
    } else {
        const float t_ms = (float)(now - s->straight_since_ms);
        const float k = cb_clipf(t_ms / p->speed_recover_ms, 0.0f, 1.0f);
        v = p->min_speed + k * (p->base_speed - p->min_speed);
    }
    v = cb_clipf(v, p->min_speed, p->max_speed);

    float L = v - u;
    float R = v + u;

    L = cb_clipf(L, 0.0f, 1.0f);
    R = cb_clipf(R, 0.0f, 1.0f);

    *left_speed  = L;
    *right_speed = R;

    s->last_error = e;
}

#endif // CHECKERBOARD_ZONE_H

