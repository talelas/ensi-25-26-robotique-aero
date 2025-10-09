// zigzag_zone.h
#ifndef ZIGZAG_ZONE_H
#define ZIGZAG_ZONE_H

#include "stm32f1xx_hal.h"
#include "sensor_processing.h"
#include <math.h>
#include <string.h>

#ifndef CONTROL_HZ
#define CONTROL_HZ 200.0f
#endif
#ifndef DT
#define DT (1.0f/CONTROL_HZ)
#endif

// Heuristic: detect zigzag using monotonic intensity ramp across sensors
// (increase from left->right or right->left) rather than "both closing to center" like hex.

typedef struct {
    // PD gains for tracking between turns
    float kp;
    float kd;
    // Speeds
    float base_speed;   // cruise speed between turns
    float turn_speed;   // speed while executing big turn
    // Detection thresholds
    float strength_min; // minimum confidence for valid evaluation
    float ramp_enter;   // |ramp_score| to enter zigzag
    float ramp_exit;    // |ramp_score| to exit zigzag (hysteresis < enter)
    float edge_strength;// strength to consider we're at a line edge (full coverage)
    uint16_t enter_frames; // frames required to enter
    uint16_t exit_frames;  // frames required to exit
    uint16_t edge_frames;  // frames to confirm an edge
    // Turn behavior
    uint32_t turn_ms;   // milliseconds to apply strong turn bias
    float steer_bias;   // additional bias during turn phase (0..1)
    float d_limit;      // derivative clamp
} ZigzagParams;

static const ZigzagParams ZIGZAG_DEFAULT = {
    .kp = 1.0f,
    .kd = 0.20f,
    .base_speed = 0.50f,
    .turn_speed = 0.35f,
    .strength_min = 0.25f,
    .ramp_enter = 0.18f,
    .ramp_exit  = 0.12f,
    .edge_strength = 0.85f,
    .enter_frames = 5,
    .exit_frames  = 6,
    .edge_frames  = 3,
    .turn_ms = 250,
    .steer_bias = 0.60f,
    .d_limit = 8.0f
};

typedef struct {
    // Filters and memory
    float last_error;
    float d_filt;
    // Activation
    uint8_t active;
    uint16_t enter_count;
    uint16_t exit_count;
    // Edge detection
    uint16_t edge_count;
    // Turn handling
    int8_t turn_dir;           // -1 = left, +1 = right, 0 = none
    uint32_t turn_start_ms;    // when current turn started
    uint8_t turns_made;        // optional counter
    // Diagnostics
    float last_ramp;           // last computed ramp score
} ZigzagState;

static inline float zz_clipf(float v, float lo, float hi){ return (v<lo)?lo:((v>hi)?hi:v); }

// Compute a left->right ramp score in [-1..+1]: positive means increasing to the right
static inline float zz_ramp_score(const SensorProcessor* proc){
    // Use filtered intensities for stability
    const float* a = proc->filtered;
    float sum = 0.f, wsum = 0.f;
    // weights: linear ramp from -1..+1 matching sensor indices
    for (int i=0;i<NUM_SENSORS;i++){
        float w = -1.0f + 2.0f * ((float)i / (float)(NUM_SENSORS-1));
        float v = a[i];
        wsum += w * v;
        sum  += v;
    }
    if (sum < 1e-6f) return 0.f;
    float score = wsum / sum; // already ~[-1..+1]
    return zz_clipf(score, -1.f, 1.f);
}

static inline void init_zigzag(ZigzagState* s){
    memset(s, 0, sizeof(*s));
    s->last_error = 0.f;
    s->d_filt = 0.f;
    s->active = 0;
    s->turn_dir = 0;
}

// Entry/exit detection using ramp hysteresis and strength
static inline void zigzag_update_activation(ZigzagState* s, const SensorProcessor* proc, const ZigzagParams* p){
    const float S = proc->last_strength;
    if (S < p->strength_min){
        s->enter_count = 0; // not enough confidence
        // allow exit to accumulate if already active
    }
    const float ramp = zz_ramp_score(proc);
    s->last_ramp = ramp;

    if (!s->active){
        if (S >= p->strength_min && fabsf(ramp) >= p->ramp_enter){
            if (++s->enter_count >= p->enter_frames){ s->active = 1; s->enter_count = 0; s->exit_count = 0; }
        } else {
            s->enter_count = 0;
        }
    } else {
        if (fabsf(ramp) <= p->ramp_exit){
            if (++s->exit_count >= p->exit_frames){ s->active = 0; s->exit_count = 0; s->enter_count = 0; }
        } else {
            s->exit_count = 0;
        }
    }
}

// Edge detection: treat near-full coverage (high strength) as an edge; hold few frames
static inline uint8_t zigzag_detect_edge(ZigzagState* s, const SensorProcessor* proc, const ZigzagParams* p){
    if (proc->last_strength >= p->edge_strength){
        if (++s->edge_count >= p->edge_frames) { s->edge_count = 0; return 1; }
    } else {
        s->edge_count = 0;
    }
    return 0;
}

// Decide turn direction at an edge using the sign of the ramp (trend across sensors)
static inline int8_t zigzag_decide_turn(const ZigzagState* s){
    // If ramp > 0 (increasing to right), next big turn likely to the right (+1)
    // If ramp < 0, turn left (-1)
    return (s->last_ramp >= 0.f) ? +1 : -1;
}

static inline void process_zigzag(ZigzagState* s,
                                  const ZigzagParams* p,
                                  const SensorProcessor* proc,
                                  float* left_speed,
                                  float* right_speed){
    const uint32_t now = HAL_GetTick();

    // Update activation based on ramp and strength
    zigzag_update_activation(s, proc, p);

    // If not active, fall back to neutral output (caller can run default PD)
    if (!s->active){
        float v = p->base_speed;
        v = zz_clipf(v, 0.f, 1.f);
        *left_speed = v; *right_speed = v; // neutral; caller can override
        s->last_error = proc->last_position;
        return;
    }

    // Detect edge and possibly start a turn phase
    if (zigzag_detect_edge(s, proc, p)){
        s->turn_dir = zigzag_decide_turn(s);
        s->turn_start_ms = now;
        if (s->turns_made < 255) s->turns_made++;
    }

    // Compute PD around line while not in the strong turn window
    float e = proc->last_position; // [-1..+1]
    float d_raw = (e - s->last_error) / DT;
    s->d_filt = 0.7f * s->d_filt + 0.3f * d_raw;
    s->d_filt = zz_clipf(s->d_filt, -p->d_limit, p->d_limit);
    float u = p->kp * e + p->kd * s->d_filt;
    u = zz_clipf(u, -1.f, 1.f);

    // Speed profile & turning bias
    float v;
    uint8_t in_turn = (s->turn_dir != 0) && ((now - s->turn_start_ms) < p->turn_ms);
    if (in_turn){
        v = p->turn_speed;
        // Add strong bias to steer into the new segment
        float bias = p->steer_bias * (float)s->turn_dir; // +/-
        float L = v - u + (-bias);
        float R = v + u + (+bias);
        *left_speed = zz_clipf(L, 0.f, 1.f);
        *right_speed = zz_clipf(R, 0.f, 1.f);
    } else {
        v = p->base_speed;
        float L = v - u;
        float R = v + u;
        *left_speed = zz_clipf(L, 0.f, 1.f);
        *right_speed = zz_clipf(R, 0.f, 1.f);
        s->turn_dir = 0; // clear after window
    }

    s->last_error = e;
}

#endif // ZIGZAG_ZONE_H
