#include "brain_zone.h"
#include <math.h>

static inline float clamp01f(float v){ return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

void BrainZone_Init(BrainZoneState* s){ if(!s) return; s->active=false; s->enter_count=0; s->exit_count=0; }

static void summarize_filtered(const SensorProcessor* sp, float* avg, float* minv, float* maxv){
    float t=0.f, mn=1.f, mx=0.f;
    for(int i=0;i<NUM_SENSORS;i++){
        float v = sp->filtered[i];
        t += v; if(v<mn) mn=v; if(v>mx) mx=v;
    }
    if(avg) *avg = t / (float)NUM_SENSORS; if(minv) *minv = mn; if(maxv) *maxv = mx;
}

bool BrainZone_IsUniformBlack(const SensorProcessor* sp){
    if(!sp) return false;
    float avg, mn, mx; summarize_filtered(sp, &avg, &mn, &mx);
    return (avg >= BRAIN_AVG_BLACK_MIN) && ((mx - mn) <= BRAIN_UNIFORM_EPS);
}

bool BrainZone_ShouldActivate(BrainZoneState* s, const SensorProcessor* sp){
    if(!s || !sp) return false;
    bool uniform_black = BrainZone_IsUniformBlack(sp);
    if(uniform_black){
        if(s->enter_count < 255) s->enter_count++;
        s->exit_count = 0;
    } else {
        s->enter_count = 0;
    }
    if(!s->active && s->enter_count >= BRAIN_ENTER_EPOCHS){ s->active = true; s->enter_count = 0; return true; }
    return false;
}

bool BrainZone_ShouldExit(BrainZoneState* s, const SensorProcessor* sp){
    if(!s || !sp) return false;
    float avg, mn, mx; summarize_filtered(sp, &avg, &mn, &mx);
    bool not_uniform = ((mx - mn) > (BRAIN_UNIFORM_EPS * 1.1f)) || (avg < (BRAIN_AVG_BLACK_MIN - 0.05f));
    if(not_uniform){ if(s->exit_count < 255) s->exit_count++; } else { s->exit_count = 0; }
    if(s->active && s->exit_count >= BRAIN_EXIT_EPOCHS){ s->active = false; s->exit_count = 0; return true; }
    return false;
}

bool BrainZone_ComputeWhitePosition(const uint16_t* raw,
                                    const CalibrationData* cal,
                                    float* position,
                                    float* strength){
    return sp_compute_position_white(raw, cal, position, strength);
}
