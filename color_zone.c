#include "color_zone.h"
#include <stddef.h>

void ColorZone_Init(ColorZoneState *state) {
    if (state) {
        state->active = false;
    }
}

bool ColorZone_Run(ColorZoneState *state, const SensorProcessor *sp) {
    if (!state || !sp) return false;
    // Hysteresis: if red detected, activate immediately; otherwise require several non-red frames to exit
    enum { RED_EXIT_EPOCHS = 3 };
    static int nonred_count = 0;
    if (sp->red_detected) {
        state->active = true;
        nonred_count = 0;
        return true;
    } else {
        if (state->active) {
            if (nonred_count < RED_EXIT_EPOCHS) nonred_count++;
            if (nonred_count >= RED_EXIT_EPOCHS) {
                state->active = false;
                nonred_count = 0;
            }
        } else {
            nonred_count = 0;
        }
        return state->active;
    }
}

bool ColorZone_ComputeRedPosition(const uint16_t *raw,
                                  const CalibrationData *cal,
                                  float *position,
                                  float *strength) {
    return sp_compute_position_red(raw, cal, position, strength);
}
