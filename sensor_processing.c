#include "sensor_processing.h"

static inline float clamp01(float v){ return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

// Normalized sensor weights [-1..+1], left -> right
const float SENSOR_WEIGHTS[NUM_SENSORS] = {
    -1.00f, -0.75f, -0.50f, -0.25f, -0.10f,
     0.10f,  0.25f,  0.50f,  0.75f,  1.00f
};

static float norm_white(uint16_t raw, uint16_t wmin, uint16_t wmax){
    const float den = (float)((wmax > wmin) ? (wmax - wmin) : 1);
    float x = ((float)raw - (float)wmin) / den;
    return clamp01(x);
}

// Public helpers for color similarities
float sp_whiteness(uint16_t raw, const CalibrationData* cal, uint8_t index){
    if(!cal || index>=NUM_SENSORS) return 0.f;
    return norm_white(raw, cal->white_min[index], cal->white_max[index]);
}

float sp_blackness(uint16_t raw, const CalibrationData* cal, uint8_t index){
    if(!cal || index>=NUM_SENSORS) return 0.f;
    // blackness as similarity to black calibration bounds
    const float den = (float)((cal->black_max[index] > cal->black_min[index]) ? (cal->black_max[index]-cal->black_min[index]) : 1);
    float x = ((float)raw - (float)cal->black_min[index]) / den; // 0 near black_min, 1 near black_max
    x = clamp01(x);
    // Consider center of black band as best match; map to a tent function
    float d = fabsf(x - 0.5f) / 0.5f; // 0 at center, 1 at edges
    float sim = 1.0f - clamp01(d);
    return sim;
}

float sp_redness(uint16_t raw, const CalibrationData* cal, uint8_t index){
    if(!cal || index>=NUM_SENSORS) return 0.f;
    const float den = (float)((cal->red_max[index] > cal->red_min[index]) ? (cal->red_max[index]-cal->red_min[index]) : 1);
    float x = ((float)raw - (float)cal->red_min[index]) / den; // 0..1 across red band
    x = clamp01(x);
    float d = fabsf(x - 0.5f) / 0.5f;
    float sim = 1.0f - clamp01(d);
    return sim;
}

void init_sensor_processor(SensorProcessor* p){
    if(!p) return;
    for (int i=0;i<NUM_SENSORS;i++) { p->filtered[i]=0.f; p->instant[i]=0.f; }
    p->last_position=0.f; p->last_strength=0.f; p->red_detected=false;
}

bool compute_line_position(const uint16_t* raw,
                           const CalibrationData* cal,
                           SensorProcessor* p,
                           float* position,
                           float* strength){
    if(!raw || !cal || !p) return false;

    float total = 0.f, weighted = 0.f;

    for (int i=0;i<NUM_SENSORS;i++){
        // Assume dark line on lighter background: use whiteness to invert to line intensity
        float white_n = norm_white(raw[i], cal->white_min[i], cal->white_max[i]);
        float intensity = 1.0f - white_n; // 0=white, 1=dark line
    p->instant[i] = clamp01(intensity);
    // Adaptive EMA filter: faster update on large jumps to improve responsiveness (checker/zigzag)
    float prev = p->filtered[i];
    float diff = fabsf(intensity - prev);
    float alpha = (diff > FILTER_JUMP_THRESH) ? FILTER_ALPHA_FAST : FILTER_ALPHA_BASE;
    p->filtered[i] = alpha * prev + (1.f - alpha) * intensity;
        total += p->filtered[i];
        weighted += p->filtered[i] * SENSOR_WEIGHTS[i];
    }

    float strength_v = clamp01(total / (float)NUM_SENSORS);
    p->last_strength = strength_v;

    bool valid = (strength_v >= MIN_SIGNAL_STRENGTH);
    if (valid){
        float denom = total + 1e-6f; // avoid div0
        float pos = weighted / denom; // already in [-1..1] range
        // clamp for safety
        if(pos < -1.f) pos = -1.f; else if(pos > 1.f) pos = 1.f;
        p->last_position = pos;
        if(position) *position = pos;
    } else {
        if(position) *position = p->last_position; // keep last
    }

    if(strength) *strength = strength_v;

    // Simple color heuristic: if average redness is high, flag red
    // Compute quick redness similarity from raw values
    float red_sum = 0.f;
    for(int i=0;i<NUM_SENSORS;i++) red_sum += sp_redness(raw[i], cal, (uint8_t)i);
    float red_avg = red_sum / (float)NUM_SENSORS;
    p->red_detected = (red_avg >= RED_THRESHOLD);

    return valid;
}

bool sp_compute_position_white(const uint16_t* raw,
                               const CalibrationData* cal,
                               float* position,
                               float* strength){
    if(!raw || !cal) return false;
    float total=0.f, weighted=0.f;
    for(int i=0;i<NUM_SENSORS;i++){
        float w = sp_whiteness(raw[i], cal, (uint8_t)i);
        total += w;
        weighted += w * SENSOR_WEIGHTS[i];
    }
    float str = clamp01(total / (float)NUM_SENSORS);
    if(strength) *strength = str;
    if(total > 1e-4f){
        float pos = weighted / total;
        if(pos < -1.f) pos = -1.f; else if(pos > 1.f) pos = 1.f;
        if(position) *position = pos;
        return true;
    }
    if(position) *position = 0.f;
    return false;
}

bool sp_compute_position_red(const uint16_t* raw,
                             const CalibrationData* cal,
                             float* position,
                             float* strength){
    if(!raw || !cal) return false;
    float total=0.f, weighted=0.f, red_sum=0.f;
    for(int i=0;i<NUM_SENSORS;i++){
        float r = sp_redness(raw[i], cal, (uint8_t)i);
        red_sum += r;
        total += r;
        weighted += r * SENSOR_WEIGHTS[i];
    }
    float avg = red_sum / (float)NUM_SENSORS;
    if(strength) *strength = avg;
    if(total > 1e-4f){
        float pos = weighted / total;
        if(pos < -1.f) pos = -1.f; else if(pos > 1.f) pos = 1.f;
        if(position) *position = pos;
        return true;
    }
    if(position) *position = 0.f;
    return false;
}
