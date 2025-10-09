#ifndef CURVED_SECTION_H
#define CURVED_SECTION_H

#include "stm32f1xx_hal.h"
#include "calibration.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

#define CURVE_HISTORY_SIZE 10
#define CURVE_DETECT_THRESHOLD 0.3f
#define MIN_CURVE_SPEED 0.3f
#define MAX_CURVE_SPEED 0.5f
#define INTEGRAL_WINDUP_LIMIT 2.0f
#define RECOVERY_TIMEOUT_MS 1000
#define LINE_LOSS_THRESHOLD 0.15f  // Minimum total signal strength
#define DT (1.0f/200.0f)          // 200Hz control loop

typedef struct {
    float error_history[CURVE_HISTORY_SIZE];
    float derivative_history[CURVE_HISTORY_SIZE];
    float cumulative_derivative;
    float integral;
    float last_error;
    float last_valid_position;
    uint32_t line_lost_time;
    bool is_line_lost;
    bool initialized;
} CurvedSectionState;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float base_speed;
} CurveParams;

// Default parameters for curved sections
static const CurveParams DEFAULT_CURVE_PARAMS = {
    .Kp = 1.2f,
    .Ki = 0.1f,
    .Kd = 0.6f,
    .base_speed = 0.4f
};

static float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Initialize curved section handling
void init_curved_section(CurvedSectionState* state) {
    memset(state, 0, sizeof(CurvedSectionState));
    state->last_valid_position = 0.0f;
    state->is_line_lost = false;
    state->initialized = true;
}

// Compute total signal strength from sensors
static float compute_total_signal(const uint16_t* sensor_values, const CalibrationData* cal_data) {
    float total = 0.0f;
    for(int i = 0; i < NUM_SENSORS; i++) {
        float normalized = (float)(sensor_values[i] - cal_data->black_min[i]) / 
                         (float)(cal_data->white_max[i] - cal_data->black_min[i]);
        normalized = constrain(normalized, 0.0f, 1.0f);
        total += normalized;
    }
    return total / NUM_SENSORS;
}

// Helper function for line loss recovery
static float handle_line_loss(CurvedSectionState* state, uint32_t current_time) {
    uint32_t lost_duration = current_time - state->line_lost_time;
    
    if(lost_duration > RECOVERY_TIMEOUT_MS) {
        // Emergency stop if line not found after timeout
        return 0.0f;
    }

    // Progressive search pattern
    float search_amplitude = 0.5f * (1.0f + (float)lost_duration/RECOVERY_TIMEOUT_MS);
    float search_direction = (state->last_valid_position > 0) ? 1.0f : -1.0f;
    
    return search_amplitude * search_direction;
}

// Helper function to compute speed reduction factor based on curve severity
static float compute_speed_factor(float cumulative_derivative) {
    float curve_severity = fabs(cumulative_derivative) / CURVE_HISTORY_SIZE;
    float speed_factor = 1.0f - (curve_severity / CURVE_DETECT_THRESHOLD);
    return constrain(speed_factor, MIN_CURVE_SPEED/MAX_CURVE_SPEED, 1.0f);
}

// Main curved section processing function
void process_curved_section(CurvedSectionState* state, 
                          const SensorProcessor* proc,
                          const CurveParams* params,
                          float* left_speed,
                          float* right_speed) {
    if (!state->initialized) {
        init_curved_section(state);
    }

    float weighted_error = proc->last_position;  // Use unified position [-1..1]
    float derivative = (weighted_error - state->last_error) / DT;
    
    // Update histories
    for(int i = CURVE_HISTORY_SIZE-1; i > 0; i--) {
        state->error_history[i] = state->error_history[i-1];
        state->derivative_history[i] = state->derivative_history[i-1];
    }
    state->error_history[0] = weighted_error;
    state->derivative_history[0] = derivative;
    
    // Compute cumulative derivative for curve detection
    state->cumulative_derivative = 0;
    for(int i = 0; i < CURVE_HISTORY_SIZE; i++) {
        state->cumulative_derivative += state->derivative_history[i];
    }

    // Line loss detection
    float total_signal = proc->last_strength;
    if(total_signal < LINE_LOSS_THRESHOLD) {
        if(!state->is_line_lost) {
            state->is_line_lost = true;
            state->line_lost_time = HAL_GetTick();
        }
    } else {
        state->is_line_lost = false;
        state->last_valid_position = weighted_error;
    }

    // Compute control output
    float control_output;
    if(state->is_line_lost) {
        control_output = handle_line_loss(state, HAL_GetTick());
    } else {
        // Update integral with anti-windup
        state->integral += weighted_error * DT;
        state->integral = constrain(state->integral, -INTEGRAL_WINDUP_LIMIT, INTEGRAL_WINDUP_LIMIT);

        // PID control output
        control_output = params->Kp * weighted_error +
                        params->Ki * state->integral +
                        params->Kd * derivative;
    }

    // Compute speed factor based on curve severity
    float speed_factor = compute_speed_factor(state->cumulative_derivative);
    float current_speed = params->base_speed * speed_factor;

    // Apply control output to motor speeds
    *left_speed = current_speed - control_output;
    *right_speed = current_speed + control_output;

    // Constrain motor speeds
    *left_speed = constrain(*left_speed, 0.0f, 1.0f);
    *right_speed = constrain(*right_speed, 0.0f, 1.0f);

    state->last_error = weighted_error;
}

#endif // CURVED_SECTION_H