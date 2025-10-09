#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define NUM_SENSORS 10
#define SAMPLES_PER_COLOR 2000  // 10 seconds * 200Hz
#define WAIT_SAMPLES 2000       // 10 seconds waiting time
#define LED_PIN GPIO_PIN_13    // Built-in LED pin, adjust as needed
#define LED_PORT GPIOC         // LED GPIO port, adjust as needed

typedef struct {
    uint16_t white_min[NUM_SENSORS];
    uint16_t white_max[NUM_SENSORS];
    uint16_t red_min[NUM_SENSORS];
    uint16_t red_max[NUM_SENSORS];
    uint16_t black_min[NUM_SENSORS];
    uint16_t black_max[NUM_SENSORS];
    bool calibration_complete;
} CalibrationData;

typedef enum {
    CAL_WHITE_COLLECT,
    CAL_WHITE_WAIT,
    CAL_RED_COLLECT,
    CAL_RED_WAIT,
    CAL_BLACK_COLLECT,
    CAL_COMPLETE
} CalibrationState;

typedef struct {
    CalibrationState state;
    uint32_t sample_counter;
    CalibrationData data;
} CalibrationManager;

// Forward declaration to avoid implicit function warning
void update_calibration_led(CalibrationState state);

// Initialize calibration data with safe defaults
void init_calibration(CalibrationManager *cal) {
    cal->state = CAL_WHITE_COLLECT;
    cal->sample_counter = 0;
    
    // Initialize with conservative values
    for (int i = 0; i < NUM_SENSORS; i++) {
        cal->data.white_min[i] = 0xFFFF;
        cal->data.white_max[i] = 0;
        cal->data.red_min[i] = 0xFFFF;
        cal->data.red_max[i] = 0;
        cal->data.black_min[i] = 0xFFFF;
        cal->data.black_max[i] = 0;
    }
    cal->data.calibration_complete = false;
}

// Update calibration values for current color
void update_calibration_values(uint16_t *min_vals, uint16_t *max_vals, const uint16_t *current_values) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (current_values[i] < min_vals[i]) {
            min_vals[i] = current_values[i];
        }
        if (current_values[i] > max_vals[i]) {
            max_vals[i] = current_values[i];
        }
    }
}

void update_calibration_led(CalibrationState state) {
    switch (state) {
        case CAL_WHITE_COLLECT:
            // Solid LED for white calibration
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            break;
            
        case CAL_WHITE_WAIT:
        case CAL_RED_WAIT:
            // Fast blink for waiting periods
            if ((HAL_GetTick() % 500) < 250)
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            break;
            
        case CAL_RED_COLLECT:
            // Double blink for red calibration
            if ((HAL_GetTick() % 1000) < 200 || 
                ((HAL_GetTick() % 1000) > 400 && (HAL_GetTick() % 1000) < 600))
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            break;
            
        case CAL_BLACK_COLLECT:
            // Slow blink for black calibration
            if ((HAL_GetTick() % 2000) < 1000)
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            break;
            
        case CAL_COMPLETE:
            // LED off when complete
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            break;
    }
}

// Main calibration step function
void calibration_step(CalibrationManager *cal, const uint16_t *sensor_values) {
    // Update LED pattern based on current state
    update_calibration_led(cal->state);
    switch (cal->state) {
        case CAL_WHITE_COLLECT:
            // Collecting white calibration data
            update_calibration_values(cal->data.white_min, cal->data.white_max, sensor_values);
            cal->sample_counter++;
            if (cal->sample_counter >= SAMPLES_PER_COLOR) {
                cal->state = CAL_WHITE_WAIT;
                cal->sample_counter = 0;
            }
            break;

        case CAL_WHITE_WAIT:
            // Waiting after white calibration
            cal->sample_counter++;
            if (cal->sample_counter >= WAIT_SAMPLES) {
                cal->state = CAL_RED_COLLECT;
                cal->sample_counter = 0;
            }
            break;

        case CAL_RED_COLLECT:
            // Collecting red calibration data
            update_calibration_values(cal->data.red_min, cal->data.red_max, sensor_values);
            cal->sample_counter++;
            if (cal->sample_counter >= SAMPLES_PER_COLOR) {
                cal->state = CAL_RED_WAIT;
                cal->sample_counter = 0;
            }
            break;

        case CAL_RED_WAIT:
            // Waiting after red calibration
            cal->sample_counter++;
            if (cal->sample_counter >= WAIT_SAMPLES) {
                cal->state = CAL_BLACK_COLLECT;
                cal->sample_counter = 0;
            }
            break;

        case CAL_BLACK_COLLECT:
            // Collecting black calibration data
            update_calibration_values(cal->data.black_min, cal->data.black_max, sensor_values);
            cal->sample_counter++;
            if (cal->sample_counter >= SAMPLES_PER_COLOR) {
                cal->state = CAL_COMPLETE;
                cal->data.calibration_complete = true;
            }
            break;

        case CAL_COMPLETE:
            // Calibration is complete, nothing to do
            break;
    }
}

// Update LED based on calibration state

#endif // CALIBRATION_H