#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include <math.h>

/* Motor Control Configuration for STM32F103C8 with H-Bridge */
#define PWM_FREQUENCY 20000      // 20kHz PWM frequency
#define PWM_RESOLUTION 1000      // 0-1000 for fine control
#define MOTOR_MAX_SPEED 1.0f     // Maximum motor speed (0.0 to 1.0)

/* H-Bridge Direction Control Pins */
#define LEFT_MOTOR_DIR_PIN GPIO_PIN_4
#define LEFT_MOTOR_DIR_PORT GPIOA
#define RIGHT_MOTOR_DIR_PIN GPIO_PIN_5
#define RIGHT_MOTOR_DIR_PORT GPIOA

/* PWM Timer Configuration */
// Choose your timer (TIM2, TIM3, or TIM4)
// TIM2: PA0 (CH1), PA1 (CH2), PA2 (CH3), PA3 (CH4)
// TIM3: PA6 (CH1), PA7 (CH2), PB0 (CH3), PB1 (CH4)
// TIM4: PB6 (CH1), PB7 (CH2), PB8 (CH3), PB9 (CH4)

/* Motor Control Functions */
void init_motor_control(void);
void set_motor_pwm(float left, float right);
void apply_motor_control(float left, float right);

/* Basic Movement Functions */
void stop_motors(void);
void move_forward(float speed);
void move_backward(float speed);

/* Turning Functions for H-Bridge */
void turn_left(float speed, float turn_factor);
void turn_right(float speed, float turn_factor);
void sharp_turn_left(float speed);
void sharp_turn_right(float speed);

/* Advanced Movement Functions */
void differential_drive(float left_speed, float right_speed);
void pivot_left(float speed);
void pivot_right(float speed);

/* Utility Functions */
float constrain_motor_speed(float speed);
void set_motor_direction(bool left_forward, bool right_forward);

/* H-Bridge Motor Control Implementation */
void set_motor_pwm(float left, float right) {
    // Constrain motor speeds to valid range (-1.0 to +1.0 for H-bridge)
    left = constrain_motor_speed(left);
    right = constrain_motor_speed(right);
    
    // Convert to PWM values for STM32F103C8
    uint32_t left_pwm = (uint32_t)(fabsf(left) * PWM_RESOLUTION);
    uint32_t right_pwm = (uint32_t)(fabsf(right) * PWM_RESOLUTION);
    
    // H-Bridge Direction Control
    set_motor_direction(left >= 0, right >= 0);
    
    // PWM Control - Replace with your actual timer configuration
    // Example for TIM2:
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);   // Left motor PWM
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_pwm);  // Right motor PWM
    
    // Example for TIM3:
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);
    
    // Example for TIM4:
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, left_pwm);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, right_pwm);
}

void init_motor_control(void) {
    // Initialize GPIO pins for H-bridge direction control
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure direction control pins
    GPIO_InitStruct.Pin = LEFT_MOTOR_DIR_PIN | RIGHT_MOTOR_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Initialize motors to stop
    stop_motors();
}

void apply_motor_control(float left, float right) {
    set_motor_pwm(left, right);
}

void stop_motors(void) {
    set_motor_pwm(0.0f, 0.0f);
}

void move_forward(float speed) {
    speed = constrain_motor_speed(speed);
    set_motor_pwm(speed, speed);
}

void move_backward(float speed) {
    speed = constrain_motor_speed(speed);
    set_motor_pwm(-speed, -speed);
}

void turn_left(float speed, float turn_factor) {
    // Left motor: backward, Right motor: forward
    speed = constrain_motor_speed(speed);
    turn_factor = constrain(turn_factor, 0.0f, 1.0f);
    set_motor_pwm(-speed * turn_factor, speed);
}

void turn_right(float speed, float turn_factor) {
    // Left motor: forward, Right motor: backward
    speed = constrain_motor_speed(speed);
    turn_factor = constrain(turn_factor, 0.0f, 1.0f);
    set_motor_pwm(speed, -speed * turn_factor);
}

void sharp_turn_left(float speed) {
    // Left motor: backward, Right motor: forward
    speed = constrain_motor_speed(speed);
    set_motor_pwm(-speed, speed);
}

void sharp_turn_right(float speed) {
    // Left motor: forward, Right motor: backward
    speed = constrain_motor_speed(speed);
    set_motor_pwm(speed, -speed);
}

void differential_drive(float left_speed, float right_speed) {
    set_motor_pwm(left_speed, right_speed);
}

void pivot_left(float speed) {
    // Pivot around left wheel (left motor stopped, right motor forward)
    speed = constrain_motor_speed(speed);
    set_motor_pwm(0.0f, speed);
}

void pivot_right(float speed) {
    // Pivot around right wheel (right motor stopped, left motor forward)
    speed = constrain_motor_speed(speed);
    set_motor_pwm(speed, 0.0f);
}

float constrain_motor_speed(float speed) {
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

void set_motor_direction(bool left_forward, bool right_forward) {
    // Left motor direction: 0=forward, 1=backward
    HAL_GPIO_WritePin(LEFT_MOTOR_DIR_PORT, LEFT_MOTOR_DIR_PIN, 
                      left_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    
    // Right motor direction: 0=forward, 1=backward
    HAL_GPIO_WritePin(RIGHT_MOTOR_DIR_PORT, RIGHT_MOTOR_DIR_PIN, 
                      right_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Utility function for constraining values */
float constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

#endif // MOTOR_CONTROL_H
