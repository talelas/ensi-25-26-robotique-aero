/* Line Follower Robot - Core Implementation
 * Enhanced version with zone detection and adaptive control
 * Updated for STM32F103C8 with H-Bridge motor control
 */

#include "stm32f1xx_hal.h"  // Changed from stm32f4xx_hal.h for STM32F103C8
#include <string.h>
#include <math.h>

/* ====== Configuration ====== */
#include "calibration.h"
#include "motor_control.h"  // New motor control module
#include "checkerboard_zone.h"
#include "zigzag_zone.h"
#include "hexagon_zone.h"
#include "color_zone.h"
#include "brain_zone.h"

#define HISTORY_SIZE     5    // For pattern detection
#define CONTROL_HZ     200.0f // Control loop frequency
#define DT           (1.0f/CONTROL_HZ)
#define CENTER_INDEX ((NUM_SENSORS-1)/2.0f)

/* Initial zone parameters */
#define BLACK_BOX_SPEED      0.4f    // 40% speed in black box
#define EDGE_THRESHOLD       0.2f    // Threshold for detecting white
#define CONFIDENCE_THRESHOLD 0.8f    // Required confidence for zone change
#define STABLE_COUNTS       10      // Number of stable readings needed

/* Zone definitions */
typedef enum {
    ZONE_START,    // Initial black box
    ZONE_STRAIGHT, // Simple line following
    ZONE_SINUOUS,  // Wavy/curved sections
    ZONE_HEXAGON,  // Hexagonal pattern
    ZONE_CHECKER,  // Checkerboard pattern
    ZONE_COLOR,    // Color-based zone
    ZONE_ZIGZAG,   // Final zigzag pattern
    ZONE_FINAL,    // End zone
    ZONE_LOST      // Recovery mode
} ZoneState;

// Per-zone PD is handled locally; no global ZoneParams table used anymore.

/* Zone-specific parameters */
    // Default steering clamp used by local PDs (no zone params)
    #define DEFAULT_MAX_STEER 0.8f

#include "sensor_processing.h"
#include "black_box_nav.h"

/* Global state */
volatile uint16_t adc_raw[NUM_SENSORS];
SensorProcessor sensor_proc;
ZoneState current_zone = ZONE_START;
CalibrationManager calibration_mgr;
bool is_calibrated = false;
float last_error = 0.0f;
float error_sum = 0.0f;
int zone_entry_time = 0;
int pattern_counter = 0;

/* Initialize sensor processor on startup */
void init_system(void) {
    init_sensor_processor(&sensor_proc);
    init_calibration(&calibration_mgr);
    init_motor_control();  // Initialize motor control
}

/* Function prototypes */
void process_sensors(void);
void update_line_position(void);
void detect_patterns(void);
ZoneState update_zone_state(void);
// calculate_control removed
// Removed generic calculate_control; each zone/module computes its own PD.
void handle_recovery(void);
void handle_zone_transition(ZoneState new_zone);

/* Pattern detection helpers - to be implemented later */

#include "curved_section.h"

/* Global state additions */
static BlackBoxState black_box_state;
static bool black_box_initialized = false;
// Legacy line_following removed; we use local straight PD instead
static CurvedSectionState curve_state;
static CurveParams curve_params = DEFAULT_CURVE_PARAMS;
static bool curve_initialized = false;
static CheckerboardState checker_state;
static CheckerboardParams checker_params = CHECKERBOARD_DEFAULT;
static ZigzagState zigzag_state;
static ZigzagParams zigzag_params = ZIGZAG_DEFAULT;
static bool zigzag_initialized = false;

static HexagonZoneState hexagon_state;

static ColorZoneState color_zone_state;
static BrainZoneState brain_state;
static uint16_t brain_post_black_frames = 0; // counter for final-stop condition

// Local straight PD constants and state
#define STRAIGHT_KP           0.8f
#define STRAIGHT_KD           0.4f
#define STRAIGHT_BASE_SPEED   0.6f
#define STRAIGHT_TURN_SPEED   0.4f
#define STRAIGHT_CURVE_DTH    0.30f
static float straight_last_error = 0.0f;
static float straight_derivative = 0.0f;

/* Main function */
int main(void) {
    // Initialize system
    init_system();
    init_checkerboard(&checker_state);
    init_zigzag(&zigzag_state);
    zigzag_initialized = true;
    HexagonZone_Init(&hexagon_state);
    ColorZone_Init(&color_zone_state);
    BrainZone_Init(&brain_state);
    
    // Main control loop
    while(1) {
        control_step();
        HAL_Delay(5); // 200Hz control loop
    }
}

/* Main control loop */
void control_step(void) {
    // Always update sensor processing first
    float pos = 0.0f, str = 0.0f;
    (void)compute_line_position(adc_raw, &calibration_mgr.data, &sensor_proc, &pos, &str);

    // 1. Handle calibration if not complete
    if (!is_calibrated) {
        calibration_step(&calibration_mgr, adc_raw);
        
        // If calibration is complete, initialize normal operation
        if (calibration_mgr.data.calibration_complete) {
            is_calibrated = true;
            current_zone = ZONE_START;
            init_black_box_nav(&black_box_state);  // Initialize black box navigation
            black_box_initialized = true;
            return;
        }
        
        // During calibration, keep motors stopped
        stop_motors();
        return;
    }
    
    // 2. Handle black box navigation
    if (current_zone == ZONE_START) {
        float left_speed, right_speed;
        
        // Process black box navigation
        if (process_black_box(&black_box_state, adc_raw, &calibration_mgr.data, 
                            &left_speed, &right_speed)) {
            // Transition detected, initialize line following
            current_zone = ZONE_STRAIGHT;
        } else {
            // Still in black box, apply calculated speeds
            differential_drive(left_speed, right_speed);
            return;  // Skip normal control loop while in black box
        }
    }
    
    // 3. Handle straight line following
    if (current_zone == ZONE_STRAIGHT) {
        float left_speed, right_speed;

        // Brain (black field) detection: continuous uniform black => invert logic (follow white)
        // Do NOT change zone; keep ZONE_STRAIGHT so hexagon right-choice continues to run.
        (void)BrainZone_ShouldActivate(&brain_state, &sensor_proc);

            // Color zone detection (red has absolute priority)
            if (ColorZone_Run(&color_zone_state, &sensor_proc)) {
                current_zone = ZONE_COLOR;
                // Follow the red line using dedicated PD on redness-derived position
                float red_pos = 0.f, red_strength = 0.f;
                if (ColorZone_ComputeRedPosition(adc_raw, &calibration_mgr.data, &red_pos, &red_strength)) {
                    float error = red_pos; // already in [-1..1]
                    // Use straight gains for color following (can be specialized later)
                    float control = STRAIGHT_KP * error + STRAIGHT_KD * ((error - last_error) / DT);
                    control = constrain(control, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
                    float base_speed = (fabsf(error) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
                    float L = base_speed - control;
                    float R = base_speed + control;
                    differential_drive(L, R);
                    last_error = error;
                } else {
                    // Fallback: keep motors at safe crawl if red strength invalid
                    differential_drive(0.2f, 0.2f);
                }
                return;
            }

        // Check for zigzag entry first using ramp-based detector
        {
            float zl, zr;
            process_zigzag(&zigzag_state, &zigzag_params, &sensor_proc, &zl, &zr);
            if (zigzag_state.active) {
                current_zone = ZONE_ZIGZAG;
                differential_drive(zl, zr);
                return;
            }
        }

        // Check for checkerboard entry using confidence-based detector
        if (checkerboard_should_activate(&checker_state, &sensor_proc, &checker_params)) {
            current_zone = ZONE_CHECKER;
            float L, R;
            process_checkerboard(&checker_state, &checker_params, &sensor_proc, &L, &R);
            differential_drive(L, R);
            return;
        }
        
        // If brain zone is active, invert logic: follow white on black background
        if (brain_state.active) {
            float wpos=0.f, wstr=0.f;
            if (BrainZone_ComputeWhitePosition(adc_raw, &calibration_mgr.data, &wpos, &wstr)) {
                // Use straight zone params but slightly lower base speed for stability on white
                // Use straight gains with slightly lower base speed for stability on white
                float base_speed = STRAIGHT_BASE_SPEED * 0.8f;
                float error = wpos; // white position in [-1..1]
                float control = STRAIGHT_KP * error + STRAIGHT_KD * ((error - last_error) / DT);
                control = constrain(control, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
                left_speed = base_speed - control;
                right_speed = base_speed + control;
                // Prefer right at intersections: increase bias if intersection detected
                if (HexagonZone_Run(&hexagon_state, &sensor_proc)) {
                    right_speed += 0.10f;
                } else {
                    right_speed += 0.05f;
                }
                differential_drive(left_speed, right_speed);
                last_error = error;
                return;
            }
        }

        // Hexagon zone detection and right-path selection (only when not in brain white-follow)
        if (HexagonZone_Run(&hexagon_state, &sensor_proc)) {
            // Bias steering to right path with a gentle speed skew
            left_speed = 0.35f;
            right_speed = 0.55f;
            differential_drive(left_speed, right_speed);
            return;
        }
        
        // Local straight PD control on black line (fixed DT)
        {
            float error = sensor_proc.last_position;
            float d_raw = (error - straight_last_error) / DT;
            straight_derivative = 0.7f * straight_derivative + 0.3f * d_raw;
            float control = STRAIGHT_KP * error + STRAIGHT_KD * straight_derivative;
            control = constrain(control, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
            float base_speed = (fabsf(error) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
            left_speed = base_speed - control;
            right_speed = base_speed + control;
            straight_last_error = error;
        }
        
        // Check for curve transition
        // Curve transition using derivative magnitude
        {
            // Increment curve detection counter
            static uint8_t curve_detect_count = 0;
            if (fabsf(straight_derivative) > STRAIGHT_CURVE_DTH) {
                if (curve_detect_count < 255) curve_detect_count++;
            } else {
                curve_detect_count = 0;
            }
            
            // Require multiple detections for stable transition
            if (curve_detect_count > 5) {
                current_zone = ZONE_SINUOUS;
                curve_detect_count = 0;
                if (!curve_initialized) {
                    init_curved_section(&curve_state);
                    curve_initialized = true;
                }
            }
        }
        
        // Apply motor speeds
        differential_drive(left_speed, right_speed);
        return;  // Skip rest of control loop while in straight line mode
    }
    
    // 4. Handle curved/sinuous sections
    if (current_zone == ZONE_SINUOUS) {
        float left_speed, right_speed;
        
        // Process curved section
        process_curved_section(&curve_state, &sensor_proc,
                             &curve_params, &left_speed, &right_speed);
        
        // Apply motor speeds
        differential_drive(left_speed, right_speed);
        
        // Exit to straight when curvature settles for several frames
        {
            static uint8_t curve_exit_count = 0;
            // Severity proxy: average absolute derivative across history
            float severity = fabsf(curve_state.cumulative_derivative) / (float)CURVE_HISTORY_SIZE;
            if (severity < 0.15f && sensor_proc.last_strength > 0.35f) {
                if (curve_exit_count < 255) curve_exit_count++;
            } else {
                curve_exit_count = 0;
            }
            if (curve_exit_count > 6) { // ~30ms at 200Hz
                current_zone = ZONE_STRAIGHT;
                // Resume straight PD control immediately
                float e2 = sensor_proc.last_position;
                float d2 = (e2 - straight_last_error) / DT;
                straight_derivative = 0.7f * straight_derivative + 0.3f * d2;
                float u2 = STRAIGHT_KP * e2 + STRAIGHT_KD * straight_derivative;
                u2 = constrain(u2, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
                float bs2 = (fabsf(e2) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
                float l2 = bs2 - u2;
                float r2 = bs2 + u2;
                differential_drive(l2, r2);
                return;
            }
        }
        
        // TODO: Add transition detection to next zone (hexagon)
        
        return;  // Skip rest of control loop while in curved section
    }

    // 5. Handle zigzag zone
    if (current_zone == ZONE_ZIGZAG) {
        float L, R;
        process_zigzag(&zigzag_state, &zigzag_params, &sensor_proc, &L, &R);
        if (!zigzag_state.active) {
            // Exit: run normal line following immediately
            // Resume straight PD control
            float e2 = sensor_proc.last_position;
            float d2 = (e2 - straight_last_error) / DT;
            straight_derivative = 0.7f * straight_derivative + 0.3f * d2;
            float u2 = STRAIGHT_KP * e2 + STRAIGHT_KD * straight_derivative;
            u2 = constrain(u2, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
            float bs2 = (fabsf(e2) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
            float l2 = bs2 - u2;
            float r2 = bs2 + u2;
            current_zone = ZONE_STRAIGHT;
            differential_drive(l2, r2);
            return;
        }
        differential_drive(L, R);
        return;
    }

    // 5. Handle checkerboard zone
    if (current_zone == ZONE_CHECKER) {
        // Exit condition: confidence recovered; return to straight line following
        if (checkerboard_should_exit(&checker_state, &sensor_proc, &checker_params)) {
            current_zone = ZONE_STRAIGHT;
            // Resume straight PD control
            float e2 = sensor_proc.last_position;
            float d2 = (e2 - straight_last_error) / DT;
            straight_derivative = 0.7f * straight_derivative + 0.3f * d2;
            float u2 = STRAIGHT_KP * e2 + STRAIGHT_KD * straight_derivative;
            u2 = constrain(u2, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
            float bs2 = (fabsf(e2) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
            float l2 = bs2 - u2;
            float r2 = bs2 + u2;
            differential_drive(l2, r2);
            return;
        }
        float L, R;
        process_checkerboard(&checker_state, &checker_params, &sensor_proc, &L, &R);
        differential_drive(L, R);
        return;
    }

    // Brain zone exit handling anywhere in loop: if active and exit condition holds, clear and resume straight control
    if (brain_state.active && BrainZone_ShouldExit(&brain_state, &sensor_proc)) {
        current_zone = ZONE_STRAIGHT;
        brain_post_black_frames = 0; // reset counter on exit
    }

    // Final stop: after exiting brain, if continuous uniform black persists for a short window, stop motors
    if (!brain_state.active && current_zone == ZONE_STRAIGHT) {
        if (BrainZone_IsUniformBlack(&sensor_proc)) {
            if (brain_post_black_frames < 0xFFFF) brain_post_black_frames++;
            if (brain_post_black_frames >= BRAIN_FINAL_BLACK_FRAMES) {
                stop_motors();
                return;
            }
        } else {
            brain_post_black_frames = 0;
        }
    }
    
    // 2. Process sensor data
    process_sensors();
    update_line_position();
    
    // 3. Detect patterns and update zone
    detect_patterns();
    ZoneState new_zone = update_zone_state();
    
    // 3. Handle zone transition if needed
    if (new_zone != current_zone) {
        handle_zone_transition(new_zone);
    }
    
    // 6. Calculate and apply control (default PD path)
        // If in color zone, continue following the red line using redness-derived position
        if (current_zone == ZONE_COLOR && color_zone_state.active) {
            float red_pos = 0.f, red_strength = 0.f;
            if (ColorZone_ComputeRedPosition(adc_raw, &calibration_mgr.data, &red_pos, &red_strength)) {
                float error = red_pos;
                float control = STRAIGHT_KP * error + STRAIGHT_KD * ((error - last_error) / DT);
                control = constrain(control, -DEFAULT_MAX_STEER, DEFAULT_MAX_STEER);
                float base_speed = (fabsf(error) > 1.0f) ? STRAIGHT_TURN_SPEED : STRAIGHT_BASE_SPEED;
                float L = base_speed - control;
                float R = base_speed + control;
                differential_drive(L, R);
                last_error = error;
            } else {
                differential_drive(0.2f, 0.2f);
            }
        } else {
            // No generic PD fallback: each zone handles its own control above.
        }
    
    // 5. Recovery if needed
    if (sensor_proc.last_strength < 0.2f) {
        handle_recovery();
    }
}

/* Sensor processing using sensor processing module */
void process_sensors(void) {
    // Use sensor processing module to compute position
    float position = compute_position(&sensor_proc, adc_raw, &calibration_mgr.data);
    
    // Update zone based on red detection if needed
    if (is_red_detected(&sensor_proc) && current_zone != ZONE_COLOR) {
        current_zone = ZONE_COLOR;
    }
}

/* Pattern detection */
void detect_patterns(void) {
    static float last_pos = 0;
    static int pattern_count = 0;
    
    // Get current position from sensor processor
    float current_pos = sensor_proc.last_position;
    
    // Calculate position change
    float pos_change = fabsf(current_pos - last_pos);
    
    // Detect rapid changes (possible pattern)
    if (pos_change > 1.5f) {
        pattern_count++;
    } else {
        pattern_count = 0;
    }
    
    // Pattern specific detection - to be implemented later
    // TODO: Add pattern detection logic for each zone
    
    last_pos = current_pos;
}

/* Zone state machine update */
ZoneState update_zone_state(void) {
    static int stable_count = 0;
    ZoneState suggested_zone = current_zone;
    
    // Pattern-based zone detection - to be implemented later
    // TODO: Add zone detection logic based on sensor patterns
    
    // Apply hysteresis to prevent rapid switching
    if (suggested_zone != current_zone) {
        stable_count++;
        if (stable_count > 5) { // Require 5 consistent readings
            stable_count = 0;
            return suggested_zone;
        }
    } else {
        stable_count = 0;
    }
    
    return current_zone;
}

/* Core PD control calculation removed; each zone handles its own PD. */

/* Recovery behavior */
void handle_recovery(void) {
    static int recovery_phase = 0;
    static int recovery_timer = 0;
    
    // Get signal strength from sensor processor
    float confidence = sensor_proc.last_strength;
    
    switch (recovery_phase) {
        case 0: // Initial search
            // Search in last known direction - use sharp turn for H-bridge
            sharp_turn_left(0.2f);
            if (confidence > 0.3f) {
                recovery_phase = 0;
                return;
            }
            if (++recovery_timer > 100) {
                recovery_phase = 1;
                recovery_timer = 0;
            }
            break;
            
        case 1: // Wider search
            // Implement spiral search pattern
            float t = recovery_timer * 0.01f;
            float search_speed = 0.3f;
            differential_drive(
                search_speed * cosf(t),
                search_speed * sinf(t)
            );
            if (confidence > 0.3f) {
                recovery_phase = 0;
                return;
            }
            if (++recovery_timer > 200) {
                recovery_phase = 2;
            }
            break;
            
        case 2: // Give up
            stop_motors();
            break;
    }
}

/* Zone transition handling */
void handle_zone_transition(ZoneState new_zone) {
    // Handle zone-specific initialization
    switch(new_zone) {
        case ZONE_HEXAGON:
            // Initialize hexagon-specific parameters
            pattern_counter = 0;
            break;
        case ZONE_CHECKER:
            // Initialize checker-specific parameters
            pattern_counter = 0;
            break;
        case ZONE_ZIGZAG:
            // Initialize zigzag-specific parameters
            pattern_counter = 0;
            break;
        case ZONE_COLOR:
            // Initialize color-specific parameters
            break;
        default:
            break;
    }
    zone_entry_time = HAL_GetTick();
}

/* Utility functions */
float normalize_sensor(uint16_t raw) {
    const uint16_t min_val = 100;
    const uint16_t max_val = 3000;
    float norm = (float)(raw - min_val) / (float)(max_val - min_val);
    return constrain(norm, 0.0f, 1.0f);
}

float constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* Pattern detection functions - to be implemented later as needed */
