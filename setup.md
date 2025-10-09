I'll explain the complete process from owning the robot to running it, including detailed timer setup and the differences between the two main files.

## **🤖 Complete Robot Setup Process**

### **Step 1: Hardware Assembly**

#### **Required Components:**
- STM32F103C8 microcontroller (Blue Pill)
- H-bridge motor driver (L293D or L298N)
- 2x DC motors with wheels
- 10x line following sensors (IR sensors)
- Chassis/frame
- Battery pack (7.4V-12V)
- Jumper wires and breadboard

#### **Wiring Connections:**
```
STM32F103C8    H-Bridge    Motors
PA4    ----->  IN1    ----->  Left Motor Direction
PA5    ----->  IN2    ----->  Right Motor Direction  
PA0    ----->  ENA    ----->  Left Motor PWM
PA1    ----->  ENB    ----->  Right Motor PWM

STM32F103C8    Sensors
PA2-PA7  ----->  Sensor 1-6 (ADC channels)
PB0-PB3  ----->  Sensor 7-10 (ADC channels)
```

### **Step 2: Software Development Environment**

#### **Required Software:**
1. **STM32CubeIDE** (free from STMicroelectronics)
2. **STM32CubeMX** (for pin configuration)
3. **STM32F1 HAL Library**

#### **Project Setup:**
1. Create new STM32 project in CubeIDE
2. Select STM32F103C8 microcontroller
3. Configure pins using CubeMX
4. Generate code with HAL library

### **Step 3: Timer Configuration (Detailed)**

Let me show you exactly how to configure the timer:


[1 tool called]

#### **Timer Configuration Steps:**

**Step 3.1: Understanding the Math**
```c
// STM32F103C8 runs at 72MHz
// We want 20kHz PWM frequency
// Calculation: 72MHz / Prescaler / Period = 20kHz
// 72,000,000 / 72 / 50 = 20,000 Hz = 20kHz

htim2.Init.Prescaler = 72 - 1;    // Divide by 72: 72MHz → 1MHz
htim2.Init.Period = 50 - 1;       // Count to 50: 1MHz → 20kHz
```

**Step 3.2: Using STM32CubeMX (Visual Method)**
1. Open STM32CubeMX
2. Select your STM32F103C8 project
3. Go to **Timers** → **TIM2**
4. Set **Clock Source**: Internal Clock
5. Set **Channel1**: PWM Generation CH1
6. Set **Channel2**: PWM Generation CH2
7. Set **Prescaler**: 71 (72-1)
8. Set **Counter Period**: 49 (50-1)
9. Generate code

**Step 3.3: Manual Configuration (Code Method)**
```c
// In your main.c, add this function:
void MX_TIM2_Init(void) {
    // Copy the entire MX_TIM2_Init function from timer_config_example.c
    // This configures TIM2 for 20kHz PWM
}
```

### **Step 4: ADC Configuration for Sensors**

#### **ADC Setup:**
```c
// Configure ADC for 10 sensors
void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 10;  // 10 sensors
    
    // Configure each sensor channel
    for(int i = 0; i < 10; i++) {
        sConfig.Channel = ADC_CHANNEL_0 + i;
        sConfig.Rank = i + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    }
}
```

### **Step 5: Code Integration**

#### **Step 5.1: Copy Files to Project**
1. Copy `motor_control.h` to your project
2. Copy `enhanced_line_follower_fixed.c` content to your `main.c`
3. Copy timer configuration to your project

#### **Step 5.2: Update motor_control.h**
```c
// Uncomment the timer you're using in motor_control.h:
// For TIM2:
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_pwm);

// Comment out other timers
```

### **Step 6: Testing and Calibration**

#### **Step 6.1: Basic Motor Test**
```c
int main(void) {
    // Initialize everything
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    
    // Test motors
    move_forward(0.5f);  // 50% speed forward
    HAL_Delay(1000);
    stop_motors();
    
    sharp_turn_left(0.3f);  // Sharp left turn
    HAL_Delay(500);
    stop_motors();
}
```

#### **Step 6.2: Sensor Calibration**
1. Place robot on white surface
2. Run calibration (LED will blink)
3. Move to black line
4. Continue calibration
5. Move to red line
6. Complete calibration

## **📊 Differences Between Two Main Files**

Let me compare the two main files:


[2 tools called]

### **🔍 Detailed Comparison:**

| Aspect | `enhanced_line_follower.c` (OLD) | `enhanced_line_follower_fixed.c` (NEW) |
|--------|-----------------------------------|----------------------------------------|
| **Microcontroller** | STM32F4xx (F4 series) | STM32F1xx (F1 series - F103C8) |
| **Motor Control** | Basic PWM only | H-bridge bidirectional control |
| **Turning Logic** | ❌ Wrong (speed variation only) | ✅ Correct (direction + speed) |
| **Motor Functions** | Built-in functions | Modular motor_control.h |
| **H-Bridge Support** | ❌ No direction control | ✅ GPIO direction control |
| **Code Organization** | Monolithic | Modular design |
| **Hardware Compatibility** | F4 series only | F103C8 optimized |

### **🔧 Key Technical Differences:**

#### **1. Header File Changes:**
```c
// OLD:
#include "stm32f4xx_hal.h"

// NEW:
#include "stm32f1xx_hal.h"  // STM32F103C8 compatible
#include "motor_control.h"  // New motor control module
```

#### **2. Motor Control Implementation:**
```c
// OLD (WRONG):
void turn_left(float speed, float turn_factor) {
    set_motor_pwm(speed * (1.0f - turn_factor), speed);
    // This only varies speed, doesn't change direction
}

// NEW (CORRECT):
void turn_left(float speed, float turn_factor) {
    set_motor_pwm(-speed * turn_factor, speed);
    // Left motor backward, Right motor forward
}
```

#### **3. Recovery Behavior:**
```c
// OLD:
apply_motor_control(0.2f, -0.2f);  // Wrong for H-bridge

// NEW:
sharp_turn_left(0.2f);  // Correct H-bridge turning
```

#### **4. Function Organization:**
```c
// OLD: All functions in main file
void set_motor_pwm(float left, float right) { /* implementation */ }
void apply_motor_control(float left, float right) { /* implementation */ }

// NEW: Modular approach
#include "motor_control.h"  // All motor functions in separate module
```

## **🚀 Complete Running Process**

### **Step 7: Final Integration**

#### **7.1: Project Structure**
```
Your_Project/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── motor_control.h
│   │   ├── calibration.h
│   │   ├── sensor_processing.h
│   │   ├── line_following.h
│   │   ├── curved_section.h
│   │   └── black_box_nav.h
│   └── Src/
│       ├── main.c (enhanced_line_follower_fixed.c content)
│       ├── timer_config.c (from timer_config_example.c)
│       └── stm32f1xx_hal_msp.c
```

#### **7.2: Build and Flash**
1. **Build Project**: Ctrl+B in STM32CubeIDE
2. **Connect ST-Link**: USB to STM32F103C8
3. **Flash Code**: Right-click → Run As → STM32 C/C++ Application
4. **Reset Robot**: Press reset button

#### **7.3: Testing Sequence**
```c
// 1. Basic movement test
move_forward(0.3f);
HAL_Delay(2000);
stop_motors();

// 2. Turning test
sharp_turn_left(0.4f);
HAL_Delay(1000);
sharp_turn_right(0.4f);
HAL_Delay(1000);
stop_motors();

// 3. Line following test
// Place robot on line and run main control loop
```

### **Step 8: Troubleshooting**

#### **Common Issues and Solutions:**

| Problem | Cause | Solution |
|---------|-------|----------|
| Motors not moving | Timer not started | Add `HAL_TIM_PWM_Start()` |
| Wrong direction | H-bridge wiring | Check IN1/IN2 connections |
| Jittery movement | Low PWM frequency | Increase to 20kHz |
| Robot doesn't follow line | Sensor calibration | Recalibrate sensors |
| Compilation errors | Wrong HAL library | Use STM32F1xx HAL |

### **Step 9: Optimization**

#### **9.1: Tune Parameters**
```c
// Adjust these values in zone_params[]:
[ZONE_STRAIGHT] = {0.8f, 0.04f, 0.7f, 0.5f, 0.8f},
//                Kp    Kd    base  turn  max
//                gain  gain  speed speed steer
```

#### **9.2: Add Debugging**
```c
// Add UART for debugging
void debug_print(const char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}
```

This complete process will get your robot from hardware assembly to running line-following code with proper H-bridge motor control!