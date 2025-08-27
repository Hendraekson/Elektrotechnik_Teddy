# Interactive Teddy Bear - Microcontroller Project
## Project Description

This university project transforms a microcontroller into an interactive teddy bear that responds to physical interactions through expressive LED "eyes." The system uses green and blue LEDs to simulate the bear's eyes, which react dynamically to mutliple sensor inputs and user interactions.

### Concept

The teddy bear's personality is expressed through three distinct behaviors:
- **Affectionate Response** - When hugged (button press), the bear shows happiness
- **Uncomfortable Reaction** - When shaken or rotated too quickly, the bear becomes uneasy
- **Cookie Eating** - When a magnetic object (like a "cookie") is brought near, the bear reacts with delight

Each interaction triggers a unique LED pattern that brings the teddy bear's eyes to life, making it appear to have emotions and reactions just like a real companion.

## Technical Overview

This project implements an interactive microcontroller system that responds to sensor inputs and user interactions through LED feedback and behavioral routines. The green and blue LEDs act as the teddy bear's expressive eyes, changing brightness and patterns based on different sensory experiences.

## Hardware Peripherals Used

### 1. Timers

The brightness control of the green and blue LEDs is managed using timers TIM1 and TIM3 respectively. Using two separate timers provides the advantage of controlling both LEDs completely independently of each other. This capability is essential for the "uncomfortable" reaction, where using a single timer would significantly increase program complexity.

Since TIM1 and TIM3 perform the same function, they are configured identically:
- Both timers do not output their signal to a GPIO pin
- Each provides a PWM signal on channel 4 without output
- Timer TIM2 is used to control a custom delay function (`DelayTime(int)`)

#### Timer Configuration

| Control Parameter | TIM1 | TIM2 | TIM3 |
|-------------------|------|------|------|
| Prescaler         | 240  | 4800 | 240  |
| Auto-Reload       | 400  | *    | 400  |
| Capture-Compare   | 100  | *    | 100  |

*Dynamic values set in delay function

**Technical Specifications:**
- TIM1 and TIM3 count at 200 kHz frequency
- PWM signal frequency: 500 Hz
- Signal period: 2 ms
- All timers operate in interrupt mode
- TIM2 counts at 10 kHz frequency (due to prescaler of 4800)
- TIM2's Auto-Reload and Capture-Compare values are dynamically set in the delay function

### 2. Connectivity

Communication with the sensor board is handled via the I²C interface:
- **Mode:** Standard mode
- **Frequency:** 100 kHz  
- **GPIO Pins:** PB6 and PB7

## Program Logic

### 2.1 Initialization

The program begins by calling the initialization functions of:
1. HAL library
2. Used peripherals

Then the registers for Prescaler, Auto-reload, and Capture-Compare of timers TIM1, TIM2, and TIM3 are overwritten to match the values in the configuration table.

**Startup Sequence:**
1. Start timers TIM1 and TIM2
2. Start PWM signal generation
3. Declare and initialize I²C communication variables (data buffers, device and register addresses, sensor configuration data)
4. Configure sensors:
   - Activate rotation rate sensor (gyroscope)
   - Activate magnetometer sensor board
   - Activate magnetometer
5. Enter main loop

### 2.2 Main Loop

The main program loop performs the following operations:

1. **Sensor Communication Check:** Verify both sensor boards are ready for communication
2. **Gyroscope Reading:** 
   - Read gyroscope data
   - If Y-axis rotation rate exceeds threshold → call `BeUncomfortable()` function
3. **Magnetometer Reading:**
   - Read magnetometer data from all three axes
   - Calculate comparison value from the three magnetometer readings
   - If value exceeds threshold → call `EatCookie()` function
4. **Loop Delay:** Use `Delay()` function to pause execution for 0.1 seconds to prevent unnecessary I²C bus and sensor board load

### 2.3 Interrupt Handling

#### Button Interrupt
- **Trigger:** Blue button pressed on microcontroller (when no subroutine is running)
- **Action:** Calls `Hug()` function via GPIO pin interrupt

#### Subroutine Protection
All three subroutines (`BeUncomfortable()`, `EatCookie()`, `Hug()`):
- Disable blue button interrupts at start to prevent interruption
- Reset any stored interrupt flags upon completion
- Re-enable interrupts before finishing

#### Timer Interrupts
- **DelayTime Function:** Terminated by TIM2's `PeriodElapsedCallback` interrupt
- **LED Control:** LEDs are switched on/off by `PulseFinishedCallback` and `PeriodElapsedCallback` interrupts from TIM1 and TIM3

## System Behaviors

The system implements three main behavioral responses:

1. **BeUncomfortable()** - Triggered by excessive Y-axis rotation
2. **EatCookie()** - Triggered by magnetic field detection
3. **Hug()** - Triggered by button press
