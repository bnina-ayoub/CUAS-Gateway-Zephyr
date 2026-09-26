# C-UAS Actuation Node: Zephyr RTOS

This repository contains the Zephyr RTOS application serving as the physical execution layer for the C-UAS turret. It translates velocity targets received via serial communication into precise PWM signals to drive the pan and tilt stepper motors.

## Core Architecture

- **Interrupt-Driven UART**: The node listens on `flexcomm0_lpuart0` for an 8-byte payload representing a `joints_t` struct (containing two floats: `pan_velocity` and `tilt_velocity`).
- **State Indication**: Successfully reading and unpacking a complete UART packet toggles a green LED (`led1`) to visually indicate active telemetry reception.
- **PWM Motor Control**: The firmware converts the target velocities into hardware-specific PWM periods (STEP) and GPIO logic levels (DIR) to drive external stepper motor controllers. A deadband is applied to stop the motors if the requested velocity falls below 0.001.

## Kinematics & PWM Conversion Formulas

The conversion from rotational velocity to PWM step frequency relies on mechanical gear ratios and a base velocity multiplier (254.7 Hz per unit of velocity).

1. **Gear Ratios**:
   - Pan mechanism: 4.0 (80/20 teeth)
   - Tilt mechanism: 3.0 (60/20 teeth)

2. **Frequency Calculation**: The absolute velocity is multiplied by the gear ratio and the base multiplier to determine the required step frequency:

   $$f_{step} = |v| \times 254.7 \times \text{Gear Ratio}$$

3. **Hardware Period Allocation**: The frequency is converted into a nanosecond period for the PWM peripheral:

   $$T_{base} = \frac{10^9}{f_{step}}$$

   The firmware applies a 1.5× scaling factor to this base period to dictate the final hardware period ($T_{pwm}$).

4. **Duty Cycle**: The pulse width is locked at 50% of the scaled period ($T_{pwm} / 2$) to ensure clean rising and falling edges for the stepper drivers.

## Known Issue

As a quick code-review catch: in `set_tilt_velocity`, the tilt frequency is currently being calculated using `PAN_VEL_TO_HZ_MULTIPLIER` instead of `TILT_VEL_TO_HZ_MULTIPLIER`. This should be fixed before finalizing the repository so the tilt kinematics are accurate.
