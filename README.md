# Bare-metal Heart Rate Monitoring System

## Overview
This project implements a bare-metal heart rate monitoring system on an STM32
microcontroller. The primary objective was to achieve reliable pulse detection
with predictable timing and low power consumption using a polling-based design
without an operating system.

The system was extended to include motion-based power management, safety
monitoring, and real-time user feedback, while still retaining a simple
bare-metal architecture.

## Hardware / Platform
- MCU: STM32 (Cortex-M based)
- Heart Rate Sensor: MAX30100 (Optical Pulse Oximeter)
- Motion Sensor: PIR
- Flame Sensor: Analog flame detector
- Display: OLED
- Actuators: Buzzer, Cooling Fan
- RTC: External RTC module

## Interfaces
- I2C – Heart rate sensor, RTC
- SPI – OLED display
- ADC – Flame sensor analog input
- GPIO / EXTI – PIR sensor, buzzer, fan control, user indicators
- PWM – Fan speed control

## Design Approach
- Super-loop (bare-metal) architecture
- Periodic sensor polling with deterministic timing
- Interrupts used only where strictly required
- Circular buffer for heart rate waveform storage
- No RTOS, no task scheduler, no dynamic memory allocation

## Power Optimization Strategy
- Motion-based power management using PIR sensor
- System enters low-power sleep mode if no motion is detected for 3 minutes
- MCU wakes up automatically on motion detection interrupt
- Reduction of sensor activity during idle periods
- Avoidance of unnecessary CPU active time

## Safety and Control Features
- Continuous flame detection using ADC-based sensing
- Audible buzzer alarm triggered on flame detection
- Cooling fan speed dynamically adjusted based on flame sensor analog value
- Real-time system status feedback via OLED display

## Timekeeping
- Real-time clock (RTC) used to maintain current date and time
- Time and date displayed on OLED for user reference
- RTC operation independent of main system execution

## Why Bare-metal?
This project was intentionally implemented without an RTOS to:
- Minimize software overhead
- Reduce latency and jitter
- Fully understand timing behavior of sensor sampling
- Gain fine-grained control over power and sleep states
- Strengthen low-level embedded system fundamentals

This approach mirrors early-stage firmware commonly used in
resource-constrained or ultra-low-power embedded devices.

## Limitations
- Limited scalability as system complexity increases
- Blocking delays can affect responsiveness
- Managing multiple concurrent features is difficult
- Logging and background tasks are not practical in this model

## Next Step
As system requirements expanded, this project was later migrated to FreeRTOS
to support multitasking and SD card data logging, while preserving the timing,
power, and interrupt-handling insights gained from the bare-metal implementation.
