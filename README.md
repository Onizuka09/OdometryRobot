# OdometryRobot

## Project Overview

**OdometryRobot** is a mobile robotics project focused on **precise motion control and odometry estimation** using a **bare-metal STM32 microcontroller**.  
The robot executes a sequence of high-level movement commands (distance and angle) and reconstructs its trajectory using encoder-based odometry and inertial feedback.

The system is designed with a **clear separation between low-level real-time control** (running on the STM32) and **high-level motion commands**.

---
## Example of Input Commands

The robot receives a sequence of movement instructions such as:

```text
Forward : 100 m
Right   : 90°
Forward : 100 m
Right   : 90°
Forward : 100 m
Left    : 90°
```
Using this sequence, the robot generates a trajectory and continuously estimates its position, orientation, and velocity through odometry.

## System Architecture

The system is divided into two main parts: Hardware and Control System.
### Hardware

Motors with encoders

  - Provide wheel rotation feedback.

  - Used for velocity and distance estimation.

### Microcontroller

STM32G0 as the main MCU.

  - Fully bare-metal implementation (no HAL, no RTOS).

  - Responsible for real-time motor control, odometry computation, and communication.

  - Avec cette série de mouvements, le robot crée un chemin. Le système est divisé en deux parties principales :

### STM32 Bare-Metal Software Design
All drivers and control modules are implemented from scratch in bare-metal C:

Low-Level Drivers

Encoder driver

Quadrature decoding using hardware timers.

Measures wheel speed and traveled distance.

UART driver

Bare-metal UART for command reception and data transmission.

Clock and system configuration

RCC, system clock, and peripheral configuration without HAL.

Motor control driver

PWM generation using timers.

Direction and speed control of DC motors.

Control Algorithms

Position PID

Distance PID controller.

Linear velocity PID controller.

Angular PID

Angle PID controller.

Angular velocity PI controller.

These controllers ensure accurate trajectory tracking and smooth motion.

Odometry Module

The odometry module computes:

Linear velocity

Angular velocity

Robot position (x, y)

Robot orientation (θ)

Using:

Wheel encoder data

Inertial measurements (accelerometer and gyroscope)

Control System

Closed-loop feedback control

Combines encoder feedback and IMU data.

Ensures accurate speed, distance, and angle tracking.

Trajectory execution

High-level commands (distance and angle) are converted into motor setpoints.

Real-time control loops run entirely on the STM32.

## PINOUTs 
### LED
- Internal LED  PA5 
### PONT-H 
- IN1 PB3  (D3)	
- IN2 PB5  (D4)	
- IN3 PB4  (D5)
- IN4 PB1  (A3)

- ENA Motor Left: PC1  PWM_timaer15_ch1   
- ENB Motor Right:  PC2  PWM_timer15_ch2  

left:  

- EconderA OUTA : PA8 (D7) tim1_ch1 (encoder mode ) 
- EncoderA OUTB : PA9 (D8) tim1_ch2 (encoder mode )

right: 

- EconderB OUTA : PC7 (D9) TIM3_ch2 (encoder mode )
- EncoderB OUTB : PC6 (x) TIM3_ch1 (encoder mode )

# UART3: UART Communication with ESP32 
- PB8 : UART3_Tx 
- PB9 : UART3_Rx 

# UART2: UART Communication
- PA2 : UART2_Tx 
- PA3 : UART2_Rx 
## Wiring  Encoders

- RED : motor+
- black: motor-
- green: GND
- blue: Vcc 
- Yellow: OUTA 
- White: OUTB 

## PWM Frequencies for Motor control 
Recommended PWM Frequencies for DC Motors
Motor Type	Ideal Frequency Range	Why?
Small Brushed DC (e.g., hobby motors)	5kHz - 20kHz	Above audible range, good torque control
Large Brushed DC	500Hz - 5kHz	Lower switching losses in power drivers
Coreless Motors	20kHz - 50kHz	Avoids coil resonance frequencies



## STM32G070-NUCLEO_PINOUTS
![image](./Docs/stm32g070-NUCLEO_PINOUTS.png)
