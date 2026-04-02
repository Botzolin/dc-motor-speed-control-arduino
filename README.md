# DC Motor Speed Control (Arduino + L298N)

## Description
This project implements a DC motor speed control system using Arduino and an L298N motor driver.

The motor speed is measured using an IR sensor acting as an encoder, and a PI controller is used to regulate the speed by adjusting the PWM command.

## Features
- DC motor speed measurement using IR sensor
- PI controller for speed regulation
- PWM motor control with L298N
- Real-time RPM calculation
- Closed-loop control implementation

## Hardware Used
- Arduino
- L298N motor driver
- DC motor
- IR sensor for RPM measurement
- Power supply
- Knob Encoder

## Control Strategy
The controller compares the desired speed with the measured speed and adjusts the PWM signal applied to the motor.
A PI controller is used to reduce steady-state error and improve speed stability.

## Files
- `DC_motor_controller.ino` – Arduino source code

## Notes
Images and demo video will be added later.

## Author
Andrei Măcăneață-Vamoș
