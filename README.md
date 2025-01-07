# Dual Motor Control System with PWM and Ultrasonic Sensor

This project demonstrates a dual-motor control system using PWM (Pulse Width Modulation) for speed control and an ultrasonic sensor for obstacle detection. The code is written in C and uses the `wiringPi` and `ncurses` libraries for GPIO control and user interaction.

## Features

- **Dual Motor Control**: Forward, backward, left, and right movement.
- **PWM Control**: Adjustable motor speed using duty cycle control.
- **Obstacle Detection**: Stops the motor when an obstacle is detected within a threshold distance using an ultrasonic sensor.
- **User Input**: Real-time control of motor direction and speed via keyboard input.
- **Safe Termination**: Graceful shutdown of motors and cleanup of resources.

## Requirements

- Raspberry Pi with `wiringPi` library installed
- Compatible motor driver (e.g., L298N)
- Dual motors
- Ultrasonic sensor (e.g., HC-SR04)
- `ncurses` library installed for terminal-based UI

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Rio-The-Yash/Eng.-Project-3.git
   cd Eng.-Project-3
   ```

2. Install the required libraries:
   ```bash
   sudo apt-get install libncurses5-dev wiringpi
   ```

3. Compile the code:
   ```bash
   rm -rf runme && gcc pwm_dual_motor.c -o runme -lncurses -lwiringPi
   ```

## Wiring Configuration

### Motor Pins:
- **Left Motor**:
  - Forward: GPIO 0 (Pin 11)
  - Backward: GPIO 1 (Pin 12)
  - PWM: GPIO 2 (Pin 13)

- **Right Motor**:
  - Forward: GPIO 21 (Pin 29)
  - Backward: GPIO 22 (Pin 31)
  - PWM: GPIO 23 (Pin 33)

### Ultrasonic Sensor Pins:
- Trigger: GPIO 15 (Pin 14)
- Echo: GPIO 16 (Pin 15)

Refer to your Raspberry Pi's GPIO pinout diagram to ensure correct connections.

## Usage

1. Run the program:
   ```bash
   ./runme
   ```

2. Use the following keyboard controls to operate the motors:
   - **`w`**: Move forward
   - **`s`**: Move backward
   - **`a`**: Turn left
   - **`d`**: Turn right
   - **`m`**: Set 90% duty cycle
   - **`n`**: Set 65% duty cycle
   - **`q`**: Stop the motors
   - **`x`**: Terminate the program

## Functional Overview

1. **PWM Control**:
   - A software-based PWM is implemented using `SIGALRM` signals, enabling adjustable motor speed.

2. **Obstacle Detection**:
   - The ultrasonic sensor continuously measures distance. If an obstacle is detected within the set threshold (50 cm), the motors stop automatically.

3. **Real-time User Interaction**:
   - The program uses `ncurses` to display status messages and accept user inputs.
