# Autonomous Underwater Vehicle (AUV) Project

## Overview

This Arduino-based project controls an Autonomous Underwater Vehicle (AUV) using an Arduino board, servos for thruster control, and sensors such as the MS5837 pressure sensor and MKRIMU accelerometer.

The AUV can be programmed to perform various movements based on user-defined behaviors and can also respond to the orientation detected by the accelerometer.

## Components

- Arduino board
- Servos for thruster control (four in total)
- MS5837 pressure sensor
- MKRIMU accelerometer
- LED for visual indication
- Leak detection sensor

## Setup

### Hardware Setup

1. Connect the servos to the specified pins (`servoPin`, `servoPin2`, `servoPin3`, `servoPin4`).
2. Connect the MS5837 pressure sensor to the Arduino.
3. Connect the MKRIMU accelerometer to the Arduino.
4. Connect the LED to the specified pin (`ledPin`).
5. Connect the leak detection sensor to the specified pin (`leakPin`).
6. Ensure that the hardware connections are correct and secure.

### Software Setup

1. Install the required libraries in the Arduino IDE:
   - Servo library
   - Wire library
   - MS5837 library
   - MKRIMU library

2. Copy and paste the provided Arduino code into the Arduino IDE.

3. Upload the code to the Arduino board.

## Usage

1. Power on the AUV and place it in water.

2. The AUV will perform initial movements based on the provided code.

3. The IMU (accelerometer) continuously checks the orientation. The AUV adjusts its behavior based on the tilt and inversion:
   - Tilting to the right: Turns right.
   - Tilting to the left: Turns left.
   - Upside down or inverted: Moves backward.
   - Level orientation: Stops.

4. Additional behaviors can be programmed using the provided functions like `moveForward()`, `moveBackward()`, `turnLeft()`, `turnRight()`, and `stopAUV()`.

## Additional Notes

- Customize the threshold values and behaviors in the `checkIMU()` function to suit your specific requirements.

- Modify the `AUVSpeeds()` function for different speed ranges based on the characteristics of your thrusters.

- Experiment with the provided functions and create new ones to implement desired AUV behaviors.

- Make sure to calibrate and test the AUV in a controlled environment, such as a pool, before deploying it in open water.

## Contributors

- Tyler Travis

Feel free to contribute to and improve this project. Happy exploring!
