#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"
#include <MKRIMU.h>

// Servo pin definitions
byte servoPin = 3;
byte servoPin2 = 10;
byte servoPin3 = 6;
byte servoPin4 = 9;

Servo servo;
Servo servo2;
Servo servo3;
Servo servo4;

// Pins for LED and leak detection
int ledPin = 13;
int leakPin = 2;
int leak = 0;

// Pins for thruster control
int FLPin = 3; // CHANGE PINS
int FRPin = 5; // CHANGE PINS
int BLPin = 6; // CHANGE PINS
int BRPin = 9; // CHANGE PINS

// Initial thrust values for each thruster
int FLThrust = 100;
int FRThrust = 100;
int BLThrust = 100;
int BRThrust = 100;

// MS5837 sensor and MKRIMU object instances
MS5837 sensor;
MKRIMU IMU;

/**
 * @brief Initializes the setup for the AUV, including servo attachment and sensor initialization.
 */
void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(leakPin, INPUT);

  Wire.begin();

  // Initialize MS5837 sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  // Initialize MKRIMU sensor
  while (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  // Set fluid density for the MS5837 sensor
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // Attach servos to respective pins
  servo.attach(servoPin);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  // Send stop signal to ESCs
  servo.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);

  delay(7000); // Delay to allow the ESCs to recognize the stopped signal

  int i = 0;

  // Delay to put the AUV in the water
  delay(720000);

  // Run the AUV for a short duration (for demonstration purposes)
  for (i = 0; i < 1; i++) {
    AUVSpeeds(1525, 1525, 1525, 1525);
    delay(10000);
    AUVSpeeds(1500, 1500, 1500, 1500);
  }
}

/**
 * @brief Main loop for the AUV control and behavior checks.
 */
void loop() {
  // Check IMU for orientation and adjust AUV behavior
  checkIMU();
  // Add other behaviors or tasks as needed
}

/**
 * @brief Sets AUV speeds based on specified values for each thruster.
 * @param leftV Speed for the left thruster.
 * @param rightV Speed for the right thruster.
 * @param backL Speed for the back left thruster.
 * @param backR Speed for the back right thruster.
 */
void AUVSpeeds(double leftV, double rightV, double backL, double backR) {
  // Check if speed values are within valid range
  if ((leftV < 1100 || leftV > 1900) || (rightV < 1100 || rightV > 1900) || (backL < 1100 || backL > 1900) || (backR < 1100 || backR > 1900)) {
    Serial.println("Invalid speed values");
  } else {
    // Set servo positions based on the specified speeds
    servo.writeMicroseconds(leftV);
    servo2.writeMicroseconds(rightV);
    servo3.writeMicroseconds(backL);
    servo4.writeMicroseconds(backR);
  }
}

/**
 * @brief Checks IMU data and adjusts AUV behavior based on orientation.
 */
void checkIMU() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print("Acceleration: ");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);

    // Adjust AUV behavior based on IMU data
    if (abs(x) > 1.5) {
      // Tilt to the right
      turnRight();
    } else if (abs(x) < -1.5) {
      // Tilt to the left
      turnLeft();
    } else if (abs(z) > 1.5) {
      // Upside down or inverted
      moveBackward();
    } else {
      // AUV is level
      stopAUV();
    }
  }
  delay(500);
}

/**
 * @brief Commands the AUV to move forward.
 */
void moveForward() {
  AUVSpeeds(1600, 1600, 1500, 1500);
}

/**
 * @brief Commands the AUV to move backward.
 */
void moveBackward() {
  AUVSpeeds(1400, 1400, 1500, 1500);
}

/**
 * @brief Commands the AUV to turn left.
 */
void turnLeft() {
  AUVSpeeds(1600, 1400, 1500, 1500);
}

/**
 * @brief Commands the AUV to turn right.
 */
void turnRight() {
  AUVSpeeds(1400, 1600, 1500, 1500);
}

/**
 * @brief Commands the AUV to stop.
 */
void stopAUV() {
  AUVSpeeds(1500, 1500, 1500, 1500);
}
