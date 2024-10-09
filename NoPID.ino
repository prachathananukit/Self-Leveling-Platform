#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// -------------------- Pin Definitions --------------------
const int servoPin = 9; // Servo connected to PWM pin 9

// -------------------- Servo Object --------------------
Servo servo;

// -------------------- MPU-6050 Object --------------------
MPU6050 mpu;

// -------------------- Calibration Offsets --------------------
const int MPU6050_ADDR = 0x68; // MPU-6050 I2C address

// -------------------- Constants --------------------
const float rollSetpoint = 0.0; // Desired roll angle (level)
const int servoCenter = 90;     // Center position of the servo
const int servoRange = 90;      // Maximum deviation from center in degrees
const int maxServoChange = 3;   // Maximum change in servo position per loop

// Define the maximum G-force range for the accelerometer
const float maxGForce = 2.0;    // Maximum range of ±2g

// Threshold for servo movement (0 degrees)
const float movementThreshold = 0;

// Variable to track the previous servo position
int prevServoPos = servoCenter;

void setup() {
  Serial.begin(9600);

  // Initialize Servo
  servo.attach(servoPin);
  servo.write(servoCenter); // Center the servo initially

  // Initialize MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if MPU-6050 not found
  }
}

void loop() {
  // Read raw accelerometer data
  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, NULL, NULL, NULL);

  // Convert raw data to G-force (assuming AFS_SEL = 0 for ±2g range)
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // Constrain the G-force values to the maximum range of ±2g
  ay_g = constrain(ay_g, -maxGForce, maxGForce);
  az_g = constrain(az_g, -maxGForce, maxGForce);

  // Calculate roll angle from accelerometer data
  float rollAcc = atan2(ay_g, az_g) * 180.0 / PI;

  // Apply threshold filter
  if (abs(rollAcc - rollSetpoint) > movementThreshold) {
    // Calculate the new servo position based on roll angle
    int newServoPos = servoCenter - (int)rollAcc; // Subtracting to invert the control direction

    // Constrain the new position within the servo range
    newServoPos = constrain(newServoPos, servoCenter - servoRange, servoCenter + servoRange);

    // Limit the movement to a maximum of 3 degrees from the previous position
    if (abs(newServoPos - prevServoPos) > maxServoChange) {
      if (newServoPos > prevServoPos) {
        newServoPos = prevServoPos + maxServoChange; // Move up by 3 degrees max
      } else {
        newServoPos = prevServoPos - maxServoChange; // Move down by 3 degrees max
      }
    }

    // Write the new position to the servo
    servo.write(newServoPos);

    // Update the previous servo position for the next loop
    prevServoPos = newServoPos;
  }

  delay(10); // Increase loop delay to 10 ms (100 Hz)
}
