#include <AFMotor.h>
#include <Wire.h>
#include <MPU6050.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor

MPU6050 mpu;

float initialYaw = 0.0;  // Store the initial yaw for reference
float yawThreshold = 1.0;  // Adjust this to define how sensitive the correction is

void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging
  Wire.begin();
  
  // Initialize the MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);  // Stop if the connection fails
  }

  // Set all motors to maximum speed initially
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  // Calibrate initial yaw
  initialYaw = getYaw();
  Serial.println("Car is calibrated. Moving forward...");
}

void loop() {
  // Move forward for 2 seconds
  goStraight();
  delay(2000);  // 2 second delay
  stopCar();    // Stop the car after moving forward
  delay(2000);  // 2 second delay


}

void goStraight() {
  float currentYaw = getYaw();  // Get current yaw from MPU6050

  if (currentYaw > initialYaw + yawThreshold) {
    correctLeft();  // Compensate if car turns right
  } else if (currentYaw < initialYaw - yawThreshold) {
    correctRight();  // Compensate if car turns left
  } else {
    forward();  // Go straight
  }
}

void forward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  Serial.println("Moving Forward");
}

void goBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Moving Backward");
}

void correctLeft() {
  motor1.setSpeed(255);  // Left motors normal speed
  motor2.setSpeed(200);  // Right motors slow down
  motor3.setSpeed(255);
  motor4.setSpeed(200);
  Serial.println("Correcting Left");
}

void correctRight() {
  motor1.setSpeed(200);  // Left motors slow down
  motor2.setSpeed(255);  // Right motors normal speed
  motor3.setSpeed(200);
  motor4.setSpeed(255);
  Serial.println("Correcting Right");
}

void stopCar() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  Serial.println("Car Stopped");
}

float getYaw() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  return gz / 131.0;
}
