#include <Wire.h>
#include <AFMotor.h> // Include the Adafruit Motor Shield library

// Create motor objects
AF_DCMotor leftFrontMotor(1); // Motor 1
AF_DCMotor leftBackMotor(2);  // Motor 2
AF_DCMotor rightFrontMotor(3); // Motor 3
AF_DCMotor rightBackMotor(4);  // Motor 4

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; // Linear acceleration
float GyroX, GyroY, GyroZ; // Angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Used in loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; // Max PWM value
const int minSpeed = 160; // Min PWM value
float angle; // Due to how MPU6050 is oriented
float targetAngle = 0;
int equilibriumSpeed = 248; // Rough estimate of PWM for straight driving
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; // Is the car driving forward OR rotating/stationary
bool prevIsDriving = true; // Equals isDriving in the previous iteration of loop()
bool paused = false; // Is the program paused

void setup() {
  Serial.begin(9600);
  Wire.begin();                      // Initialize communication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050
  Wire.write(0x6B);                  // Talk to register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into register 6B
  Wire.endTransmission(true);        // End the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  currentTime = micros();
}

void loop() {
  // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per second, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  // Combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll; // For my orientation, angle = roll. Adjust based on your orientation
  
  // Print the values on the serial monitor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') { // Drive forward
      Serial.println("forward");
      isDriving = true;
    } else if (c == 'a') { // Turn left
      Serial.println("left");
      targetAngle += 90;
      if (targetAngle > 180) {
        targetAngle -= 360;
      }
      isDriving = false;
    } else if (c == 'd') { // Turn right
      Serial.println("right");
      targetAngle -= 90;
      if (targetAngle <= -180) {
        targetAngle += 360;
      }
      isDriving = false;
    } else if (c == 'q') { // Stop or brake
      Serial.println("stop");
      isDriving = false;
    } else if (c == 'i') { // Print information
      Serial.print("angle: ");
      Serial.println(angle);
      Serial.print("targetAngle: ");
      Serial.println(targetAngle);
      Serial.print("GyroX: ");
      Serial.println(GyroX);
      Serial.print("elapsedTime (in ms): "); // Estimates time to run loop() once
      Serial.println(elapsedTime * 1000);
      Serial.print("equilibriumSpeed: ");
      Serial.println(equilibriumSpeed);
    } else if (c == 'p') { // Pause the program
      paused = !paused;
      stopCar();
      isDriving = false;
      Serial.println("key p was pressed, which pauses/unpauses the program");
    }
  }

  static int count;
  static int countStraight;
  if (count < 6) {  
    count++;
  } else { // Runs once after loop() runs 7 times. Loop runs about every 2.8ms
    count = 0;
    if (!paused) {
      if (isDriving != prevIsDriving) {
        leftSpeedVal = equilibriumSpeed;
        countStraight = 0;
        Serial.print("mode changed, isDriving: ");
        Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3) {
          if (countStraight < 20) {
            countStraight++;
          } else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal; // To find equilibrium speed, 20 consecutive readings needed to indicate car is going straight
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } else {
          countStraight = 0;
        }
        driving();
      } else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }
}

void driving() { // Called by loop(), isDriving = true
  int deltaAngle = round(targetAngle - angle);
  forward();
  if (deltaAngle != 0) {
    controlSpeed();
    rightSpeedVal = maxSpeed;
    leftFrontMotor.setSpeed(leftSpeedVal);
    leftBackMotor.setSpeed(leftSpeedVal);
    rightFrontMotor.setSpeed(rightSpeedVal);
    rightBackMotor.setSpeed(rightSpeedVal);
  }
}

void controlSpeed() { // This function is called by driving()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  // Setting up proportional control
  if (deltaAngle > 30) {
    targetGyroX = 60;
  } else if (deltaAngle < -30) {
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if (round(targetGyroX - GyroX) == 0) {
    // No change needed
  } else if (targetGyroX > GyroX) {
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); // Would increase GyroX
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate() { // Called by loop(), isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1) {
    stopCar();
  } else {
    if (angle > targetAngle) { // Turn left
      left();
    } else if (angle < targetAngle) { // Turn right
      right();
    }

    // Setting up proportional control
    if (abs(deltaAngle) > 30) {
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if (round(targetGyroX - abs(GyroX)) == 0) {
      // No change needed
    } else if (targetGyroX > abs(GyroX)) {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1); // Would increase GyroX
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, +1);
    }
  }
}

void forward() { // Move forward
  leftFrontMotor.setSpeed(equilibriumSpeed);
  leftBackMotor.setSpeed(equilibriumSpeed);
  rightFrontMotor.setSpeed(equilibriumSpeed);
  rightBackMotor.setSpeed(equilibriumSpeed);
  leftFrontMotor.run(FORWARD);
  leftBackMotor.run(FORWARD);
  rightFrontMotor.run(FORWARD);
  rightBackMotor.run(FORWARD);
}

void left() { // Turn left
  leftFrontMotor.setSpeed(0);
  leftBackMotor.setSpeed(0);
  rightFrontMotor.setSpeed(equilibriumSpeed);
  rightBackMotor.setSpeed(equilibriumSpeed);
  rightFrontMotor.run(FORWARD);
  rightBackMotor.run(FORWARD);
}

void right() { // Turn right
  rightFrontMotor.setSpeed(0);
  rightBackMotor.setSpeed(0);
  leftFrontMotor.setSpeed(equilibriumSpeed);
  leftBackMotor.setSpeed(equilibriumSpeed);
  leftFrontMotor.run(FORWARD);
  leftBackMotor.run(FORWARD);
}

void stopCar() { // Stop or brake car
  leftFrontMotor.setSpeed(0);
  leftBackMotor.setSpeed(0);
  rightFrontMotor.setSpeed(0);
  rightBackMotor.setSpeed(0);
  leftFrontMotor.run(RELEASE);
  leftBackMotor.run(RELEASE);
  rightFrontMotor.run(RELEASE);
  rightBackMotor.run(RELEASE);
}

int changeSpeed(int speed, int direction) { // Helper function to increase or decrease speed
  if (speed == 0 && direction == 1) {
    speed = minSpeed;
  } else if (speed == minSpeed && direction == -1) {
    speed = 0;
  } else if (speed == maxSpeed && direction == 1) {
    speed = maxSpeed;
  } else {
    speed += direction * 5;
  }
  return constrain(speed, 0, maxSpeed);
}

void readAcceleration() { 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
}

void calculateError() { 
  AccErrorX = 0;
  AccErrorY = 0;
  GyroErrorX = 0;
  GyroErrorY = 0;
  GyroErrorZ = 0;
  int numberOfReads = 100;
  for (int i = 0; i < numberOfReads; i++) {
    readAcceleration();
    readGyro();
    AccErrorX += AccX;
    AccErrorY += AccY;
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    delay(10);
  }
  AccErrorX /= numberOfReads;
  AccErrorY /= numberOfReads;
  GyroErrorX /= numberOfReads;
  GyroErrorY /= numberOfReads;
  GyroErrorZ /= numberOfReads;
}
