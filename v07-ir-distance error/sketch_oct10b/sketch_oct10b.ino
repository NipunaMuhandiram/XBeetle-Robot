#include <AFMotor.h>

//defining motors
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Define sensor pins
const int centerleftSensor = A1;
const int centerrightSensor = A3;
const int centerSensor  = A2;
const int farleftSensor = A0;
const int farrightSensor = A4;

// PID Constants
float Kp = 0.6;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.4;  // Derivative gain

// Variables for PID calculation
int lastError = 0;
float integral = 0;

// Threshold to detect line
int lineThreshold = 500;

void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging

  // Set all motors to maximum speed
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
}

void loop() {
  int centerleftValue = analogRead(centerleftSensor);
  int centerValue = analogRead(centerSensor);
  int centerrightValue = analogRead(centerrightSensor);
  int farleftValue = analogRead(farleftSensor);
  int farrightValue = analogRead(farrightSensor);

  // Debugging: Print sensor values
  Serial.print("CENTER_LEFT : "); Serial.print(centerleftValue);
  Serial.print(" CENTER: "); Serial.print(centerValue);
  Serial.print(" CENTER_RIGHT: "); Serial.print(centerrightValue);
  Serial.print(" FAR_LEFT: "); Serial.print(farleftValue);
  Serial.print(" FAR_RIGHT: "); Serial.println(farrightValue);

  // Detect if we are at a corner
  if (farleftValue < lineThreshold && centerValue > lineThreshold && farrightValue > lineThreshold) {
    // Far-left sensor sees the line, indicating a left corner
    turnLeft90();
    // turnLeft90();
  } 
  else if (farrightValue < lineThreshold && centerValue > lineThreshold && farleftValue > lineThreshold) {
    // Far-right sensor sees the line, indicating a right corner
    turnRight90();
    // turnRight90();
  } 
  else {
    // Regular line following using PID
    followLinePID(centerleftValue, centerValue, centerrightValue, farleftValue, farrightValue);
  }
  
  delay(50);  // Short delay for smoother movement
}

void followLinePID(int centerleftValue, int centerValue, int centerrightValue, int farleftValue, int farrightValue) {
  // Calculate weighted error from sensors
  int error = (farleftValue * -2) + (centerleftValue * -1) + (centerValue * 0) + (centerrightValue * 1) + (farrightValue * 2);

  // Calculate PID terms
  int P = error;                            // Proportional term
  integral += error;                        // Accumulate the integral term
  int I = integral;                         // Integral term
  int D = error - lastError;                // Derivative term (rate of change)
  lastError = error;                        // Update last error

  // PID output (correction factor)
  int correction = (Kp * P) + (Ki * I) + (Kd * D);

  // Apply correction to motor speeds
  int baseSpeed = 120;  // Set a base speed for both motors

  int leftMotorSpeed = baseSpeed - correction;  // Adjust left motor speed
  int rightMotorSpeed = baseSpeed + correction; // Adjust right motor speed

  // Ensure motor speeds are within valid bounds (0 - 255)
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 200);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 200);

  // Control motors based on corrected speed
  motor1.setSpeed(leftMotorSpeed);
  motor2.setSpeed(leftMotorSpeed);
  motor3.setSpeed(rightMotorSpeed);
  motor4.setSpeed(rightMotorSpeed);

  // Move forward with adjusted speeds
  forward();
}

void forward() {
    //Forward
    motor1.run(FORWARD);
    motor1.setSpeed(150);
    motor2.run(FORWARD);
    motor2.setSpeed(150);
    motor3.run(FORWARD);
    motor3.setSpeed(150);
    motor4.run(FORWARD);
    motor4.setSpeed(150);
}

// Function to handle a 90-degree left turn
void turnLeft90() {
   //turn left
    motor1.run(FORWARD);
    motor1.setSpeed(200);
    motor2.run(FORWARD);
    motor2.setSpeed(200);
    motor3.run(BACKWARD);
    motor3.setSpeed(200);
    motor4.run(BACKWARD);
    motor4.setSpeed(200);
}

// Function to handle a 90-degree right turn
void turnRight90() {
    //turn right
    motor1.run(BACKWARD);
    motor1.setSpeed(200);
    motor2.run(BACKWARD);
    motor2.setSpeed(200);
    motor3.run(FORWARD);
    motor3.setSpeed(200);
    motor4.run(FORWARD);
    motor4.setSpeed(200);
}

void stopCar() {
    //stop
    motor1.run(RELEASE);
    motor1.setSpeed(0);
    motor2.run(RELEASE);
    motor2.setSpeed(0);
    motor3.run(RELEASE);
    motor3.setSpeed(0);
    motor4.run(RELEASE);
    motor4.setSpeed(0);
}
