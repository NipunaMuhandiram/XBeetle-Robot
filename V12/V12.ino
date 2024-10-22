#include <AFMotor.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor

// Define sensor pins
const int centerleftSensor = 19;  // Left sensor
const int centerrightSensor = 20;  // Right sensor
const int intr = 18;    // Pin for external interrupt (Middle sensor, always on the black line)

// PID control variables
float kp = 1.0, ki = 0.0, kd = 0.0;
float lastError = 0, integral = 0;

// Timing variables
unsigned long previousMillis = 0;  // Store last time sensors were checked
const long interval = 50;           // Check sensors every 50 milliseconds

// Flags to indicate if interrupt was triggered and if it has been handled
volatile bool carStopped = false;
bool interruptHandled = false;  // Ensure interrupt only handled once

void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging

  // Set all motors to maximum speed initially
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  // Attach interrupts to sensor pins (FALLING edge for detecting line change)
  attachInterrupt(digitalPinToInterrupt(intr), stopCarInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(centerleftSensor), sensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(centerrightSensor), sensorInterrupt, CHANGE);

  Serial.println("Setup complete: Sensors initialized and interrupts attached");
}

void loop() {
  // If the interrupt occurred and hasn't been handled yet
  if (carStopped && !interruptHandled) {
    Serial.println("Car stopped due to interrupt.");
    stopCar();  // Stop the car
    delay(1000);  // Delay to prevent immediate restart
    Serial.println("Car stopped and delay finished. Resetting flags.");
    turnLeft(180);
    delay(500);
    carStopped = false;   // Reset the carStopped flag
    interruptHandled = true;  // Mark the interrupt as handled
  } else {
    unsigned long currentMillis = millis(); // Get the current time
    if (interruptHandled) {
      // Implement PID control after the interrupt has been handled
      Serial.println("Interrupt handled, running PID control...");
      pidControl();
    } else {
      Serial.println("No interrupt, moving backward...");
      backward(180);  // Move backward if no line is detected
    }
  }

  // Optional: Short delay to allow other processes, if necessary
  delay(10);  // Allow a brief pause
}

// PID Control function with debugging
void pidControl() {
  int centerleftValue = digitalRead(centerleftSensor);
  int centerrightValue = digitalRead(centerrightSensor);

  // Debug sensor readings
  Serial.print("Sensor Values - Left: ");
  Serial.print(centerleftValue);
  Serial.print(", Right: ");
  Serial.println(centerrightValue);

  // Calculate the error based on center value (deviation from the center)
  int error = (centerleftValue) - (centerrightValue);

  // PID calculations
  float proportional = kp * error;
  integral += error;
  float integral_term = ki * integral;
  float derivative = kd * (error - lastError);
  lastError = error;

  // PID output
  float pid_output = proportional + integral_term + derivative;

  // Adjust motor speeds based on PID output
  int baseSpeed = 100;
  int leftMotorSpeed = baseSpeed + pid_output;
  int rightMotorSpeed = baseSpeed - pid_output;

  // Make sure speeds are within valid range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 185);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 185);

  // Debug PID and motor speed values
  Serial.print("PID Output: ");
  Serial.print(pid_output);
  Serial.print(" | Left Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right Motor Speed: ");
  Serial.println(rightMotorSpeed);

  // Set motor speeds
  motor1.setSpeed(leftMotorSpeed);
  motor2.setSpeed(rightMotorSpeed);
  motor3.setSpeed(leftMotorSpeed);
  motor4.setSpeed(rightMotorSpeed);

  // Move forward with adjusted speeds
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopCar() {
  // Stop all motors
  motor1.setSpeed(0); 
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
  Serial.println("Car Stopped");
}

// Interrupt Service Routine (ISR) to stop the car when the interrupt is triggered
void stopCarInterrupt() {
  if (!interruptHandled) {  // Only trigger if interrupt hasn't been handled
    carStopped = true;  // Set flag to indicate car has stopped
    Serial.println("Interrupt triggered: Car will stop.");
  }
}

// Interrupt handler for left and right sensors to feed PID controller
void sensorInterrupt() {
  // Recalculate PID only if the car is moving
  if (!carStopped) {
    Serial.println("Sensor interrupt triggered. Running PID control...");
    pidControl();
  }
}

// Placeholder functions for backward and turning with debugging
void backward(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  Serial.println("Moving Backward with speed: " + String(speed));
}

void turnLeft(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  Serial.println("Turning left with speed: " + String(speed));
}

void turnRight(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  Serial.println("Turning Right with speed: " + String(speed));
}
