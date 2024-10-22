// car stop once

#include <AFMotor.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor

// Define sensor pins
const int centerleftSensor = A1;  // Left sensor
const int centerrightSensor = A3;  // Center sensor
const int centerSensor  = A2;     // Right sensor
const int farleftSensor = A0;     // Front Left sensor
const int farrightSensor = A4;    // Front Right sensor

const int intr = 18;    // Pin for external interrupt (Interrupt sensor)

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

  // Attach the interrupt to pin 18 (on the FALLING edge)
  attachInterrupt(digitalPinToInterrupt(intr), stopCarInterrupt, FALLING);
}

void loop() {
  // If the interrupt occurred and hasn't been handled yet
  if (carStopped && !interruptHandled) {
    stopCar();  // Stop the car
    delay(1000);  // Delay to prevent immediate restart
    turnLeft(180);
    delay(500);
    carStopped = false;   // Reset the carStopped flag
    interruptHandled = true;  // Mark the interrupt as handled
  } else {
    unsigned long currentMillis = millis(); // Get the current time
    backward(180);  // Move backward if no line is detected
  }

  // Optional: Short delay to allow other processes, if necessary
  delay(10);  // Allow a brief pause
}

bool checkSensorsForLine() {
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

  // Check if any sensor value is below 500 (indicating line detection)
  if (centerleftValue < 500 || centerValue < 500 || centerrightValue < 500 || farleftValue < 500 || farrightValue < 500) {
    return true;  // A sensor has detected the line
  }
  return false;  // No sensor has detected the line
}

// Function to move forward with PWM speed control
void forward(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
  Serial.print("Moving Forward at speed: ");
  Serial.println(speed);
}

void stopCar() {
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
  }
}

// Placeholder functions for backward and turning
void backward(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Moving Backward");
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
  Serial.println("Turning left");
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
  Serial.println("Turning Right");
}
