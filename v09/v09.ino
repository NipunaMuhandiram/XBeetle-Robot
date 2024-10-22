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

const int intr = 18;    // Front Right sensor

// Timing variables
unsigned long previousMillis = 0;  // Store last time sensors were checked
const long interval = 50;           // Check sensors every 50 milliseconds

void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging

  // Set all motors to maximum speed initially
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Continuously check the sensors
  if (checkSensorsForLine()) {
    // backward();
    // delay(300); 
    stopCar(); 
    delay(500);  // Hold stop for a moment (can be adjusted)
    
  } else {
    
    if (!checkSensorsForLine()) {
    backward();
    // delay(300); 
    // stopCar(); 
    // delay(500);  // Hold stop for a moment (can be adjusted)
    }
      // Move forward if no line is detected
  }

  // Optional: Short delay to allow other processes, if necessary
  delay(10); // Allow a brief pause
}

bool checkSensorsForLine() {
  int centerleftValue = analogRead(centerleftSensor);
  int centerValue = analogRead(centerSensor);
  int centerrightValue = analogRead(centerrightSensor);
  int farleftValue = analogRead(farleftSensor);
  int farrightValue = analogRead(farrightSensor);
  int intrsenor = digitalRead(intr);
  
  // Debugging: Print sensor values
  Serial.print("CENTER_LEFT : "); Serial.print(centerleftValue);
  Serial.print(" CENTER: "); Serial.print(centerValue);
  Serial.print(" CENTER_RIGHT: "); Serial.print(centerrightValue);
  Serial.print(" FAR_LEFT: "); Serial.print(farleftValue);
  Serial.print(" FAR_RIGHT: "); Serial.println(farrightValue);

  Serial.print(" INTR : "); Serial.println(intrsenor);
  // Check if any sensor value is below 500
  if (centerleftValue < 500 || centerValue < 500 || centerrightValue < 500 || farleftValue < 500 || farrightValue < 500 || intrsenor) {
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
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  Serial.println("Car Stopped");
}

// Placeholder functions for backward and turning
void backward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Moving Backward");
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  Serial.println("Turning left");
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Turning Right");
}
