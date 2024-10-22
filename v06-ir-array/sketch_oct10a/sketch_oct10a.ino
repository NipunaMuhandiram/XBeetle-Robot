#include <AFMotor.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor

// Define sensor pins
const int centerleftSensor = A1;  // Left sensor
const int centerrightSensor = A3;  // Center sensor
const int centerSensor  = A2;  // Right sensor
const int farleftSensor = A0;  // Front Left sensor
const int farrightSensor = A4;  // Front Right sensor

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

  // Simple line following logic
  if (centerValue < 500) {  // Assuming values below 500 indicate line
    forward();
  } else if (centerleftValue < 500 || farleftValue < 500) {
   turnLeft();  // If the left sensor is on the line, turn right
  } else if (centerrightValue < 500 || farrightValue < 500) {
    turnRight();  // If the right sensor is on the line, turn left
  } else {
    stopCar();  // Stop if no line is detected
  }

  delay(100);  // Adjust the delay as needed for smooth operation
}

void forward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  Serial.println("Moving Forward");
}

void backward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Moving Backward");
}

void stopCar() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  Serial.println("Car Stopped");
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
