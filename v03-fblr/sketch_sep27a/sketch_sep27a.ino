#include <AFMotor.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor



void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging

  // Set all motors to maximum speed
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);


}

void loop() {


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
  delay(240);

  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(240);
  Serial.println("Turning left");

}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  delay(290);

  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(290);
  Serial.println("Turning Right");
}

