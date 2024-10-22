#include <AFMotor.h>

// Define motor objects for 4 motors
AF_DCMotor motor1(1);  // Front Left Motor
AF_DCMotor motor2(2);  // Front Right Motor
AF_DCMotor motor3(3);  // Back Left Motor
AF_DCMotor motor4(4);  // Back Right Motor

// Define sensor pins
const int intrb = 19;  // Left sensor
const int centerrightSensor = 20;  // Right sensor
const int intr = 18;    // Pin for external interrupt (Middle sensor, always on the black line)
const int farleftSensor =23;
const int farrightSensor = 22;


// PID control variables
float kp = 1.0, ki = 0.0, kd = 0.0;
float lastError = 0, integral = 0;

// Timing variables
unsigned long previousMillis = 0;  // Store last time sensors were checked
const long interval = 50;           // Check sensors every 50 milliseconds

// Flags to indicate if interrupt was triggered and if it has been handled
volatile bool carStoppedFront = false;
volatile bool carStoppedBack = false;
volatile bool driveDirection = false;

bool interruptHandled = false;  // Ensure interrupt only handled once


void setup() {
  Serial.begin(9600);  // Start the serial communication for debugging

  // Set all motors to maximum speed initially
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  // Attach interrupts to sensor pins (FALLING edge for detecting line change)
  attachInterrupt(digitalPinToInterrupt(intr), stopCarInterruptFront, FALLING);
  attachInterrupt(digitalPinToInterrupt(intrb), sensorInterruptBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(centerrightSensor), sensorInterrupt, CHANGE);

  Serial.println("Setup complete: Sensors initialized and interrupts attached");
}

void loop() {
  
          // if (carStoppedFront) {
          //   carStoppedFront = false;
            
          // } else if(carStoppedBack) {
          //     carStoppedBack = false;
              
          // }else {
          //   forward(150);
          // }

          if (!driveDirection) {
              forward(150);
            
          } else {
              backward(150);
              
          }
          

  
  delay(10);  // Allow a brief pause
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
void stopCarInterruptFront() {
  driveDirection = true;
  carStoppedFront = true;
  Serial.println("Car stopped front.");
  stopCar(); 
  // backward(150);
}

// Interrupt Service Routine (ISR) to stop the car when the interrupt is triggered
void sensorInterruptBack() {
  driveDirection = false;
  carStoppedBack = true;
  Serial.println("Car stopped back.");
  stopCar(); 
  // forward(150);
}

// Interrupt handler for left and right sensors to feed PID controller
void sensorInterrupt() {
  // Recalculate PID only if the car is moving
  // if (!carStopped) {
  //   Serial.println("Sensor interrupt triggered. Running PID control...");
  //   // pidControl();
  // }
}

// Placeholder functions for backward and turning with debugging
void forward(int speed) {
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

void backward(int speed) {
  motor1.setSpeed(speed);  // Set speed dynamically
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  Serial.println("Moving forward with speed: " + String(speed));
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
