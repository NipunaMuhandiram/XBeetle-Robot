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

// Define pins for the front ultrasonic sensor
const int trigFront = 52;
const int echoFront = 53;

// Define pins for the back ultrasonic sensor
const int trigBack = 22;
const int echoBack = 23;


// // PID control variables
// float kp = 1.0, ki = 0.0, kd = 0.0;
// float lastError = 0, integral = 0;

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

  // Set pin modes
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);

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

        // Measure distances
        long distanceFront = measureDistance(trigFront, echoFront);
        long distanceBack = measureDistance(trigBack, echoBack);

        // Print the measured distances
        Serial.print("Front Distance: ");
        Serial.print(distanceFront);
        Serial.println(" cm");

        Serial.print("Back Distance: ");
        Serial.print(distanceBack);
        Serial.println(" cm");
  
          // if (carStoppedFront) {
          //   carStoppedFront = false;
            
          // } else if(carStoppedBack) {
          //     carStoppedBack = false;
              
          // }else {
          //   forward(150);
          // }

          // if (!driveDirection) {
          //   if (distanceFront < 100 ){
          //        forward(150);
          //       // zigzagUntilObstacleFront(150, 100, 150);

          //   }else{
          //     zigzagUntilObstacleFront(150, 100, 150);

          //     // if (distanceFront < 100 ){
          //     //    forward(150);
          //     // }
          //   }
             
            
          // } else {
              
          // if (distanceFront < 100 ){
          //        backward(150);
          //     // zigzagUntilObstacleBack(150, 100, 150);

          //   }else{
          //     zigzagUntilObstacleBack(150, 100, 150);
          //     // if (distanceFront < 100 ){
          //     //    backward(150);
          //     // }
          //   }
              
          // }
          

  
  delay(10);  // Allow a brief pause
}


void zigzagUntilObstacleFront(int speed, int delayTime, int distanceThreshold) {
    while (true) {
        // Measure the front distance to detect obstacles
        long distanceFront = measureDistance(trigFront, echoFront);

        // Check if an obstacle is detected within the threshold distance
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }

        // Move forward while turning left
        turnLeft(speed);
        delay(500);
        forward(speed);
        delay(delayTime);  // Delay for the duration of the left movement

        // Check for an obstacle after the left turn
        distanceFront = measureDistance(trigFront, echoFront);
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }

        // Move forward while turning right
        turnRight(speed);
        delay(500);
        forward(speed);
        delay(delayTime);  // Delay for the duration of the right movement

        // Check for an obstacle after the right turn
        distanceFront = measureDistance(trigFront, echoFront);
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }
    }
}


void zigzagUntilObstacleBack(int speed, int delayTime, int distanceThreshold) {
    while (true) {
        // Measure the front distance to detect obstacles
        long distanceFront = measureDistance(trigFront, echoFront);

        // Check if an obstacle is detected within the threshold distance
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }

        // Move forward while turning left
        turnLeft(speed);
        delay(500);
        backward(speed);
        delay(delayTime);  // Delay for the duration of the left movement

        // Check for an obstacle after the left turn
        distanceFront = measureDistance(trigFront, echoFront);
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }

        // Move forward while turning right
        turnRight(speed);
        delay(500);
        backward(speed);
        delay(delayTime);  // Delay for the duration of the right movement

        // Check for an obstacle after the right turn
        distanceFront = measureDistance(trigFront, echoFront);
        if (distanceFront < distanceThreshold) {
            Serial.println("Obstacle detected! Stopping zigzag movement.");
            stopCar();  // Stop the car when an obstacle is detected
            break;      // Exit the loop
        }
    }
}


// Function to measure the distance using an ultrasonic sensor
long measureDistance(int trigPin, int echoPin) {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);   // Clear the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Set the trigPin HIGH for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the pulse on the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (duration / 2 because of the round-trip)
  // Speed of sound is 343 m/s or 0.0343 cm/us
  long distance = (duration * 0.0343) / 2;

  return distance;
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
  backward(150);
  // zigzagUntilObstacleFront(150, 100, 150);
  // backward(150);
  
}

// Interrupt Service Routine (ISR) to stop the car when the interrupt is triggered
void sensorInterruptBack() {
  driveDirection = false;
  carStoppedBack = true;
  Serial.println("Car stopped back.");
  stopCar(); 
  forward(150);
  // zigzagUntilObstacleBack(150, 100, 150);
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
