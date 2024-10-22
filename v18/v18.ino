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
           // Check sensors every 50 milliseconds

// Flags to indicate if interrupt was triggered and if it has been handled
volatile bool carStoppedFront = false;
volatile bool carStoppedBack = false;
volatile bool driveDirection = false;

volatile bool isInitial = false;

bool interruptHandled = false;  // Ensure interrupt only handled once

// Timing variables
unsigned long previousMillis = 0;  // Store last time sensors were checked
const long interval = 50;           // Check sensors every 50 milliseconds

// Track interrupts and timing for both front and back sensors
volatile int interruptCountFront = 0;       // Count of front interrupts
volatile int interruptCountBack = 0;        // Count of back interrupts
unsigned long lastInterruptTimeFront = 0;   // Time of last front interrupt
unsigned long lastInterruptTimeBack = 0;    // Time of last back interrupt

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

        // Reset front interrupt count if 10 seconds have passed
        if (millis() - lastInterruptTimeFront > 3000) {
            interruptCountFront = 0;  // Reset the front interrupt count after 10 seconds
        }

        // Reset back interrupt count if 10 seconds have passed
        if (millis() - lastInterruptTimeBack > 3000) {
            interruptCountBack = 0;  // Reset the back interrupt count after 10 seconds
        }

          if (!driveDirection) {

              // checkObstacleAndTurnBack(200, 50);

              // if (isInitial){
              //   turnLeft(200);   // Slight left turn at lower speed
              //   delay(100);
              //   isInitial = false;      // Hold the turn for 500ms
              // }

              // stopCar();       // Stop before moving forward

              forward(150);
            
          } else {
              // checkObstacleAndTurnFront(200, 50);

              // if (isInitial){
              //   turnRight(200);  // Slight right turn at lower speed
              //   delay(100);      // Hold the turn for 500ms
              //   isInitial = false;
              // }

              // stopCar();       // Stop before moving backward

              backward(150);
              
          }
  
  delay(10);  // Allow a brief pause
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

void stopCarInterruptFront() {
    long distanceFront = measureDistance(trigFront, echoFront);
    driveDirection = true;
    carStoppedFront = true;
    Serial.println("Car stopped front.");
    stopCar(); 

    // Increment the interrupt count for the front sensor
    interruptCountFront++;
    lastInterruptTimeFront = millis();  // Update the time of the last interrupt

    // Check if front interrupt count exceeds 5
    if (interruptCountFront > 5) {
        Serial.println("Front interrupt count exceeded 5, turning left.");
        turnLeft(150); // Turn left if threshold is exceeded
        delay(1000);   // Delay to allow the turn to complete
        interruptCountFront = 0;  // Reset front count after turning
    } else {
        // Continuously check for obstacles while turning and moving
        while (true) {
            // Use the checkObstacleAndTurnFront function to detect obstacles and turn if necessary
            checkObstacleAndTurnFront(200, 50);  // Check for obstacles with speed 200, distance threshold 50

            distanceFront = measureDistance(trigFront, echoFront);
            if (distanceFront > 80) {
                // No obstacle detected, continue moving backward
                Serial.println("No obstacle detected, moving backward.");
                if (isInitial) {
                    turnRight(200);  // Slight right turn at lower speed
                    delay(100);      // Hold the turn for 100ms
                    isInitial = false;
                }

                backward(150);  // Move backward
            } else {
                // Obstacle detected, stop and exit the loop
                Serial.println("Obstacle detected, stopping car.");
                stopCar();
                break;  // Exit the while loop
            }

            delay(100);  // Delay to allow for sensor readings
        }
    }
}

void sensorInterruptBack() {
    driveDirection = false;
    carStoppedBack = true;
    Serial.println("Car stopped back.");
    stopCar(); 

    // Increment the interrupt count for back sensor
    interruptCountBack++;
    lastInterruptTimeBack = millis();  // Update the time of the last interrupt

    // Check if back interrupt count exceeds 5
    if (interruptCountBack > 5) {
        Serial.println("Back interrupt count exceeded 5, turning right.");
        turnRight(150); // Turn right if threshold is exceeded
        delay(1000);    // Delay to allow the turn to complete
        interruptCountBack = 0;  // Reset back count after turning
    } else {
        // Continuously check for obstacles while turning and moving
        while (true) {
            // Use the checkObstacleAndTurnBack function to detect obstacles and turn if necessary
            checkObstacleAndTurnBack(200, 50);  // Check for obstacles with speed 200, distance threshold 50

            long distanceBack = measureDistance(trigBack, echoBack);  // Measure distance at the back
            if (distanceBack > 50) {
                // No obstacle detected, continue moving forward
                Serial.println("No obstacle detected, moving forward.");
                if (isInitial) {
                    turnLeft(200);  // Slight left turn at lower speed
                    delay(100);     // Hold the turn for 100ms
                    isInitial = false;
                }

                forward(150);  // Move forward
            } else {
                // Obstacle detected, stop and exit the loop
                Serial.println("Obstacle detected, stopping car.");
                stopCar();
                break;  // Exit the while loop
            }

            delay(100);  // Delay to allow for sensor readings
        }
    }
}


// void stopCarInterruptFront() {
//     long distanceFront = measureDistance(trigFront, echoFront);
//     driveDirection = true;
//     carStoppedFront = true;
//     Serial.println("Car stopped front.");
//     stopCar(); 

//     // Increment the interrupt count for front sensor
//     interruptCountFront++;
//     lastInterruptTimeFront = millis();  // Update the time of the last interrupt

//     // Check if front interrupt count exceeds 5
//     if (interruptCountFront > 5) {
//         Serial.println("Front interrupt count exceeded 5, turning left.");
//         turnLeft(150); // Turn left if threshold is exceeded
//         delay(1000);   // Delay to allow the turn to complete
//         interruptCountFront = 0;  // Reset front count after turning
//     } else {
//         // Add slight angle before moving backward
//         checkObstacleAndTurnFront(200, 50);

//         if (isInitial){
//           turnRight(200);  // Slight right turn at lower speed
//           delay(100);      // Hold the turn for 500ms
//           isInitial = false;
//         }

//         // stopCar();       // Stop before moving backward

//         backward(150);
//     }
// }

// void sensorInterruptBack() {
//     driveDirection = false;
//     carStoppedBack = true;
//     Serial.println("Car stopped back.");
//     stopCar(); 

//     // Increment the interrupt count for back sensor
//     interruptCountBack++;
//     lastInterruptTimeBack = millis();  // Update the time of the last interrupt

//     // Check if back interrupt count exceeds 5
//     if (interruptCountBack > 5) {
//         Serial.println("Back interrupt count exceeded 5, turning right.");
//         turnRight(150); // Turn right if threshold is exceeded
//         delay(1000);    // Delay to allow the turn to complete
//         interruptCountBack = 0;  // Reset back count after turning
//     } else {
//         // Add slight angle before moving forward
//         checkObstacleAndTurnBack(200, 50);

//         if (isInitial){
//           turnLeft(200);   // Slight left turn at lower speed
//           delay(100);
//           isInitial = false;      // Hold the turn for 500ms
//         }

//         // stopCar();       // Stop before moving forward

//         forward(150);
//     }
// }

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

void checkObstacleAndTurnFront(int speed, int distanceThreshold) {
    long distanceFront;
    
    // Step 1: Initially, the robot is moving straight, so we check for obstacles
    distanceFront = measureDistance(trigFront, echoFront);

    if (distanceFront < distanceThreshold) {
        Serial.println("Obstacle detected in front! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }
    
    // Step 2: Turn Left 45 degrees
    turnLeft45(speed);
    turnLeft45(speed);
    // turnLeft45(speed);
    delay(500);  // Delay to hold the turn for 500ms
    
    distanceFront = measureDistance(trigFront, echoFront);
    if (distanceFront < distanceThreshold) {
        Serial.println("Obstacle detected after turning left! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }

    // Step 3: Turn Right 90 degrees (this moves the robot 45 degrees to the right from the original position)
    turnRight45(speed);  // Reset to the original direction
    turnRight45(speed);  // Move 45 degrees to the right
    // turnRight45(speed);
    delay(1000);  // Delay for the right turn

    distanceFront = measureDistance(trigFront, echoFront);
    if (distanceFront < distanceThreshold) {
        Serial.println("Obstacle detected after turning right! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }
    isInitial = true;
    Serial.println("No obstacle detected. Reset to original direction.");
}

void checkObstacleAndTurnBack(int speed, int distanceThreshold) {
    long distanceBack;
    
    // Step 1: Initially, the robot is moving straight, so we check for obstacles
    // distanceFront = measureDistance(trigFront, echoFront);
    distanceBack = measureDistance(trigBack, echoBack);

    if (distanceBack < distanceThreshold) {
        Serial.println("Obstacle detected in front! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }
    
    // Step 2: Turn Left 45 degrees
    turnLeft45(speed);
    turnLeft45(speed);
    // turnLeft45(speed);
    delay(500);  // Delay to hold the turn for 500ms
    
    distanceBack = measureDistance(trigBack, echoBack);;
    if (distanceBack < distanceThreshold) {
        Serial.println("Obstacle detected after turning left! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }

    // Step 3: Turn Right 90 degrees (this moves the robot 45 degrees to the right from the original position)
    turnRight45(speed);  // Reset to the original direction
    turnRight45(speed);  // Move 45 degrees to the right
    // turnRight45(speed);
    delay(1000);  // Delay for the right turn

    distanceBack = measureDistance(trigBack, echoBack);
    if (distanceBack < distanceThreshold) {
        Serial.println("Obstacle detected after turning right! Stopping.");
        stopCar();
        return;  // Stop if the obstacle is detected
    }
    isInitial = true;
    Serial.println("No obstacle detected. Reset to original direction.");
}

void turnLeft45(int speed) {
    motor1.setSpeed(speed);  
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(speed);

    // Turn left by driving left side motors backward and right side motors forward
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);

    delay(300);  // Adjust delay to achieve approximately 45-degree turn
    stopCar();   // Stop the car after the turn
    Serial.println("Turned left 45 degrees");
}

void turnRight45(int speed) {
    motor1.setSpeed(speed);  
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(speed);

    // Turn right by driving left side motors forward and right side motors backward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);

    delay(300);  // Adjust delay to achieve approximately 45-degree turn
    stopCar();   // Stop the car after the turn
    Serial.println("Turned right 45 degrees");
}
