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

// Flag variables to detect if both interrupts are triggered
volatile bool frontInterruptTriggered = false;
volatile bool backInterruptTriggered = false;

// Time threshold for considering "simultaneous" interrupts
const unsigned long simultaneousThreshold = 10;  // 10 milliseconds

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

        // Check if both interrupts were triggered within a short time interval
        if (frontInterruptTriggered && backInterruptTriggered) {
            unsigned long timeDifference = abs(lastInterruptTimeFront - lastInterruptTimeBack);
            if (timeDifference <= simultaneousThreshold) {
                Serial.println("Both front and back interrupts triggered simultaneously!");
                turnLeft(255);   // Slight left turn at lower speed
                delay(200);
                forward(150);

            }
            // Reset flags
            frontInterruptTriggered = false;
            backInterruptTriggered = false;
        }

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
              // turnLeft(200);   // Slight left turn at lower speed
              // delay(50);
              forward(150);
            
          } else {
              // checkObstacleAndTurnFront(200, 50);

              // if (isInitial){
              //   turnRight(200);  // Slight right turn at lower speed
              //   delay(100);      // Hold the turn for 500ms
              //   isInitial = false;
              // }

              // stopCar();       // Stop before moving backward
              // turnRight(200);
              // delay(50);
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
    stopCar();
    lastInterruptTimeFront = millis();  // Record the time of interrupt
    frontInterruptTriggered = true;     // Set the flag for front sensor interrupt
    long distanceFront = measureDistance(trigFront, echoFront);
    driveDirection = true;
    carStoppedFront = true;
    Serial.println("Car stopped front.");
    stopCar(); 
    forward(150);
     
    // forwardDual(150,130);
    // stopCar(); 
    // turnLeft(150);
    // Increment the interrupt count for the front sensor
    interruptCountFront++;
    lastInterruptTimeFront = millis();  // Update the time of the last interrupt

    // Check if front interrupt count exceeds 5
    // if (interruptCountFront > 10) {
    //     Serial.println("Front interrupt count exceeded 5, turning left.");
    //     turnLeft(150); // Turn left if threshold is exceeded
    //     delay(1000);   // Delay to allow the turn to complete
    //     interruptCountFront = 0;  // Reset front count after turning
    // } else {
    //     // Continuously check for obstacles while turning and moving
    //     // checkAndTurnToObstacleFront();
    // }
}

void sensorInterruptBack() {
  stopCar();
    lastInterruptTimeBack = millis();  // Record the time of interrupt
    backInterruptTriggered = true;     // Set the flag for back sensor interrupt
    driveDirection = false;
    carStoppedBack = true;
    Serial.println("Car stopped back.");
    stopCar(); 
    backward(150);
     
    // backwardDual(150,130);
    // stopCar(); 
    // turnRight(150);
    // Increment the interrupt count for back sensor
    interruptCountBack++;
    lastInterruptTimeBack = millis();  // Update the time of the last interrupt

    // Check if back interrupt count exceeds 5
    // if (interruptCountBack > 10) {
    //     Serial.println("Back interrupt count exceeded 5, turning right.");
    //     turnRight(150); // Turn right if threshold is exceeded
    //     delay(1000);    // Delay to allow the turn to complete
    //     interruptCountBack = 0;  // Reset back count after turning
    // } else {
    //     // Continuously check for obstacles while turning and moving
    //     // checkAndTurnToObstacleBack();
    // }
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

// Placeholder functions for backward and turning with debugging
void forwardDual(int speed1 ,int speed2) {
  motor1.setSpeed(speed1);  // Set speed dynamically
  motor2.setSpeed(speed1);
  motor3.setSpeed(speed2);
  motor4.setSpeed(speed2);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  // Serial.println("Moving Backward with speed: " + String(speed));
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

void backwardDual(int speed1,int speed2) {
  motor1.setSpeed(speed1);  // Set speed dynamically
  motor2.setSpeed(speed1);
  motor3.setSpeed(speed2);
  motor4.setSpeed(speed2);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Serial.println("Moving forward with speed: " + String(speed));
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


// void checkAndTurnToObstacleBack() {
//     // Check front distance
//     long frontDistance = measureDistance(trigFront, echoFront);
    
//     // If the front distance is below the threshold, check for obstacles
//     if (frontDistance < 80) {
//         Serial.println("Obstacle detected in front!");

//         // Move forward towards the obstacle
//         backward(150);
//         backward(150);
//         // delay(500); // Move forward for a brief moment
//     }else{
//           // turnLeft(150);
//     }
// }


// void checkAndTurnToObstacleFront() {
//     // Check front distance
//     long frontDistance = measureDistance(trigBack, echoBack);
    
//     // If the front distance is below the threshold, check for obstacles
//     if (frontDistance < 80) {
//         Serial.println("Obstacle detected in front!");

//         // Move forward towards the obstacle
//         forward(150);
//         // delay(500); // Move forward for a brief moment
//     }else{
//           // turnRight(150);
//     }
// }

