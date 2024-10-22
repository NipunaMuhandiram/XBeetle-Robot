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
const int farleftSensor =23;
const int farrightSensor = 22;


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
    carStopped = false;   // Reset the carStopped flag
    handleTurn();
    interruptHandled = true;  // Mark the interrupt as handled
    pidControl();
    delay(5000);
  } else {
      unsigned long currentMillis = millis(); // Get the current time
      int centerleftValue = digitalRead(centerleftSensor);
      int farleftValue = digitalRead(farleftSensor);
      if(centerleftValue == HIGH || farleftValue == HIGH){
          stopCar();
          handleTurn();
      }
      pidControl();
      backward(150);
  }

  // Optional: Short delay to allow other processes, if necessary
  delay(10);  // Allow a brief pause
}



void handleTurn() {
  Serial.println("Handling turn: Moving forward until a black line is detected...");

  // Keep moving forward until a black line is detected by one of the sensors
  while (true) {
    int centerleftValue = digitalRead(centerleftSensor);
    int centerrightValue = digitalRead(centerrightSensor);
    int farleftValue = digitalRead(farleftSensor);
    int farrightValue = digitalRead(farrightSensor);

    // Debug sensor readings
    Serial.print("Sensor Values - Center Left: ");
    Serial.print(centerleftValue);
    Serial.print(", Center Right: ");
    Serial.print(centerrightValue);
    Serial.print(", Far Left: ");
    Serial.print(farleftValue);
    Serial.print(", Far Right: ");
    Serial.println(farrightValue);

    forward(150);
    delay(10);
    stopCar();

        // Check for far left sensor detection
    if (centerleftValue == HIGH || centerrightValue == HIGH || farrightValue == HIGH || farleftValue == HIGH) {
      Serial.println("Far left line detected. Initiating left turn...");
      Serial.println("carStopped is ");
      Serial.println(carStopped);
      stopCar();
      delay(10);

      backward(200);
      // backward(200);
      delay(100);

      // Rotate left until `carStopped = true`
      while (!carStopped) {
        turnLeft(160); // Turning left with speed
        delay(50);    // Delay to avoid overwhelming the loop
        stopCar();
        pidControl();
        // Debugging to check if the car is stopped
        Serial.println("Turning left... Waiting for car to stop.");
        carStopped = true;
      }
      if(carStopped){
        if (centerleftValue == HIGH || centerrightValue == HIGH || farrightValue == HIGH || farleftValue == HIGH) {
          pidControl();
          pidControl();
          pidControl();
        
        }else{
          // not sure working or not
          while (!centerleftValue == HIGH || !centerrightValue == HIGH || !farrightValue == HIGH || !farleftValue == HIGH) {
                  turnLeft(160); // Turning left with speed
                  delay(50);    // Delay to avoid overwhelming the loop
                  stopCar();
                  backward(150);
                  delay(50);    // Delay to avoid overwhelming the loop
                  stopCar();
                  pidControl();

                  // Re-read sensor values to decide the next action
                  centerleftValue = digitalRead(centerleftSensor);
                  centerrightValue = digitalRead(centerrightSensor);
                  farleftValue = digitalRead(farleftSensor);
                  farrightValue = digitalRead(farrightSensor);
                }

        }

      }
      
      // backward(120);
      // delay(100);
      // Serial.println("Car has stopped after left turn.");
      return; // Exit handleTurn once carStopped becomes true
    }

    // // Check for any other black line detected
    // if (centerleftValue == HIGH || centerrightValue == HIGH || farrightValue == HIGH || farleftValue == HIGH) {
    //   Serial.println("Black line detected. Stopping...");

    //     if (farleftValue == HIGH) {
    //       Serial.println("Far left line detected, correcting path...");
    //       return; 
    //     }
    //   stopCar();
    //   delay(10000);  
    //   break; 
    // }


    delay(10);
  }
  carStopped = false;   // Reset the carStopped flag
}



void pidControl() {
  int centerleftValue = digitalRead(centerleftSensor);
  int centerrightValue = digitalRead(centerrightSensor);

  int farleftValue = digitalRead(farleftSensor);
  int farrightValue = digitalRead(farrightSensor);

  // Error calculation
  int error = 0;
  if (centerleftValue == HIGH || farleftValue == HIGH) {
    error = -1;  // Car is too far to the right
  } else if (centerrightValue == HIGH || farrightValue == HIGH) {
    error = 1;   // Car is too far to the left
  }

  // PID calculations
  float proportional = kp * error;
  integral += error;
  float integral_term = ki * integral;
  float derivative = kd * (error - lastError);
  lastError = error;

  float pid_output = proportional + integral_term + derivative;

  // Determine the turn direction based on PID output
  if (pid_output > 0) {
    // Turn left since the car is too far to the right
    Serial.println("Turning Left based on PID output");
    turnRight(150);  // Adjust speed as needed
    delay(10);    // Delay to avoid overwhelming the loop
    stopCar();
  } else if (pid_output < 0) {
    // Turn right since the car is too far to the left
    Serial.println("Turning Right based on PID output");
      // Adjust speed as needed
    turnLeft(150);
    delay(10);    // Delay to avoid overwhelming the loop
    stopCar();
  } else {
    // Move forward if the car is centered
    Serial.println("Moving Forward, car aligned with the line");
    // backward(120);  // Adjust speed as needed
  }
  
  delay(10);  // Small delay to avoid overwhelming the loop
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

void forward(int speed) {
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
