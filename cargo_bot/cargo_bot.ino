// Define motor control pins
#define ENA 9
#define IN1 2
#define IN2 3
#define ENB 10
#define IN3 4
#define IN4 5

// Define ultrasonic sensor pins
#define TRIG_PIN 7
#define ECHO_PIN 8

// Define ultrasonic sensor for fall detection
#define TRIG_PIN_2 6
#define ECHO_PIN_2 A0

// Define IR sensor pins
#define IR_SENSOR_1 11
#define IR_SENSOR_2 12

// Define buzzer pin
#define BUZZER_PIN 13

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Set ultrasonic sensor pins for fall detection
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  // Set IR sensor pins as inputs
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);

  // Set buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

  stopBuzzer(); // Make sure the buzzer is off when starting
}

void loop() {
  // Get the distance from both ultrasonic sensors
  long distance = measureDistance(TRIG_PIN, ECHO_PIN);

  long groundDistance = measureDistance(TRIG_PIN_2, ECHO_PIN_2);  // Ground distance (fall detection)
  // Fall detection: stop immediately if distance from ground is >= 8 cm
  if (groundDistance >= 15) {
    stopMotors(); // Stop to prevent falling immediately
    soundContinuousBuzzer(); // Continuous sound for fall detection
    return;       // Skip other logic when a fall is detected
  }

  // Read IR sensors
  bool irObstacle1 = digitalRead(IR_SENSOR_1);
  bool irObstacle2 = digitalRead(IR_SENSOR_2);

  // Debugging output to check the values of the IR sensors
  Serial.print("IR Sensor 1: ");
  Serial.println(irObstacle1);
  Serial.print("IR Sensor 2: ");
  Serial.println(irObstacle2);

  // Check if both IR sensors detect an obstacle
  if (irObstacle1 == LOW && irObstacle2 == LOW) {
    stopMotors(); // Stop if both IR sensors detect an obstacle
    soundBuzzer(); // Sound buzzer as obstacle is detected
  }
  // Check if the left IR sensor detects an obstacle
  else if (irObstacle1 == LOW) {
   //turnRight(); // If left sensor detects an obstacle, turn right
    turnLeft(); // If right sensor detects an obstacle, turn left
    soundBuzzer(); // Sound buzzer as obstacle is detected
  } 
  // Check if the right IR sensor detects an obstacle
  else if (irObstacle2 == LOW) {
    turnRight(); // If left sensor detects an obstacle, turn right

    soundBuzzer(); // Sound buzzer as obstacle is detected
  }
  // Check the distance using the front ultrasonic sensor
  else if (distance < 20) {
    stopMotors(); // Stop if the object is too close
    soundBuzzer(); // Sound buzzer as obstacle is detected
  } 
  else if (distance < 100) {
    moveForward(); // Move forward if object is within range
    stopBuzzer(); // Stop buzzer as no obstacle is nearby
  } 
  else {
    stopMotors(); // Stop if no object is detected
    stopBuzzer(); // Stop buzzer
  }
}

// Function to measure distance using ultrasonic sensor
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Function to move the robot forward
void moveForward() {
  // Motor 1
  digitalWrite(IN1, HIGH);  // Set Motor 1 to move forward
  digitalWrite(IN2, LOW);   // Ensure reverse is off
  analogWrite(ENA, 255);    // Full speed for Motor 1

  // Motor 2
  digitalWrite(IN3, HIGH);  // Set Motor 2 to move forward
  digitalWrite(IN4, LOW);   // Ensure reverse is off
  analogWrite(ENB, 255);    // Full speed for Motor 2
}

// Function to stop the motors
void stopMotors() {
  analogWrite(ENA, 0); // Stop Motor 1
  analogWrite(ENB, 0); // Stop Motor 2
}

// Function to turn left (by stopping right motor and moving left motor)
void turnLeft() {
  digitalWrite(IN1, HIGH);  // Motor 1 forward
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);    // Full speed

  digitalWrite(IN3, HIGH);   // Motor 2 stop
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 50);      // Stop motor
}

// Function to turn right (by stopping left motor and moving right motor)
void turnRight() {
  digitalWrite(IN1, HIGH);   // Motor 1 stop
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 50);      // Stop motor

  digitalWrite(IN3, HIGH);  // Motor 2 forward
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);    // Full speed
}

// Function to sound the buzzer with beeping pattern
void soundBuzzer() {
  tone(BUZZER_PIN, 4000); // Beep at 4000 Hz frequency
  delay(100);  // Short delay to simulate beep sound
  noTone(BUZZER_PIN); // Stop the tone to create beep effect
  delay(50);  // Pause between beeps
}

// Function to sound the buzzer continuously (for fall detection)
void soundContinuousBuzzer() {
  tone(BUZZER_PIN, 4000); // Continuous beep at 4000 Hz frequency
}

// Function to stop the buzzer
void stopBuzzer() {
  noTone(BUZZER_PIN); // Stop the buzzer sound
}
