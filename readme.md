# 🤖 CARGO BOT 🚀

## 🎯 Overview
CARGO BOT is an Arduino-based intelligent robot designed to detect obstacles, prevent falls, and navigate efficiently while carrying attached props like bags, laptops, luggage, wheelchairs, and trolleys. It offers a versatile solution for automated object transportation and movement.

## 💡 Unique Selling Point (USP)
The bot can be attached to various props, ensuring safe and hassle-free mobility by autonomously navigating around obstacles and preventing potential falls.

## ✨ Features

1. **🕵️ Obstacle Detection**
   - IR sensors detect obstacles in left and right directions
   - Front ultrasonic sensor detects objects ahead

2. **🛡️ Fall Prevention**
   - Ground ultrasonic sensor measures distance to prevent falling
   - Stops immediately when ground distance exceeds safe threshold
   - Triggers continuous buzzer alert during fall risk scenarios

3. **🏎️ Precise Motor Control**
   - Dual motor system enables:
     - Forward motion
     - Left turns
     - Right turns
     - Complete stops

4. **🔊 Audible Alerts**
   - Buzzer notifications for:
     - Obstacle detection
     - Fall prevention
   - Distinct sound patterns for different scenarios

## 🛠️ Hardware Requirements

| Component | Quantity | Specifications |
|-----------|----------|----------------|
| Microcontroller | 1 | Arduino Uno |
| Ultrasonic Sensors | 2 | Distance detection |
| IR Sensors | 2 | Obstacle detection |
| DC Motors | 2 | With motor driver module |
| Buzzer | 1 | Audible alerts |
| Power Supply | 1 | Compatible with Arduino and motors |

## 🔌 Detailed Pin Configuration

### Motor Control
| Pin Name | Arduino Pin |
|----------|-------------|
| ENA | Pin 9 |
| IN1 | Pin 2 |
| IN2 | Pin 3 |
| ENB | Pin 10 |
| IN3 | Pin 4 |
| IN4 | Pin 5 |

### Sensor Pins
| Sensor | TRIG PIN | ECHO PIN |
|--------|----------|----------|
| Front Sensor | Pin 7 | Pin 8 |
| Ground Sensor | Pin 6 | Pin A0 |

### Additional Pins
| Component | Pin |
|-----------|-----|
| IR Sensor 1 | Pin 11 |
| IR Sensor 2 | Pin 12 |
| Buzzer | Pin 13 |

## 💻 Software Requirements

| Requirement | Details |
|-------------|---------|
| IDE | Arduino IDE |
| Debugging Tool | Serial Monitor |

## 🚀 Installation and Setup
1. Clone the repository
2. Open the code in Arduino IDE
3. Connect Arduino board
4. Verify and upload code

## 🎮 Usage
1. Attach CARGO BOT to desired prop
2. Power on the device
3. Robot autonomously navigates, avoiding obstacles and preventing falls

## 🐞 Debugging
- Use Serial Monitor in Arduino IDE to:
  - Check sensor readings
  - Monitor obstacle detection status

## 🚧 Future Enhancements
1. Bluetooth/Wi-Fi remote control integration
2. Camera module for visual object recognition
3. GPS tracking for location-based navigation

## 📝 Code Overview
The project uses multiple functions for:
- Distance measurement
- Motor control (forward, stop, left turn, right turn)
- Obstacle and fall detection
- Buzzer alerts

## </> Code
```bash
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
```


## 🔍 Debugging Tips
- Monitor sensor inputs via Serial Monitor
- Verify connections match pin configurations
- Test each component independently

## ⚠️ Limitations
- Requires line-of-sight for obstacle detection
- Performance may vary with different surfaces and lighting conditions



---

### Created by ABBAS ALI & DAS GAJRAJ SHARMA 
