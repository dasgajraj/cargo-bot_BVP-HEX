#include <Servo.h> 

#define ENA 9
#define IN1 2
#define IN2 3
#define ENB 10
#define IN3 4
#define IN4 5

#define TRIG_PIN 7
#define ECHO_PIN 8

#define IR_SENSOR_1 11
#define IR_SENSOR_2 12

#define SERVO_PIN 13

#define LED_LEFT 6     // Left LED pin
#define LED_RIGHT 14   // Right LED pin

Servo myServo;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  
  pinMode(LED_LEFT, OUTPUT);     // Initialize left LED pin
  pinMode(LED_RIGHT, OUTPUT);    // Initialize right LED pin
  
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  
  Serial.begin(9600);
}

void loop() {
  long distance = measureDistance();
  bool irObstacle1 = digitalRead(IR_SENSOR_1);
  bool irObstacle2 = digitalRead(IR_SENSOR_2);

  // Reset LED indication
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);

  if (irObstacle1 == LOW || irObstacle2 == LOW) {
    stopMotors();
    stopServo();
    Serial.println("Obstacle detected by IR, stopping!");
  } 
  else if (distance < 10) {
    stopMotors();
    stopServo();
    Serial.println("Object too close (Ultrasonic), stopping!");
  } 
  else if (distance < 60) {
    moveForward();
    startServo();
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, HIGH);  // Indicate moving forward
    Serial.println("Object within range (Ultrasonic), moving forward.");
  } 
  else {
    stopMotors();
    stopServo();
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);  // No movement, turn off LEDs
    Serial.println("Nothing in front, stopping.");
  }

  // Additional movement logic: Left and Right based on conditions
  if (irObstacle1 == LOW) {
    moveLeft();
    digitalWrite(LED_LEFT, HIGH);  // Indicate turning left
    Serial.println("Obstacle detected on left, turning left.");
  } 
  else if (irObstacle2 == LOW) {
    moveRight();
    digitalWrite(LED_RIGHT, HIGH); // Indicate turning right
    Serial.println("Obstacle detected on right, turning right.");
  }

  delay(100);
}

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);
}

void moveLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Reverse left motor direction
  analogWrite(ENA, 255);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);     // Right motor moving forward
}

void moveRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);    // Left motor moving forward
  analogWrite(ENA, 255);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);   // Reverse right motor direction
  analogWrite(ENB, 255);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void startServo() {
  myServo.write(90);
}

void stopServo() {
  myServo.detach();
}
