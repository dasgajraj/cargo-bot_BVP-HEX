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

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  Serial.begin(9600);
}

void loop() {
  long distance = measureDistance();

  bool irObstacle1 = digitalRead(IR_SENSOR_1);
  bool irObstacle2 = digitalRead(IR_SENSOR_2);

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
    Serial.println("Object within range (Ultrasonic), moving forward.");
  } 
  else {
    stopMotors();
    stopServo();
    Serial.println("Nothing in front, stopping.");
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
