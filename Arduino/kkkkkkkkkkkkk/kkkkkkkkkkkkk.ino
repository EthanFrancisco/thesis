#include <Arduino.h>
#include <Ultrasonic.h>

// Define motor pins
#define VACUUM_MOTOR_PIN  5
#define CONSTANT_MOTOR1_PIN  18
#define CONSTANT_MOTOR2_PIN  19
#define LEFT_MOTOR_PIN  21
#define RIGHT_MOTOR_PIN  22

// Define ultrasonic sensor pins
#define ULTRASONIC_TRIG1  23
#define ULTRASONIC_ECHO1  34
#define ULTRASONIC_TRIG2  25
#define ULTRASONIC_ECHO2  35
#define ULTRASONIC_TRIG3  26
#define ULTRASONIC_ECHO3  32
#define ULTRASONIC_TRIG4  27
#define ULTRASONIC_ECHO4  33

// Define IR sensor pin
#define IR_SENSOR_PIN  4

// Create ultrasonic sensor objects
Ultrasonic ultrasonic1(ULTRASONIC_TRIG1, ULTRASONIC_ECHO1);
Ultrasonic ultrasonic2(ULTRASONIC_TRIG2, ULTRASONIC_ECHO2);
Ultrasonic ultrasonic3(ULTRASONIC_TRIG3, ULTRASONIC_ECHO3);
Ultrasonic ultrasonic4(ULTRASONIC_TRIG4, ULTRASONIC_ECHO4);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(VACUUM_MOTOR_PIN, OUTPUT);
  pinMode(CONSTANT_MOTOR1_PIN, OUTPUT);
  pinMode(CONSTANT_MOTOR2_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);

  // Initialize IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);
}

void loop() {
  // Read ultrasonic sensor distances
  long distance1 = ultrasonic1.read();
  long distance2 = ultrasonic2.read();
  long distance3 = ultrasonic3.read();
  long distance4 = ultrasonic4.read();

  // Read IR sensor value
  int irValue = digitalRead(IR_SENSOR_PIN);

  // Print sensor values to serial monitor
  Serial.print("Ultrasonic 1: ");
  Serial.print(distance1);
  Serial.print(" cm, Ultrasonic 2: ");
  Serial.print(distance2);
  Serial.print(" cm, Ultrasonic 3: ");
  Serial.print(distance3);
  Serial.print(" cm, Ultrasonic 4: ");
  Serial.print(distance4);
  Serial.print(" cm, IR Sensor: ");
  Serial.println(irValue);

  // Control motors based on sensor values
  if (irValue == HIGH) {
    digitalWrite(VACUUM_MOTOR_PIN, HIGH);
  } else {
    digitalWrite(VACUUM_MOTOR_PIN, LOW);
  }

  // Example motor control logic
  digitalWrite(CONSTANT_MOTOR1_PIN, HIGH);
  digitalWrite(CONSTANT_MOTOR2_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PIN, LOW);

  // Add your own motor control logic here

  // Delay for a short period
  delay(100);
}
