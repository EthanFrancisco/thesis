/*
  Ultrasonic Sensor1 Pins = (U1TXp, 8), (URXp, 7)
  Ultrasonic Sensor2 Pins = (U2TXp, 5), (URXp, 4)
  GPS Sensor Pins = (GPSTXp, 3), (GPSRXp, 2)

  LM Driver pins = (RPWM, 11), (LPWM, 10)
  RM Driver pins = (RPWM, 9), (LPWM, 6)
*/


#include <SoftwareSerial.h>

//SoftwareSerial ss(3, 2);  // RX, TX

SoftwareSerial sensor1Serial(12, 8);  // RX, TX for sensor 1 UP
SoftwareSerial sensor2Serial(7, 5);   // RX, TX for sensor 2 DOWN

const int motor1RPWM = 11;
const int motor1LPWM = 10;
const int motor2RPWM = 9;
const int motor2LPWM = 6;

unsigned char data1[4] = {};
unsigned char data2[4] = {};

float distance1;
float distance2;
int heading;
String detection;
int targetAngle;
int angleDifference;
String turnDirection;

void setup() {
  Serial.begin(57600);
  sensor1Serial.begin(9600);
  sensor2Serial.begin(9600);

  pinMode(motor1RPWM, OUTPUT);
  pinMode(motor1LPWM, OUTPUT);
  pinMode(motor2RPWM, OUTPUT);
  pinMode(motor2LPWM, OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(A5, OUTPUT);
}


int calculateAngleDifference(int currentAngle, int targetAngle) {
  int difference = targetAngle - currentAngle;
  if (difference < 0) {
    difference += 360;
  }
  return difference;
}

String determineTurnDirection(int currentAngle, int targetAngle) {
  int difference = calculateAngleDifference(currentAngle, targetAngle);
  if (difference <= 180) {
    return "Right";
  } else {
    return "Left";
  }
}

bool readSensor(SoftwareSerial &sensorSerial, unsigned char *data) {
  int bytesRead = 0;
  unsigned long startTime = millis();
  while (bytesRead < 4 && (millis() - startTime) < 1000) {
    if (sensorSerial.available()) {
      data[bytesRead++] = sensorSerial.read();
    }
  }
  return bytesRead == 4;
}

void processSensorData(unsigned char *data, float &distance, const char *sensorName) {
  if (data[0] == 0xff) {
    int sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3]) {
      distance = (data[1] << 8) + data[2];
      if (distance > 30) {
        Serial.print(sensorName);
        Serial.print(" distance=");
        Serial.print(distance / 10);
        Serial.println("cm");
      } else {
        Serial.print(sensorName);
        Serial.println(": Below the lower limit");
      }
    } else {
      Serial.print(sensorName);
      Serial.println(": ERROR - Checksum mismatch");
    }
  } else {
    Serial.print(sensorName);
    Serial.println(": ERROR - Invalid start byte");
  }
}

void motorForward(int motorLPWM, int motorRPWM, int speed) {
  analogWrite(motorLPWM, 0);
  analogWrite(motorRPWM, speed);
}

void motorBackward(int motorLPWM, int motorRPWM, int speed) {
  analogWrite(motorLPWM, speed);
  analogWrite(motorRPWM, 0);
}

void splitString(const String &input, char marker, String parts[], int maxParts) {
  int partIndex = 0;
  int startIndex = 0;
  int endIndex = input.indexOf(marker);

  while (endIndex != -1 && partIndex < maxParts) {
    parts[partIndex++] = input.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
    endIndex = input.indexOf(marker, startIndex);
  }

  // Add the last part
  if (partIndex < maxParts) {
    parts[partIndex] = input.substring(startIndex);
  }
}


void loop() {
  /*
    // Read and process data from sensor 1
    sensor1Serial.listen();
    if (readSensor(sensor1Serial, data1)) {
        processSensorData(data1, distance1, "Sensor 1");
    } else {
        Serial.println("Sensor 1: ERROR - Not enough data available");
    }

    delay(50); // Add a delay to make the output readable

    // Read and process data from sensor 2
    sensor2Serial.listen();
    if (readSensor(sensor2Serial, data2)) {
        processSensorData(data2, distance2, "Sensor 2");
    } else {
        Serial.println("Sensor 2: ERROR - Not enough data available");
    }

    delay(50); // Add a delay to make the output readable

    // Read and print GPS data
    gpsSerial.listen();
    readGPSData();

    delay(50); // Add a delay to make the output readable

    // Example motor control
    motorForward(motor1LPWM, motor1RPWM, 80); // Motor 1 forward at full speed
    motorBackward(motor2LPWM, motor2RPWM, 80); // Motor 2 backward at full speed

    delay(50); // Run motors for 2 seconds

    motorForward(motor1LPWM, motor1RPWM, 0); // Stop Motor 1
    motorBackward(motor2LPWM, motor2RPWM, 0); // Stop Motor 2

    delay(50); // Wait for 2 seconds
  */

  //normal forward
  //motorBackward(motor1LPWM, motor1RPWM, 30);
  //motorBackward(motor2LPWM, motor2RPWM, 30);

  // rigth turn
  //   motorForward(motor2LPWM, motor2RPWM, 120);
  //   motorForward(motor1LPWM, motor1RPWM, 0);
  // left
  //   motorForward(motor1LPWM, motor1RPWM, 120);
  //   motorForward(motor2LPWM, motor2RPWM, 0);


  // Read and process data from sensor 1

  // Read and process data from sensor 2


  //read serial from pi
  // Read and process data from sensor 1
  /*
    sensor1Serial.listen();
    if (readSensor(sensor1Serial, data1)) {
      processSensorData(data1, distance1, "Sensor 1");
    } else {
      Serial.println("Sensor 1: ERROR - Not enough data available");
    }

    delay(10); // Add a delay to make the output readable

    // Read and process data from sensor 2
    sensor2Serial.listen();
    if (readSensor(sensor2Serial, data2)) {
      processSensorData(data2, distance2, "Sensor 2");
    } else {
      Serial.println("Sensor 2: ERROR - Not enough data available");
    }

    delay(10); // Add a delay to make the output readable
  */

  if (Serial.available() > 0) {
    // Read the incoming data
    String incomingData = Serial.readStringUntil('\n');
    // Print the incoming data to the Serial Monitor
    /*
        if (incomingData == "B") {
          motorForward(motor2LPWM, motor2RPWM, 120);
          motorForward(motor1LPWM, motor1RPWM, 0);
        } else if (incomingData == "C") {
          motorForward(motor2LPWM, motor2RPWM, 0);
          motorForward(motor1LPWM, motor1RPWM, 120);
        } else if (incomingData != "A") {
          motorBackward(motor2LPWM, motor2RPWM, 30);
          motorBackward(motor1LPWM, motor1RPWM, 30);
        } else {
          motorForward(motor2LPWM, motor2RPWM, 0);
          motorForward(motor1LPWM, motor1RPWM, 0);
        }

      }*/

    char marker = '|';
    const int maxParts = 10;
    String parts[maxParts];

    splitString(incomingData, marker, parts, maxParts);

    detection  = parts[0];
    heading = parts[1].toInt();
    targetAngle = 180;
    angleDifference = calculateAngleDifference(heading, targetAngle);
    turnDirection = determineTurnDirection(heading, targetAngle);

    bool heading_correct = heading >= 178 && heading <= 193;

    if (heading_correct) {
      motorBackward(motor1LPWM, motor1RPWM, 30);
      motorBackward(motor2LPWM, motor2RPWM, 30);
    } else if (turnDirection == "Right") {
      motorForward(motor2LPWM, motor2RPWM, 200);
      motorForward(motor1LPWM, motor1RPWM, 0);
    } else if (turnDirection == "Left") {
      motorForward(motor1LPWM, motor1RPWM, 200);
      motorForward(motor2LPWM, motor2RPWM, 0);
    }



    /*
      if (heading_correct && detection == "A")
      {
      motorBackward(motor1LPWM, motor1RPWM, 30);
      motorBackward(motor2LPWM, motor2RPWM, 30);
      } else if (turnDirection == "Right" && detection == "A") {
      motorForward(motor2LPWM, motor2RPWM, 200);
      motorForward(motor1LPWM, motor1RPWM, 0);
      } else if (turnDirection == "Left" && detection == "A") {
      motorForward(motor1LPWM, motor1RPWM, 200);
      motorForward(motor2LPWM, motor2RPWM, 0);
      } else {
      motorBackward(motor1LPWM, motor1RPWM, 0);
      motorBackward(motor2LPWM, motor2RPWM, 0);
      }*/
  }
}
