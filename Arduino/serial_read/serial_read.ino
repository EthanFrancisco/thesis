void setup() {
  pinMode(13, OUTPUT);
  pinMode(A5, OUTPUT);
  // Start the serial communication at 9600 baud rate
  Serial.begin(57600);
  // Wait for the serial port to connect. Needed for native USB port only
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming data
    String incomingData = Serial.readStringUntil('\n');
    digitalWrite(A5, 1);
  } else {
    digitalWrite(A5, 0);
  }
}
