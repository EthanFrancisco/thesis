#   sudo apt-get update
#   sudo apt-get upgrade
#
#   sudo apt-get install python3 python3-pip
#   sudo apt-get install python3-opencv
#   sudo apt-get install python3-picamera
#   
#   pip3 install ultralytics
#
#   pip3 install pyserial

from ultralytics import YOLO
import cv2
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import serial

'''
// Arduino code
const int motorPin1 = 9; // Motor driver pin 1
const int motorPin2 = 10; // Motor driver pin 2

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the incoming command

    if (command == 'F') {
      // Move forward
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } else if (command == 'B') {
      // Move backward
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    } else if (command == 'S') {
      // Stop
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
    }
  }
}
'''

'''
# Python code
# Set up the serial connection to the Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for the connection to initialize

def send_command(command):
    arduino.write(command.encode())

try:
    while True:
        # Example commands to control the motor
        send_command('F')  # Move forward
        time.sleep(2)
        send_command('S')  # Stop
        time.sleep(1)
        send_command('B')  # Move backward
        time.sleep(2)
        send_command('S')  # Stop
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    arduino.close()
'''

# Initialize Pi Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

# Load YOLO model
model = YOLO("yolo-Weights/yolov8n.pt")

# Object classes (currently empty)
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"]

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	img = frame.array

	results = model(img, stream=True)

	# Process results
	for r in results:
		boxes = r.boxes

		for box in boxes:
			# Bounding box coordinates
			x1, y1, x2, y2 = box.xyxy[0]
			x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

			# Draw bounding box on the image
			cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

			# Calculate and print confidence
			confidence = math.ceil((box.conf[0] * 100)) / 100
			print("Confidence --->", confidence)

	# Display the image
	cv2.imshow("Image", img)

	# Clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# Break loop on 'q' key press
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release resources
cv2.destroyAllWindows()