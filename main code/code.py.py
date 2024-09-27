import cv2
import numpy as np
import time
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
from time import sleep

# Define functions for motor and arm operations
def motor_run():
    GPIO.setmode(GPIO.BOARD)

    Motor1A = 16
    Motor1B = 18
    Motor1E = 22

    Motor2A = 23
    Motor2B = 21
    Motor2E = 19

    GPIO.setup(Motor1A, GPIO.OUT)
    GPIO.setup(Motor1B, GPIO.OUT)
    GPIO.setup(Motor1E, GPIO.OUT)

    GPIO.setup(Motor2A, GPIO.OUT)
    GPIO.setup(Motor2B, GPIO.OUT)
    GPIO.setup(Motor2E, GPIO.OUT)

    print("Going forwards")
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor1E, GPIO.HIGH)

    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.HIGH)

    sleep(4)

    print("Now stop")
    GPIO.output(Motor1E, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.LOW)

    GPIO.cleanup()

    # Motor control code
    pass

def arm_run():
    # arm control code
      GPIO.setmode(GPIO.BOARD)
 
    Motor3A = 36
    Motor3B = 38
    Motor3E = 40

    Motor4A = 33
    Motor4B = 35
    Motor4E = 37


    GPIO.setup(Motor3A,GPIO.OUT)
    GPIO.setup(Motor3B,GPIO.OUT)
    GPIO.setup(Motor3E,GPIO.OUT)
 
    GPIO.setup(Motor4A,GPIO.OUT)
    GPIO.setup(Motor4B,GPIO.OUT)
    GPIO.setup(Motor4E,GPIO.OUT)

    print("ARM MOVING DOWN")
    GPIO.output(Motor3A,GPIO.LOW)
    GPIO.output(Motor3B,GPIO.HIGH)
    GPIO.output(Motor3E,GPIO.HIGH)
    sleep(5)

    print("open gripper")
    GPIO.output(Motor4A,GPIO.HIGH)
    GPIO.output(Motor4B,GPIO.LOW)
    GPIO.output(Motor4E,GPIO.HIGH)
    sleep(4)
    
    print("close gripper")
    GPIO.output(Motor4A,GPIO.LOW)
    GPIO.output(Motor4B,GPIO.HIGH)
    GPIO.output(Motor4E,GPIO.HIGH)
    sleep(4)
    
    

    print("ARM MOVING UP")
    GPIO.output(Motor3A,GPIO.HIGH)
    GPIO.output(Motor3B,GPIO.LOW)
    GPIO.output(Motor3E,GPIO.HIGH)
    sleep(4)

    print("STOP MOVING ARM")
    GPIO.output(Motor3E,GPIO.LOW)
    
    GPIO.cleanup()
    return

    pass

# Function to read distance from ultrasonic sensor
def read_distance(trig_pin, echo_pin):
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

    GPIO.output(trig_pin, False)
    time.sleep(0.1)

    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Ultrasonic sensor pins
TRIG_PIN = 11
ECHO_PIN = 13

# Initialize LCD display
lcd = CharLCD(i2c_expander='PCF8574', address=0x3f, port=1, cols=16, rows=2)

# Initialize camera
camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# YOLO model paths and parameters
labelsPath = 'obj.names'
weightsPath = 'crop_weed_detection.weights'
configPath = 'crop_weed.cfg'
confi = 0.5
thresh = 0.5

# Load YOLO model
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
ln = net.getLayerNames()
ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]

# Main loop
while True:
    # Read distance from ultrasonic sensor
    distance = read_distance(TRIG_PIN, ECHO_PIN)

    # If an object is detected within a certain range
    if distance < 30:  # Adjust this distance as needed
        # Stop the robot
        # Your code to stop the robot goes here

        # Capture frame from camera
        ret, frame = camera.read()

        if not ret:
            break

        (H, W) = frame.shape[:2]

        # Perform object detection
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (512, 512), swapRB=True, crop=False)
        net.setInput(blob)
        layerOutputs = net.forward(ln)

        # Initialize lists for detected objects
        boxes = []
        confidences = []
        classIDs = []

        # Loop over each of the layer outputs
        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if confidence > confi:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, confi, thresh)

        if len(idxs) > 0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                label = LABELS[classIDs[i]]

                # If weed is detected, run the arm function
                if label == 'weed':
                    arm_run()
                    lcd.clear()
                    lcd.write_string("Weed Detected")
                    # Additional actions if needed
                # If crop is detected, run the motor function
                elif label == 'crop':
                    motor_run()
                    lcd.clear()
                    lcd.write_string("Crop Detected")
                    # Additional actions if needed

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
GPIO.cleanup()
camera.release()
cv2.destroyAllWindows()
print("[STATUS]   : Completed")
print("[END]")
 