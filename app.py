import cv2 
import serial
import time

# Initialize serial communication with Arduino (adjust the COM port and baud rate)
try:
    arduino = serial.Serial('COM5', 9600, timeout=1)
    time.sleep(2)  # Give some time to establish the connection
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)  # Exit if unable to open the port

# Load the pre-trained car classifier (Haar cascade)
car_cascade = cv2.CascadeClassifier('D:/14ecs/models/haarcascade_car.xml')

# Check if the classifier loaded successfully
if car_cascade.empty():
    print("Error loading Haar cascade. Please check the file path.")
    exit(1)

# Capture video from webcam or a video file
cap = cv2.VideoCapture(0)  # Change '0' to the video file path if using a file

car_detected = False           # Flag to indicate car detection
green_light_start_time = 0     # Variable to store when the green light turned on
frames_with_cars = 0           # Counter for frames with car detection

# Turn on the red light initially
arduino.write(b'0')
print("Initial state: Red light ON")

car_width_min = 50  # Adjust these values as needed
car_height_min = 20

while True:
    # Read a frame from the video capture
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect vehicles in the frame
    cars = car_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Filter detected cars based on size
    valid_cars = []
    for (x, y, w, h) in cars:
        print(f"Detected object with width: {w}, height: {h}")  # Log dimensions
        if w > car_width_min and h > car_height_min:  # Filter based on size
            valid_cars.append((x, y, w, h))

    # Draw rectangles around valid detected vehicles
    if valid_cars:
        frames_with_cars += 1  # Increment counter if valid cars are detected
        for (x, y, w, h) in valid_cars:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Check for continuous car detection and light control logic
    if frames_with_cars >= 3 and not car_detected:
        arduino.write(b'1')  # Signal Arduino to turn the green light on
        print("Green light ON for 10 seconds")  # Updated message
        car_detected = True
        green_light_start_time = time.time()

    if car_detected and (time.time() - green_light_start_time >= 10):  # Changed to 10 seconds
        arduino.write(b'0')  # Signal Arduino to turn the red light on
        print("Red light ON after 10 seconds")  # Updated message
        car_detected = False
        frames_with_cars = 0

    # Display the video with detection
    cv2.imshow('Vehicle Detection', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arduino.close()
