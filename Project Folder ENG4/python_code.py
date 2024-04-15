import cv2
import numpy as np
import serial
import time

# Open a serial connection to Arduino
arduino = serial.Serial('COM18', 9600)  # Change 'COM3' to your Arduino port
time.sleep(2)  # Allow time for Arduino to reset

# Open a connection to the webcam (0 represents the default camera)
cap = cv2.VideoCapture(0)

# Initialize variables for the contour with the highest contrast
max_contrast_contour = None
max_contrast_coords = None
max_contrast = -1  # Initialize to a negative value

# Initialize pan/tilt position
pan_position = 90
tilt_position = 90

# Function to move towards the object
def move_towards_object(x, y):
    global pan_position, tilt_position
    center_x = 320  # Assuming a 640x480 image
    center_y = 240
    pan_adjust = int((x - center_x) / 5)  # Adjust based on your servo and image dimensions
    tilt_adjust = int((y - center_y) / 5)

    pan_position += pan_adjust
    tilt_position += tilt_adjust

    # Limit pan/tilt angles to avoid going beyond servo limits
    pan_position = max(0, min(180, pan_position))
    tilt_position = max(0, min(180, tilt_position))

    # Send pan/tilt commands to Arduino
    arduino.write(f"P{pan_position}\n".encode())
    arduino.write(f"T{tilt_position}\n".encode())

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()

    # Convert the frame from BGR to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the green color in HSV
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # Create a mask for the green color
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Reset variables for each frame
    max_contrast_contour = None
    max_contrast_coords = None
    max_contrast = -1

    for contour in contours:
        # Calculate the area of the bounding rectangle around the contour
        x, y, w, h = cv2.boundingRect(contour)
        contour_area = cv2.contourArea(contour)

        # Calculate the intensity or saturation of the region
        intensity = contour_area / (w * h)

        # Check if the current contour has higher contrast
        if intensity > max_contrast:
            max_contrast = intensity
            max_contrast_contour = contour
            max_contrast_coords = (x, y, w, h)

    # Draw bounding box and center coordinates if a contour with high contrast is found
    if max_contrast_contour is not None:
        x, y, w, h = max_contrast_coords
        center_x = x + w // 2
        center_y = y + h // 2

        move_towards_object(center_x, center_y)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.putText(frame, f"Center: ({center_x}, {center_y})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the resulting frame and a contrast window
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Contrast Window', mask)

    # Break the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam, close all windows, and close the serial connection
cap.release()
cv2.destroyAllWindows()
arduino.close()