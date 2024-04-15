import cv2
import numpy as np
import serial
import time

# Define serial port settings
SERIAL_PORT = 'COM15'  # Adjust the serial port accordingly
BAUD_RATE = 9600

# Define serial communication protocol
MESSAGE_START = b'START'
MESSAGE_END = b'END'

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def send_command_to_arduino(command):
    # Send command to Arduino
    ser.write(MESSAGE_START + command.encode() + MESSAGE_END)
    print(f"Sent command to Arduino: {command}")

def receive_response_from_arduino():
    # Read response from Arduino
    response = ser.read_until(MESSAGE_END)
    return response.strip(MESSAGE_END)

# Function to detect yellow object and calculate centroid
def detect_yellow_object(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding box around the largest contour and calculate centroid
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Calculate centroid of the bounding box
        centroid_x = x + (w // 2)
        centroid_y = y + (h // 2)
        cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

        return centroid_x, centroid_y

    return None

# Function to control pan and tilt servos
def control_pan_tilt(centroid_x, centroid_y, frame_width, frame_height):
    # Define servo control parameters
    pan_angle = map_to_range(centroid_x, 0, frame_width, 0, 180)
    tilt_angle = map_to_range(centroid_y, 0, frame_height, 180, 0)

    # Print debug information
    print(f"Centroid: ({centroid_x}, {centroid_y})")
    print(f"Pan Angle: {pan_angle}, Tilt Angle: {tilt_angle}")

    # Send pan and tilt angles to Arduino
    send_command_to_arduino(f"PAN{int(pan_angle)}")
    send_command_to_arduino(f"TILT{int(tilt_angle)}")

# Function to map a value from one range to another
def map_to_range(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

# Main function
def main():
    # Open video capture device
    cap = cv2.VideoCapture(1)

    while True:
        # Read a frame from the video capture device
        ret, frame = cap.read()
        if not ret:
            break

        # Detect yellow object and calculate centroid
        yellow_centroid = detect_yellow_object(frame)

        # If yellow object is detected, control pan and tilt servos
        if yellow_centroid:
            centroid_x, centroid_y = yellow_centroid
            frame_height, frame_width = frame.shape[:2]
            control_pan_tilt(centroid_x, centroid_y, frame_width, frame_height)

        # Display the frame
        cv2.imshow('Yellow Object Detection and Servo Control', frame)

        # Break loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture device and close serial port
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
