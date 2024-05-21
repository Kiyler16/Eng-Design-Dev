import cv2
import serial
import time

# Connect to Arduino serial port
arduino = serial.Serial('COM18', 9600)  # Adjust port accordingly

def send_position(x, y):
    # Send servo positions to Arduino
    arduino.write(f"{x} {y}\n".encode())

def move_servos(current_x, current_y, target_x, target_y, slow_speed=0.1, fast_speed=0.1):
    # Calculate the difference between current and target positions
    dx = target_x - current_x
    dy = target_y - current_y

    # Adjust speed based on the distance to the target
    speed_option = fast_speed
    if abs(dx) < 10 and abs(dy) < 10:
        speed_option = slow_speed  # Slow down when close to the target
    

    # Move the servos towards the target position
    new_x = current_x + int(dx * speed_option)
    new_y = current_y + int(dy * speed_option)
    
    # Constrain servo positions to valid range (0-180)                BOTH NEED EDIT
    new_x = max(0, min(180, new_x))
    new_y = max(0, min(180, new_y))

    return new_x, new_y

def track_faces():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)  # Use the default camera
    time.sleep(2)  # Allow the camera to warm up

    current_x, current_y = 90, 90  # Initial servo positions

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            x_center = x + w // 2
            y_center = y + h // 2
            
            # Move the servos towards the detected face
            current_x, current_y = move_servos(current_x, current_y, x_center, y_center, slow_speed=1, fast_speed=1)

            # Send servo positions to Arduino
            send_position(current_x, current_y)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)

        cv2.imshow('Face Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_faces()