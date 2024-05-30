import cv2
import serial
import time

# Connect to Arduino serial port
arduino = serial.Serial('COM18', 9600)  # Adjust port accordingly

# ------------ PID Configuration -------------
# You must fine tune these variables to adjust the PID algorithm, until it works properly.
#
# Kp = Proportional gain
# Ki = Integral gain
# Kd = Derivative gain
#
# Simple process for tuning PID: 
#   1. Place an object in front of the camera so that the camera detects it. Do not move the object.
#   2. Increase Kp until your servos oscillate around the object.
#       For example, the servos should keep over shooting the object back and forth, back and forth.
#   3. Increase Ki until the servos quickly find the object, and stop near the object, but not necessarily exactly ontop of the object.
#       For example, the servos stop moving, but they overshoot the object by a few degrees.
#   4. Increase Kd until the servos quickly find the object and stop on the object, nearly perfectly.
x_Kp = 50
x_Ki = 50
x_Kd = 50

y_Kp = 50
y_Ki = 50
y_Kd = 50

class PidController(object):
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_sum = 0
        self.previous_error = 0
        self.previous_time = time.time() 

    def calculate_pid(self, goal_camera_position, current_camera_position):
        current_time = time.time()
        print(current_time)
        elapsed_time = current_time - self.previous_time 

        error = goal_camera_position - current_camera_position

        self.integral_sum = self.integral_sum + (error * elapsed_time)

        derivative = (error - self.previous_error) // elapsed_time

        self.previous_error = error
        self.previous_time = current_time

        return (error * self.Kp) + (derivative * self.Kd) + (self.integral_sum * self.Ki)
        

def send_position(x, y):
    # Send servo positions to Arduino
    arduino.write(f"{x} {y}\n".encode())

def track_faces():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)  # Use the default camera

    # Initial servo positions
    previous_x, previous_y = 90, 90 
    # Center the servos
    send_position(previous_x, previous_y)

    time.sleep(2)  # Allow the camera to warm up, and the servos to center

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_height, frame_width = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        x_pid_controller = PidController(x_Kp, x_Ki, x_Kd)
        y_pid_controller = PidController(y_Kp, y_Ki, y_Kd)

        for (x, y, w, h) in faces:
            contour_center_x = x + w // 2
            contour_center_y = y + h // 2
            
            x_pid_value = x_pid_controller.calculate_pid(frame_width // 2, contour_center_x)
            y_pid_value = y_pid_controller.calculate_pid(frame_height // 2, contour_center_y)

            # Move the servos towards the detected face
            current_x = max(0, min(180, previous_x + x_pid_value))
            current_y = max(0, min(180, previous_y + y_pid_value))

            # Send servo positions to Arduino
            send_position(current_x, current_y)

            # Remember the current positions
            previous_x = current_x
            previous_y = current_y

            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (contour_center_x, contour_center_y), 5, (0, 0, 255), -1)

        cv2.imshow('Face Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_faces()