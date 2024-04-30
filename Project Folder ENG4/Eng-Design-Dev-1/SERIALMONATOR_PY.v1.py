import time
from pyfirmata import Arduino, SERVO

# Arduino board and serial port settings
board = Arduino('COM18')

# Servo objects for pan and tilt
servo_pan_pin = 9
servo_tilt_pin = 10
servo_pan = board.get_pin('d:{}:s'.format(servo_pan_pin))
servo_tilt = board.get_pin('d:{}:s'.format(servo_tilt_pin))

# Servo angle limits
pan_min_angle = 80
pan_max_angle = 180
tilt_min_angle = 140
tilt_max_angle = 90

# Main loop
while True:
    # Check if data is available from the serial port (Arduino)
    if board.digital[servo_pan_pin].read():
        # Read the data from the digital pin connected to the servo
        input_str = board.digital[servo_pan_pin].read()

        # Perform actions based on received input
        if input_str == 1:
            servo_pan.write(pan_min_angle)
        elif input_str == 2:
            servo_pan.write(pan_max_angle)
        elif input_str == 3:
            servo_tilt.write(tilt_min_angle)
        elif input_str == 4:
            servo_tilt.write(tilt_max_angle)
