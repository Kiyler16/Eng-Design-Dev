import serial
import time

# Define the serial port and baud rate
serial_port = 'COM16'  # Replace 'COMX' with the appropriate serial port of your Arduino Uno
baud_rate = 9600

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

# Wait for the serial connection to be established
time.sleep(2)

# Send command to Arduino
command = b'on\n'
ser.write(command)

# Close the serial port
ser.close()