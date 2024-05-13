import serial
import time

# Configure the serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust the port and baud rate as needed

while True:
    # Define the data to send
    data = bytearray([ord('m'), ord('1'), 100, 200])
    
    # Send data over serial
    ser.write(data)
    
    # Delay for a short time
    time.sleep(1)  # Adjust delay as needed
    print("Data Sent")
