import serial

# Configure the serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust the port as needed

while True:
    # Read 4 bytes from the serial port
    data = ser.read(4)
    
    # Interpret the data
    identifier = chr(data[0]) + chr(data[1])
    value1 = data[2]
    value2 = data[3]
    
    # Print the received data
    print("Received: Identifier:", identifier, "Value1:", value1, "Value2:", value2)
