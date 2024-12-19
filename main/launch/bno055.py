import serial
import time

# Configure the serial port
SERIAL_PORT = '/dev/serial0'  # Use /dev/serial0 for Raspberry Pi
BAUD_RATE = 115200

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Function to read 6 bytes of Euler angles
def read_euler():
    # Request data from the BNO055
    ser.write(b'\x00')  # Send a command to read data (this may vary based on your setup)
    time.sleep(0.1)  # Wait for the sensor to respond
    data = ser.read(6)  # Read 6 bytes of data

    # Convert the bytes to angles
    heading = (data[0] | (data[1] << 8)) / 16.0
    roll = (data[2] | (data[3] << 8)) / 16.0
    pitch = (data[4] | (data[5] << 8)) / 16.0
    return heading, roll, pitch

# Main program
try:
    print("BNO055 Sensor Initialized")
    while True:
        heading, roll, pitch = read_euler()
        print(f"Orientation: Heading: {heading:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by User")
except Exception as e:
    print(f"Error: {e}")
finally:
    ser.close()
