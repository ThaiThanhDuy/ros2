import smbus
import time

# BNO055 I2C address
BNO055_ADDRESS = 0x28

# BNO055 register addresses
BNO055_REG_CHIP_ID = 0x00
BNO055_REG_EULER_H = 0x1A  # Start of Euler angles

# Initialize I2C bus
bus = smbus.SMBus(1)  # Use 1 for Raspberry Pi 2 and later

# Function to read a byte from a register
def read_byte(register):
    return bus.read_byte_data(BNO055_ADDRESS, register)

# Function to read 6 bytes of data (Euler angles)
def read_euler():
    euler_data = bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_REG_EULER_H, 6)
    heading = (euler_data[0] | (euler_data[1] << 8)) / 16.0
    roll = (euler_data[2] | (euler_data[3] << 8)) / 16.0
    pitch = (euler_data[4] | (euler_data[5] << 8)) / 16.0
    return heading, roll, pitch

# Main program
try:
    # Check the chip ID
    chip_id = read_byte(BNO055_REG_CHIP_ID)
    print(f"BNO055 Chip ID: {chip_id}")

    while True:
        heading, roll, pitch = read_euler()
        print(f"Orientation: Heading: {heading:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by User")
except Exception as e:
    print(f"Error: {e}")
