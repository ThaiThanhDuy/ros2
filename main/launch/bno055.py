import smbus
import time

# BNO055 I2C address
BNO055_ADDRESS = 0x28

# BNO055 register addresses
BNO055_REG_CHIP_ID = 0x00
BNO055_REG_OPR_MODE = 0x3D
BNO055_REG_SYS_TRIGGER = 0x3F
BNO055_REG_EULER_H = 0x1A  # Start of Euler angles

# Operation modes
OPR_MODE_NDOF = 0x0C  # NDOF mode

# Initialize I2C bus
bus = smbus.SMBus(1)  # Use 1 for Raspberry Pi 2 and later

# Function to read a byte from a register
def read_byte(register):
    return bus.read_byte_data(BNO055_ADDRESS, register)

# Function to write a byte to a register
def write_byte(register, value):
    bus.write_byte_data(BNO055_ADDRESS, register, value)

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

    # Reset the sensor
    write_byte(BNO055_REG_SYS_TRIGGER, 0x20)
    time.sleep(0.5)

    # Set the operation mode to NDOF
    write_byte(BNO055_REG_OPR_MODE, OPR_MODE_NDOF)
    time.sleep(0.5)

    while True:
        heading, roll, pitch = read_euler()
        print(f"Orientation: Heading: {heading:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by User")
except Exception as e:
    print(f"Error: {e}")
