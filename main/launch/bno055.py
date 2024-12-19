import time
import board
import busio
from adafruit_bno055 import BNO055_I2C

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create BNO055 object
bno = BNO055_I2C(i2c, address=0x28)

# Initialize the sensor
bno.begin()

# Set the external crystal use
bno.set_ext_crystal_use(True)

print("BNO055 Sensor Initialized")

try:
    while True:
        # Read the Euler angles
        euler = bno.euler
        if euler is not None:
            print(f"Orientation: Heading: {euler[0]:.2f}, Roll: {euler[1]:.2f}, Pitch: {euler[2]:.2f}")
        else:
            print("Failed to read orientation data")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by User")
