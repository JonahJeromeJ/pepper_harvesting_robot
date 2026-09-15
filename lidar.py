'''
Obtains z value from lidar and returns it when get_z_axis is called
VIN = PIN1 | SDA = PIN3 | SCL = PIN5 | GND = PIN6
'''
import time
import board
import adafruit_vl53l0x

def get_x_axis():
    # Initialize the I2C bus and the VL53L0X sensor
    i2c = board.I2C()  # Initialize I2C bus
    sensor = adafruit_vl53l0x.VL53L0X(i2c)  # Initialize the VL53L0X sensor

    # Return the distance in millimeters
    return sensor.range-22  # This returns the distance in mm

# Main program
if __name__ == "__main__":
    try:
        while True:
            distance = get_x_axis()  # Get the distance from the sensor
            print(f"Distance: {distance} mm")
            time.sleep(1)  # Wait for 1 second before getting the next reading

    except KeyboardInterrupt:
        print("Program interrupted by user.")
