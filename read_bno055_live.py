import time

import board
import adafruit_bno055

def format_tuple(values, digits = 3):
    if values is None:
        return "None"
    return "(" + ", ".join(f"{v:.{digits}f}" for v in values) + ")"

def main():
    # Raspberry Pi 4 I2C bus: SDA=GPIO2/pin 3, SCL=GPIO3/pin 5
    i2c = board.I2C()

    # Most BNO055 breakouts use address 0x28.
    # If i2cdetect shows 0x29, use: adafruit_bno055.BNO055_I2C(i2c, address=0x29)
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    print("BNO055 live IMU test")
    print("Tilt/rotate the sensor and watch acceleration/orientation change.")
    print("Press Ctrl+C to stop.\n")

    while True:
        # Raw accelerometer: includes gravity, units are m/s^2
        acceleration = sensor.acceleration

        # Linear acceleration: acceleration with gravity removed by sensor fusion
        linear_accel = sensor.linear_acceleration

        # Euler orientation: heading, roll, pitch in degrees
        euler = sensor.euler

        # Gyroscope: angular velocity in rad/s
        gyro = sensor.gyro

        # Calibration status tuple: system, gyro, accel, magnetometer; each 0-3
        calib = sensor.calibration_status

        print(
            "accel=" + format_tuple(acceleration) + " m/s^2 | "
            "linear=" + format_tuple(linear_accel) + " m/s^2 | "
            "euler=" + format_tuple(euler, digits=2) + " deg | "
            "gyro=" + format_tuple(gyro) + " rad/s | "
            f"calib={calib}"
        )

        time.sleep(0.05)  # about 20 Hz print rate


if __name__ == "__main__":
    main()
