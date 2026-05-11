import time
import board
import adafruit_bno055


class BNO055_IMU:
    def __init__(self, address=0x28):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c, address=address)

    def format_tuple(self, values, digits=3):
        if values is None:
            return "None"
        return "(" + ", ".join(
            "None" if v is None else f"{v:.{digits}f}"
            for v in values
        ) + ")"

    def read_acceleration(self):
        return self.sensor.acceleration

    def read_linear_acceleration(self):
        return self.sensor.linear_acceleration

    def read_euler(self):
        return self.sensor.euler

    def read_gyro(self):
        return self.sensor.gyro

    def read_calibration(self):
        return self.sensor.calibration_status

    def get_data(self):  
        acceleration = self.read_acceleration()
        linear_accel = self.read_linear_acceleration()
        euler = self.read_euler()
        gyro = self.read_gyro()
        calib = self.read_calibration()

        return (
            "accel=" + self.format_tuple(acceleration) + " m/s^2 | "
            "linear=" + self.format_tuple(linear_accel) + " m/s^2 | "
            "euler=" + self.format_tuple(euler, digits=2) + " deg | "
            "gyro=" + self.format_tuple(gyro) + " rad/s | "
            f"calib={calib}"
        )

    def run(self, delay=0.05):
        print("BNO055 live IMU test")
        print("Tilt/rotate the sensor and watch acceleration/orientation change.")
        print("Press Ctrl+C to stop.\n")

        while True:
            print(self.get_data())  
            time.sleep(delay)


if __name__ == "__main__":
    imu = BNO055_IMU()
    imu.run()
