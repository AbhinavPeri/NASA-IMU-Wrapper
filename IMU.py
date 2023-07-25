import board
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lis3mdl import LIS3MDL
from ahrs.filters import EKF

GRAVITY = np.array([0, 0, 9.802])

# TODO: Change calibration parameters
# ACCEL_OFFSET = np.array([-3.50157689e-02, -2.89182617e-03, 1.00599958e+01])
GYRO_OFFSET = np.array([-0.00136681, 0.00487601, -0.0033906])
MAG_ELLIPSOID_CENTER = np.array([0, 0, 0])
MAG_ELLIPSOID_TRANSFORM = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])

# ACCEL_OFFSET = ACCEL_OFFSET - GRAVITY

# TODO: Change covariances
COVARIANCES = [2 ** 2, 0.1 ** 2, 0]


class IMU:

    def __init__(self, use_mag=False):
        self.__filter = None
        self.__imu = None
        self.__mag = None

        self.__orientation_q = np.array([1, 0, 0, 0])
        self.__acc = np.zeros(3)
        self.__gyro = np.zeros(3)
        self.__mag_field = np.zeros(3)

        self.__use_mag = use_mag
        self.__setup_sensors()
        self.__setup_filter(q0=self.__orientation_q)

    def __setup_sensors(self):
        i2c = board.I2C()
        self.__imu = LSM6DSOX(i2c)
        self.__mag = LIS3MDL(i2c)

    def __setup_filter(self, q0: np.ndarray):
        self.__filter = EKF(q0=q0, noises=COVARIANCES)

    def get_raw_sensor_data(self):
        acc = np.array(self.__imu.acceleration)
        gyro = np.array(self.__imu.gyro)
        mag = np.array(self.__mag.magnetic)
        return acc, gyro, mag

    def get_calibrated_sensor_data(self):
        acc, gyro, mag = self.get_raw_sensor_data()

        # acc -= ACCEL_OFFSET
        # norm = np.linalg.norm(acc)
        # if norm != 0:
        #     acc *= 9.8 / norm

        gyro -= GYRO_OFFSET

        mag_tmp = mag - MAG_ELLIPSOID_CENTER
        mag = MAG_ELLIPSOID_TRANSFORM.dot(mag_tmp)

        return acc, gyro, mag

    def update_data(self):
        acc, gyro, mag = self.get_calibrated_sensor_data()

        self.__acc = acc
        self.__gyro = gyro
        self.__mag_field = mag

        if self.__use_mag:
            self.__orientation_q = self.__filter.update(self.__orientation_q, acc, gyro, mag)
        else:
            self.__orientation_q = self.__filter.update(self.__orientation_q, acc, gyro)

    def get_data(self):
        return self.__orientation_q, self.__acc, self.__gyro, self.__mag

    def reset(self, new_orientation_q: np.ndarray):
        self.__orientation_q = new_orientation_q
        self.__setup_filter(q0=new_orientation_q)

