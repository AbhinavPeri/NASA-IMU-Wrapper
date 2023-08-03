import board
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import GyroRange, AccelRange, Rate
from adafruit_lis3mdl import LIS3MDL
from ahrs.filters import EKF, Madgwick

GRAVITY = np.array([0, 0, 9.802])

# TODO: Change calibration parameters
# ACCEL_OFFSET = np.array([-7.56248618e-03, 1.48946819e-01, 0.2475915])
GYRO_OFFSET = np.array([-0.00135783, 0.00470014, -0.0028537])
MAG_ELLIPSOID_CENTER = np.array([311.13542933, -5.74025537, 317.9063922])
MAG_ELLIPSOID_TRANSFORM = np.array([[0.58015691, -0.02669021, 0.44919358],
                                    [-0.02669021, 0.13769037, 0.05340476],
                                    [0.44919358, 0.05340476, 0.51868662]])

# ACCEL_OFFSET = ACCEL_OFFSET - GRAVITY

# TODO: Change covariances
COVARIANCES = [0.5 ** 2, 0.8 ** 2, 1000 ** 2]
BETA = 0.033

class IMU:

    def __init__(self, freq=52.5, filter_type="K", use_mag=False):
        self.__filter = None
        self.__filter_type = filter_type
        self.__imu = None
        self.__mag = None

        self.__orientation_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.__acc = np.zeros(3)
        self.__gyro = np.zeros(3)
        self.__mag_field = np.zeros(3)

        self.__use_mag = use_mag
        self.__setup_sensors()
        self.__setup_filter(q0=self.__orientation_q, freq=freq)

    def __setup_sensors(self):
        i2c = board.I2C()
        self.__imu = LSM6DSOX(i2c)
        self.__mag = LIS3MDL(i2c)

        rate = Rate.RATE_6_66K_HZ
        gyro_range = GyroRange.RANGE_2000_DPS
        accel_range = AccelRange.RANGE_16G

        self.__imu.rate = rate
        self.__imu.gyro_range = gyro_range
        self.__imu.accel_range = accel_range

    def __setup_filter(self, q0: np.ndarray, freq=52.5):
        if self.__filter_type == "K":
            self.__filter = EKF(q0=q0, noises=COVARIANCES, frequency=freq)
        else:
            self.__filter = Madgwick(q0=q0, gain=BETA, frequency=freq)

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
        pass
        # self.__acc = acc
        # self.__gyro = gyro
        # self.__mag_field = mag
        
        # if self.__filter_type == "K":
            # self.__orientation_q = self.__filter.update(self.__orientation_q, gyro, acc, mag=mag if self.__use_mag else None)
            # return

        # if self.__use_mag:
            # self.__orientation_q = self.__filter.updateMARG(self.__orientation_q, gyro, acc, mag)
        # else:
            # self.__orientation_q = self.__filter.updateIMU(self.__orientation_q, gyro, acc)


    def get_data(self):
        return self.__orientation_q, self.__acc, self.__gyro, self.__mag_field

    def reset(self, new_orientation_q: np.ndarray):
        self.__orientation_q = new_orientation_q
        self.__setup_filter(q0=new_orientation_q)

