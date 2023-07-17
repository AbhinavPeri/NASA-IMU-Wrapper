from IMU import IMU
from multiprocessing import Process
import time


class IMUService:

    def __init__(self, freq: float, use_mag=False):
        self.__thread = Process(target=self.__update, daemon=True)
        self.__imu = IMU(use_mag=use_mag)
        self.__freq = freq
        self.__data = None
        self.__thread.start()

    def __update(self):
        dt = 1/self.__freq
        self.__imu.update_data(dt=dt)
        self.__data = self.__imu.get_data()
        time.sleep(dt)

