from IMU import IMU
import threading
import time
import numpy as np


class IMUService:

    def __init__(self, freq: float, filter_type="K", use_mag=False):
        self.__thread = threading.Thread(target=self.__update, daemon=True)
        self.__imu = IMU(freq=freq, filter_type=filter_type, use_mag=use_mag)
        self.__freq = freq
        self.__data = None
        self.__thread.start()
        print("IMU Thread Started")

    def __update(self):
        while True:
            start_time = time.time()
            self.__imu.update_data()
            # self.__data = self.__imu.get_data()
            # time.sleep(max(1/self.__freq - (time.time() - start_time), 0))
            print(time.time() - start_time)
    
    def get_data(self):
        return self.__data

    def reset(self):
        self.__imu.reset(np.array([1.0, 0.0, 0.0, 0.0]))
