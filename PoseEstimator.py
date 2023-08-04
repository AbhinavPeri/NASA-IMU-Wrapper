from IMU import IMU
from GPS import GPS
import threading
import time
import numpy as np
from squaternion import Quaternion
import logging

#now we will Create and configure logger
logging.basicConfig(filename="std.log",
					format='%(asctime)s %(message)s',
					filemode='w')

#Let us Create an object
logger=logging.getLogger()

#Now we are going to Set the threshold of logger to DEBUG
logger.setLevel(logging.DEBUG)

class PoseEstimator:

    def __init__(self, alpha: float, freq: float, imu_filter_type="K", use_mag=False):
        self.__thread = threading.Thread(target=self.__update, daemon=True)
        
        self.__imu = IMU(freq=freq, filter_type=imu_filter_type, use_mag=use_mag)
        self.__gps = GPS()
        
        self.__freq = freq
        self.__alpha = alpha

        self.__pos = None
        self.__orientation = None
        self.__gps_yaw_offset = 0

        self.__thread.start()
        print("Pose Estimator Thread Started")

    def __update(self):
        print("IMU Service: Calibrating... Please don't touch the IMU")
        self.__imu.calibrate()
        print("IMU Service: Calibration has finished")

        _, gps_heading, gps_speed, old_gps_timestamp = self.__gps.get_data()
        if not gps_heading is None:
            gps_heading = self.convert_gps_to_imu(gps_heading)
        else:
            old_gps_time_stamp = 0

        while True:
            start_time = time.time()
            self.__imu.update_data(1/self.__freq)

            _, gps_heading, gps_speed, gps_timestamp = self.__gps.get_data()
            
            q, _, _, _ = self.__imu.get_data()
            q = Quaternion(*q)
            e = q.to_euler(degrees=True)
            if not gps_heading is None and not gps_timestamp == old_gps_timestamp and gps_speed > 1.6:
                gps_heading = self.convert_gps_to_imu(gps_heading)
                old_gps_timestamp = gps_timestamp
                self.__orientation = np.array(list(e))
                logger.debug("=" * 20)
                logger.debug("Current Orientation: " + str(self.__orientation))
                logger.debug("GPS Heading: " + str(gps_heading))
                logger.debug("GPS Speed: " + str(gps_speed))
                self.__orientation[2] = self.__alpha * e[2] + (1 - self.__alpha) * gps_heading
                new_q = Quaternion.from_euler(*self.__orientation, degrees=True)
                self.__imu.reset(np.array(new_q))
            else:
                self.__orientation = np.array(list(e))

            sleep_time = max(1/self.__freq - (time.time() - start_time), 0)
            if sleep_time == 0:
                print("Decrease filter frequency. Filter cannot run at requested speed")
            time.sleep(sleep_time)
            # print(time.time() - start_time)
    
    def get_pose(self):
        return self.__pos, self.__orientation

    def reset(self, gps_yaw_offset: float):
        self.__imu.reset(np.array([1.0, 0.0, 0.0, 0.0]))
        self.__gps_yaw_offset = gps_yaw_offset

    def convert_gps_to_imu(self, gps_angle: float):
        new_gps_angle = gps_angle - self.__gps_yaw_offset
        new_gps_angle = new_gps_angle % 360
        if (0 <= new_gps_angle) and ( new_gps_angle <= 180):
            imu_coordinate_system_gps = -1 * new_gps_angle

        else: 
            imu_coordinate_system_gps = 360 - new_gps_angle
        
        return imu_coordinate_system_gps


    def get_offset(self):
        return self.__gps_yaw_offset
