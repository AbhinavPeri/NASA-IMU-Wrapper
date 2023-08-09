from IMU import IMU
from GPS import GPS
import threading
import time
import numpy as np
from squaternion import Quaternion
import logging
import signal


#now we will Create and configure logger
logging.basicConfig(filename="std.log",
					format='%(asctime)s %(message)s',
					filemode='w')

#Let us Create an object
logger=logging.getLogger()

#Now we are going to Set the threshold of logger to DEBUG
logger.setLevel(logging.DEBUG)


class AttitudeEstimatorGPSIMU:

    def __init__(self, freq: float, alpha: float, gps_yaw_offset=0, imu_filter_type="M", use_mag=False):
        self.__thread = threading.Thread(target=self.__update, daemon=True)
        
        self.__imu = IMU(freq=freq, filter_type=imu_filter_type, use_mag=use_mag)
        self.__gps = GPS()
        self.__has_gps = False
        
        self.__freq = freq
        self.__alpha = alpha
        self.__gps_yaw_offset = gps_yaw_offset
        
        self.__orientation = np.zeros(3)
        self.__measured_dt_start = None        
        signal.signal(signal.ALRM, lambda signum, frame : raise Exception())

    def __update(self):
        # Starting Sensors
        print("Attitude Estimator: Calibrating... Please don't touch the IMU")
        self.__imu.calibrate()
        print("Attitude Estimator: Calibration has finished")

        print("Attitude Estimator: Acquiring GPS fix")
        self.__has_gps = self.__gps.acquire_gps_fix(-1)
        
        # Setting up timed loop
        expected_wake_time = time.time()
        prev_offset = None
        while True:
            start = time.time()
            offset_time = start - expected_wake_time
            if prev_offset and offset_time - prev_offset > 0.002:
                print("Attitude Estimator Thread is not waking up consistently. Unable to meet specified frequency")
            prev_offset = offset_time

            # Fusing GPS and IMU estimates
            self.__fuse_gps_imu()
            
            sleep_time = max(1/self.__freq - (time.time() - start) - offset_time, 0)
            expected_wake_time += 1/self.__freq
            if sleep_time == 0:
                print("Attitude Estimator Thread Duration exceeds specified frequency")
            time.sleep(max(sleep_time - 0.001, 0))
    

    def __fuse_gps_imu(self):
        # start = time.time() 
        
        signal.alarm(0.01)
        try:
            if self.__has_gps:
                _, gps_heading, gps_speed, new_message_received = self.__gps.get_data()
        except:
            pass

        measured_dt = None
        if self.__measured_dt_start:
            measured_dt = time.time() - self.__measured_dt_start
        else:
            measured_dt = 1 / self.__freq
        print(measured_dt)
        self.__imu.update_data(measured_dt)
        
        self.__measured_dt_start = time.time()

        q, _, _, _ = self.__imu.get_data()
        q = Quaternion(*q)
        e = np.array(q.to_euler(degrees=True))
        
        self.__orientation = e

        if self.__has_gps and new_message_received and gps_speed > 1.6:
            gps_heading = self.convert_gps_to_imu(gps_heading)

            logger.debug("=" * 20)
            logger.debug("Current Orientation: " + str(self.__orientation))
            logger.debug("GPS Heading: " + str(gps_heading))
            logger.debug("GPS Speed: " + str(gps_speed))
            
            self.__orientation[2] = self.__alpha * e[2] + (1 - self.__alpha) * gps_heading
            new_q = Quaternion.from_euler(*self.__orientation, degrees=True)
            self.__imu.reset(np.array(new_q))

        # print(time.time() - start)
   
    
    def start(self):
        self.__thread.start()
        print("Attitude Estimator Started")


    def get_orientation(self):
        return self.__orientation
    

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

