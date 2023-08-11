import multiprocessing.managers

from IMU import IMU
from GPS import GPS
from multiprocessing import Process, Queue, Manager, Value
import time
import numpy as np
from squaternion import Quaternion
import logging
import signal

# now we will Create and configure logger
logging.basicConfig(filename="std.log",
                    format='%(asctime)s %(message)s',
                    filemode='w')

# Let us Create an object
logger = logging.getLogger()

# Now we are going to Set the threshold of logger to DEBUG
logger.setLevel(logging.DEBUG)


class PoseEstimator:

    def __init__(self, freq: float, alpha: float, gps_yaw_offset=0, initial_pose=np.zeros(3), gps_timeout=10,
                 imu_filter_type="M", use_mag=False):

        self.__imu_reset_queue = Queue(1)

        self.__imu = IMU(freq=freq, filter_type=imu_filter_type, use_mag=use_mag)
        self.__gps = GPS()
        self.__has_gps = Value('i', 0)

        self.__freq = freq
        self.__alpha = alpha

        self.__manager = Manager()

        self.__gps_yaw_offset = Value('d', gps_yaw_offset)
        self.__gps_timeout = gps_timeout
        self.__gps_data = self.__manager.list([None, None, None, False])

        self.__position = self.__manager.list([initial_pose[0], initial_pose[1]])
        self.__speed = Value('d', 0)
        self.__orientation = self.__manager.list([0.0, 0.0, initial_pose[2]])

        self.__gps_process = Process(target=self.__gps_update, daemon=True)
        self.__update_pose_process = Process(target=self.__update, daemon=True)


    def __update(self):
        # Starting Sensors
        print("Attitude Estimator: Calibrating... Please don't touch the IMU", flush=True)
        self.__imu.calibrate()
        print("Attitude Estimator: Calibration has finished", flush=True)

        # Setting up timed loop
        expected_wake_time = time.time()
        prev_offset = None
        while True:
            start = time.time()
            offset_time = start - expected_wake_time
            if prev_offset and offset_time - prev_offset > 0.002:
                print("Attitude Estimator Thread is not waking up consistently. Unable to meet specified frequency", flush=True)
            prev_offset = offset_time

            # Fusing GPS and IMU estimates
            self.__fuse_gps_imu()

            sleep_time = max(1 / self.__freq - (time.time() - start) - offset_time, 0)
            expected_wake_time += 1 / self.__freq
            if sleep_time == 0:
                print("Attitude Estimator Thread Duration exceeds specified frequency", flush=True)
            time.sleep(max(sleep_time - 0.001, 0))

    def __gps_update(self):
        print("Attitude Estimator: Acquiring GPS fix", flush=True)
        self.__has_gps.value = 1 if self.__gps.acquire_gps_fix(self.__gps_timeout) else 0
        # if not self.__has_gps:
        #     return

        while True:
            data = self.__gps.get_data()
            self.__gps_data[:] = data
            time.sleep(0.5)

    def __fuse_gps_imu(self):
        use_gps_data = False
        if self.__has_gps.value:
            location, gps_heading, gps_speed, new_message_received = list(self.__gps_data)
            use_gps_data = new_message_received and gps_speed > 1.6

            self.__speed.value = gps_speed * 0.514444
            self.__update_position(location)

        self.__imu.update_data(1 / self.__freq)

        q, _, _, _ = self.__imu.get_data()
        q = Quaternion(*q)
        e = np.array(q.to_euler(degrees=True))

        self.__orientation[:] = list(e)


        if use_gps_data:
            gps_heading = self.convert_gps_to_imu(gps_heading)

            logger.debug("=" * 20)
            logger.debug("Current Orientation: " + str(self.__orientation))
            logger.debug("GPS Heading: " + str(gps_heading))
            logger.debug("GPS Speed: " + str(gps_speed))

            self.__orientation[2] = self.__alpha * e[2] + (1 - self.__alpha) * gps_heading
            new_q = Quaternion.from_euler(*self.__orientation, degrees=True)

            self.__imu.reset(np.array(new_q))

        if not self.__imu_reset_queue.empty():
            reset = self.__imu_reset_queue.get()
            self.__orientation[:] = list(reset[1])
            self.__imu.reset(np.array(reset[0]))

    def __update_position(self, location):
        pass

    def start(self):
        self.__gps_process.start()
        self.__update_pose_process.start()
        print("Pose Estimator Started", flush=True)

    def get_position(self):
        return np.array(list(self.__position))

    def get_orientation(self):
        return np.array(self.__orientation)

    def get_speed(self):
        return self.__speed.value

    def get_gps_data(self):
        return list(self.__gps_data)

    def reset(self, reset_orientation=None, gps_yaw_offset=None):
        if not type(reset_orientation) is None:
            q_reset = Quaternion().from_euler(*reset_orientation, degrees=True)
            self.__imu_reset_queue.put((q_reset, reset_orientation))

        if gps_yaw_offset:
            self.__gps_yaw_offset.value = gps_yaw_offset


    def convert_gps_to_imu(self, gps_angle: float):
        new_gps_angle = gps_angle - self.__gps_yaw_offset
        new_gps_angle = new_gps_angle % 360
        if (0 <= new_gps_angle) and (new_gps_angle <= 180):
            imu_coordinate_system_gps = -1 * new_gps_angle

        else:
            imu_coordinate_system_gps = 360 - new_gps_angle

        return imu_coordinate_system_gps
