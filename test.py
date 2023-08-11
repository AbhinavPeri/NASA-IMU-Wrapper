from PoseEstimator import PoseEstimator
import time
from squaternion import Quaternion
from IMUService import IMUService
import numpy as np

if __name__ == '__main__':

    pose_estimator = PoseEstimator(freq=80, alpha=0.7, gps_timeout=0)
    pose_estimator.start()
    pose_estimator.reset(reset_orientation=np.array([0, 0, 0]))
    while True:
        angles = pose_estimator.get_orientation()
        print(angles)
        time.sleep(0.1)
