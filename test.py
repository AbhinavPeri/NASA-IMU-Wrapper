from PoseEstimator import PoseEstimator
import time
from squaternion import Quaternion
from IMUService import IMUService
import numpy as np

if __name__ == '__main__':
    
    p = PoseEstimator(alpha=0.7, freq=20, imu_filter_type="M", use_mag=False)
    p.reset(206)
    # imu = IMUService(freq=20, filter_type="M", use_mag=False)
    # imu.reset()
    while True:
        _, angles = p.get_pose()
        print(angles)
        # q, _, _, _ = imu.get_data()
        # if not q is None:
            # angles = Quaternion(*q).to_euler(degrees=True)
            # print(angles)
