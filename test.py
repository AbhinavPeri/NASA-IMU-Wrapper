import time
from typing import List

import ahrs.common.quaternion

from IMU import IMU
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

from ahrs.filters import AngularRate
from ahrs.filters import Tilt
from squaternion import Quaternion

imu = IMU(use_mag=False)

style.use('fivethirtyeight')

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


def calibrate_gyro():
    avg_gyro = np.array([0.0, 0.0, 0.0])
    avg_acc = np.array([0.0, 0.0, 0.0])

    loop_num = 1
    while True:
        # read data and calculate the average
        acceleration, gyro, _ = imu.get_raw_sensor_data()
        avg_gyro += np.array(gyro)
        avg_acc += np.array(acceleration)

        print(avg_acc / loop_num)
        print(avg_gyro / loop_num)
        print("")
        loop_num += 1


def ellipsoid_fit(x, y, z):
    D = np.array(
        [np.multiply(x, x), np.multiply(y, y), np.multiply(z, z), 2 * np.multiply(x, y), 2 * np.multiply(x, z),
         2 * np.multiply(y, z), 2 * x, 2 * y, 2 * z])
    m_1 = np.linalg.inv(np.dot(D, D.transpose()))
    m_2 = np.dot(D, np.full((len(x), 1), 1))
    v = np.dot(m_1, m_2)
    v = v.transpose()[0]
    A = np.array([[v[0], v[3], v[4], v[6]],
                  [v[3], v[1], v[5], v[7]],
                  [v[4], v[5], v[2], v[8]],
                  [v[6], v[7], v[8], -1]])

    center = np.dot(-np.linalg.inv(A[0:3, 0:3]), np.array([[v[6]], [v[7]], [v[8]]]))

    T = np.identity(4)

    T[3, 0], T[3, 1], T[3, 2] = center[0], center[1], center[2]

    R = np.dot(T, np.dot(A, T.transpose()))

    evals, evecs = np.linalg.eig(np.linalg.inv(R[0:3, 0:3]) * -R[3, 3])

    radii = np.sqrt(np.reciprocal(evals))

    return center, radii, evecs, v


def calibrate_mag():
    # read data for 100 seconds
    t0 = time.perf_counter()
    _, _, magnetic = imu.get_raw_sensor_data()
    magnetic_data = np.array(magnetic)
    while True:
        _, _, magnetic = imu.get_raw_sensor_data()
        magnetic_data = np.vstack([magnetic_data, magnetic])
        t1 = time.perf_counter() - t0

        if t1 > 100:
            break
    # np.savetxt('magnetometer.csv', magnetic_data, delimiter=',', fmt='%1.7f')

    mag_x = magnetic_data[:, 0]
    mag_y = magnetic_data[:, 1]
    mag_z = magnetic_data[:, 2]
    e_center, e_radii, e_eigenvecs, e_algebraic = ellipsoid_fit(mag_x, mag_y, mag_z)

    S = np.array([mag_x - e_center[0], mag_y - e_center[1], mag_z - e_center[2]])

    scale = np.dot(np.linalg.inv(np.array([[e_radii[0], 0, 0], [0, e_radii[1], 0], [0, 0, e_radii[2]]])),
                   np.identity(3)) * min(e_radii)

    e_map = e_eigenvecs.transpose()
    invmap = e_eigenvecs
    comp = np.dot(invmap, np.dot(scale, e_map))

    S = np.dot(comp, S)

    print('magn_ellipsoid_center = ', e_center)
    print('magn_ellipsoid_transform = ', comp)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(mag_x, mag_y, mag_z, c='r')
    ax.scatter(S[0], S[1], S[2], c='b')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()


def test_integration():
    q = np.array([1, 0, 0, 0])
    q_filter = AngularRate(q0=q)
    while True:
        _, gyro, _, = imu.get_calibrated_sensor_data()
        q = q_filter.update(q, gyro)
        euler = Quaternion(*q).to_euler(degrees=True)
        print(euler)


def tilt_orientation():
    q_filter = Tilt()
    accel, _, _, = imu.get_calibrated_sensor_data()
    accel *= 9.8 / np.linalg.norm(accel)
    q = ahrs.common.quaternion.Quaternion(q_filter.estimate(accel))
    euler = np.rad2deg(q.to_angles())
    return euler


def test_filtering_kalman():
    q = np.array([1, 0, 0, 0])
    imu.reset(new_orientation_q=q)
    while True:
        imu.update_data()
        q, _, _, _ = imu.get_data()
        euler = Quaternion(*q).to_euler(degrees=True)
        print(euler)


def tune_kalman_filter():
    q = np.array([1, 0, 0, 0])
    imu.reset(new_orientation_q=q)
    measured_data = []
    real_data = []
    ani = animation.FuncAnimation(fig, live_plotting, fargs=(measured_data, real_data), interval=1000)
    plt.show()


def live_plotting(i, measured_data: List, real_data: List):
    accel, _, _, = imu.get_calibrated_sensor_data()
    r_euler = tilt_orientation()

    imu.update_data()
    m_q, _, _, _ = imu.get_data()
    m_euler = np.rad2deg(ahrs.common.quaternion.Quaternion(m_q).to_angles())

    measured_data.append(m_euler[0])
    real_data.append(r_euler[0])

    print("Kalman Filter: ", m_euler[0])
    print("Measured Tilt: ", r_euler[0])

    # Limit x and y lists to 20 items
    measured_data = measured_data[-20:]
    real_data = real_data[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(np.linspace(0, len(measured_data), len(measured_data)), measured_data)
    ax.plot(np.linspace(0, len(real_data), len(real_data)), real_data)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Kalman Filter Tuning')
    plt.ylabel('Orientation (Degrees)')


if __name__ == '__main__':
    tune_kalman_filter()
