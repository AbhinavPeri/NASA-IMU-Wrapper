# NASA-IMU-Wrapper

Code for reading an IMU sensor and running the AHRS fusion libraries for attitude estimation. The filters used are the Madgwick primarily and the EKF. The code also includes support for gps heading fusion, and runs in a separate process to run in parallel with main controls loop
