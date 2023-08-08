from GPS import GPS
from IMUService import IMUService
import time

if __name__ == "__main__":
    gps = GPS()
    gps.start()
    max_time = 0
    while True:
        start = time.monotonic()
        gps.get_gps_data()
        max_time = max(max_time, time.monotonic() - start)
        print(max_time)
        time.sleep(0.1)

