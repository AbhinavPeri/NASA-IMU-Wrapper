from IMUService import IMUService
from GPS.GPS import GPS
import time

if __name__ == '__main__':
    # service = IMUService(freq=50, filter_type="M")
    gps = GPS()
    while True:
        print(gps.get_data())
        time.sleep(0.5)
        # print(service.get_data())
