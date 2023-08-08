import adafruit_gps
import time
import board
import serial
from multiprocessing import Process, Pipe

class GPS:

    def __init__(self, fix_timeout=-1):   
        # setup sensor hardware
        self.__uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.__gps = adafruit_gps.GPS(self.__uart, debug=False)
        self.__gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.__gps.send_command(b"PMTK220,500")
    
        # data
        self.__data = [None, None, None, False]

        # setup multiprocessing
        data_requested, self.__request_data = Pipe()
        self.__receive_data, send_data = Pipe()
        self.__process = Process(target=self.__update_data, args=(self.__gps, data_requested, send_data, fix_timeout), daemon=True)
        
    
    def __update_data(self, gps, data_requested, send_data, fix_timeout):
        self.__acquire_gps_fix(gps, fix_timeout)
        
        while True:
            data = self.__get_data(gps)
            if data_requested.poll(timeout=0.001):
                data_requested.recv()
                send_data.send(data)


    def __get_data(self, gps):
        new_message_received = gps.update()

        if new_message_received:
            location = [gps.latitude_degrees, gps.latitude_minutes, gps.longitude_degrees, gps.longitude_minutes]
            track_angle_deg = gps.track_angle_deg
            speed_knots = gps.speed_knots 
            return location, track_angle_deg, speed_knots, new_message_received
        return None


    def __acquire_gps_fix(self, gps, timeout=-1):
        start = time.time()
        while not gps.has_fix:
            if time.time() - start > timeout and timeout > 0:
                break
            gps.update()
            print("GPS: Waiting for fix...")
            time.sleep(1)
        
        has_fix = gps.has_fix

        if has_fix:
            print("GPS: Fix Acquired")
        else:
            print("GPS: Failed to acquire fix")
        
        return has_fix


    def start(self):
        self.__process.start()


    def get_gps_data(self):
        self.__request_data.send(True)
        if self.__receive_data.poll(0.002):
            data = self.__receive_data
            if data:
                self.__data = data
            else:
                self.__data[3] = False
        else:
            self.__data[3] = False

        return self.__data

