import adafruit_gps
import time
import board
import serial
import threading

class GPS:

    def __init__(self):
        
        # setup sensor hardware
        self.__uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.__gps = adafruit_gps.GPS(self.__uart, debug=False)
        self.__gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.__gps.send_command(b"PMTK220,500")
        
        # data
        self.__location = None
        self.__track_angle_deg = None
        self.__speed_knots = None
        self.__timestamp = None

        self.__gps_thread = threading.Thread(target=self.__gps_update, daemon=True)
        self.__gps_thread.start()


    def __gps_update(self):
        while not self.__gps.has_fix:
            self.__gps.update()
            print("GPS Thread: Waiting for fix...")
            time.sleep(1)
        
        print("GPS Thread: Fix Acquired")

        while True:
            
            self.__gps.update()

            self.__location = [self.__gps.latitude_degrees, self.__gps.latitude_minutes, self.__gps.longitude_degrees, self.__gps.longitude_minutes]
            self.__track_angle_deg = self.__gps.track_angle_deg
            self.__speed_knots = self.__gps.speed_knots
            self.__timestamp = time.time() 
    
    def get_data(self):
        return self.__location, self.__track_angle_deg, self.__speed_knots, self.__timestamp







