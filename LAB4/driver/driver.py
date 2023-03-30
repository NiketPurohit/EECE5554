#!/bin/env python3
# from puck import eece5554
import serial.tools.list_ports as usbs
import utm
import sys
from serial import *
import std_msgs
import rospy
import argparse
from gps_driver.msg import gps_msg
import sys
from serial import *

class eece5554:

    def __init__(self,passed_port):
        if sys.platform.startswith('win') or sys.platform.startswith('linux') or sys.platform.startswith('cygwin') or sys.platform.startswith('darwin'):
            self.ports = list(usbs.comports())
        else:
            raise EnvironmentError('Unsupported platform')
        self.gps_connect(passed_port)
        self.long_chars = ['E','W']
        self.lat_chars = ['S','N']
    
    def gps_connect(self,passed_port):
        gps_want = "Prolific Technology Inc."
        gps_version = "USB-Serial Controller D"
        if passed_port == None:
            for p in self.ports:
                if (p.manufacturer == gps_want and p.product == gps_version) or p.manufacturer == "Prolific":
                    passed_port = p.device
                    break
                else:
                    print("[-] Oops! it seems I cannot find the Puck!")
                    return
            try:
                self.gps = Serial(passed_port,timeout=None,baudrate=4800,xonxoff=False,rtscts=False,dsrdtr=False)
                self.gps.flush()
                self.gps.reset_input_buffer()
                self.gps.reset_output_buffer()
            except SerialException as e:
                print("[*] Please check the serial ports and connections ",e)
        else:
            self.gps = Serial(passed_port,timeout=None,baudrate=4800,xonxoff=False,rtscts=False,dsrdtr=False)
            self.gps.flush()
            self.gps.reset_input_buffer()
            self.gps.reset_output_buffer()
        
    def lat_long_input(self,data_string):
        if data_string[0] not in ('GPGGA','GPRMC') or not data_string[3]:
            return (0.0,0.0)
        latitude_index = [data_string.index(c) for c in self.lat_chars if c in data_string]
        latitude_sign = 1 if data_string[latitude_index[0]] != self.lat_chars[0] else -1
        latitude_index = latitude_index[0] - 1

        longitude_index = [data_string.index(c) for c in self.long_chars if c in data_string]
        longitude_sign = -1 if data_string[longitude_index[0]] == self.long_chars[1] else 1
        longitude_index = longitude_index[0] - 1

        lat_ddmm = float(data_string[latitude_index][:2]) + float(data_string[latitude_index][2:])/60
        lon_ddmm = float(data_string[longitude_index][1:3]) + float(data_string[longitude_index][3:])/60
        return (latitude_sign * lat_ddmm,longitude_sign * lon_ddmm)

    def read_utm_latlong(self,data_string):
        normal_lat_long = self.lat_long_input(data_string)
        if normal_lat_long != (0.0, 0.0):
            utm_latlon = utm.from_latlon(*normal_lat_long)
            return utm_latlon

    def horizontal_dilution_of_precision_input(self,data_string):
        if data_string[0] == 'GPGGA':
            return round(float(data_string[8]),2)
        elif data_string[0] == 'GPGSA':
            return round(float(data_string[16]),2)
        else:
            return 0.0
    
    def alt_input(self,data_string):
        if data_string[0] == 'GPGGA':
            return round(float(data_string[9]),2)
        else:
            return 0.0
    
    def time_input(self,data_string):
        if data_string[0] in ('GPGGA','GPRMC'):
            return data_string[1]

    def data_string_return(self):
        read_data = self.gps.readline().decode().replace("$","").replace("\n","").replace("\r","").split(",")
        read_data.pop()
        return read_data
    
    def line_input(self):
        return self.gps.readline()  

class gps_ros():
    def __init__(self):
        
        rospy.init_node('gps',anonymous = True)
        param_names = rospy.get_param_names()
        # print("params",param_names)
        arg_port = rospy.get_param("/driver/port")
        rospy.loginfo(arg_port)
        arg_port = arg_port if arg_port != 'None' else None
        self.my_gps = eece5554(arg_port)
        self.msg = gps_msg()
        self.pub = rospy.Publisher('/gps',gps_msg,queue_size = 10)
        self.rate = rospy.Rate(10)
        rospy.loginfo_once("[o] No argument passed, considering AUTO PORT SELECTION") if not arg_port else rospy.loginfo_once("[/] Is this the port you passed -- %s",arg_port)
    
    def talker(self):
        i = 0
        print("hello")
        while not rospy.is_shutdown():
            usf_data = self.my_gps.data_string_return()
            time_stamp = self.my_gps.time_input(usf_data)
            if time_stamp:
                lat,long = self.my_gps.lat_long_input(usf_data)
                utm_latlon = self.my_gps.read_utm_latlong(usf_data)
                self.msg.Header.seq = i
                self.msg.Header.stamp.secs = int(time_stamp[0:2]) * 3600 + int(time_stamp[2:4]) * 60 + int(float(time_stamp[4:6]))
                self.msg.Header.frame_id = 'GPS1_Frame'
                self.msg.Header.stamp.nsecs = int(float(time_stamp[6:]))*10**9    
                self.msg.Latitude = lat
                self.msg.Longitude = long
                self.msg.Altitude = self.my_gps.alt_input(usf_data)
                self.msg.HDOP = self.my_gps.horizontal_dilution_of_precision_input(usf_data)
                self.msg.UTM_easting = utm_latlon[0]
                self.msg.UTM_northing = utm_latlon[1]
                self.msg.UTC = f"{int(time_stamp[0:2])}:{int(time_stamp[2:4])}:{int(float(time_stamp[4::]))}"
                self.msg.Zone = utm_latlon[2]
                self.msg.Letter = utm_latlon[3]
                self.pub.publish(self.msg)
                i = i+1

if __name__ == '__main__':
    gpsros = gps_ros()
    try:
        gpsros.talker()
        rospy.spin()
    except rospy.ROSInterruptException: pass

class Driver:

    def __init__(self) -> None:
        self.publisher = rospy.Publisher('imu', Vectornav, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        #rate = rospy.Rate(40)
        self.msg = Vectornav()

        args = rospy.myargv(argv = sys.argv)
        if len(args) != 2:
            print("error")
            sys.exit(1)

        port = args[1]
        self.ser_port = rospy.get_param('~port',port)
        self.baudrate = rospy.get_param('~baudrate',115200)

    def run(self):
        ser = serial.Serial(self.ser_port, self.baudrate, timeout = 3)
        ser.write(b"$VNWRG,07,40*xx")
        while not rospy.is_shutdown():
            recieve = str(ser.readline())
            # recieve = recieve.decode('utf-8')

            if "$VNYMR" in str(recieve):
                data = str(recieve)
                # print(data)

                now = rospy.get_rostime()
                rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

                # now = rospy.get_rostime()
                # rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
                # #now = datetime.now()
                # #current_time = now.strftime("%H:%M:%S")
                # #print("Current Time =", current_time)

                # yaw = float(data[1])
                # pitch = float(data[2])
                # roll = float(data[3])
                # magX = float(data[4])
                # magY = float(data[5])
                # magZ = float(data[6])
                # accX = float(data[7])
                # accY = float(data[8])
                # accZ = float(data[9])
                # gyroX = float(data[10])
                # gyroY = float(data[11])
                # gyroZ = float(data[12][0:9])
                yaw,pitch,roll,magX,magY,magZ,accX,accY,accZ,gyroX,gyroY,gyroZ = self.parse_data(data)

                #def orientation(roll, pitch, yaw):   #Convert an Euler angle to a quaternion.
                qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                #return [qx, qy, qz, qw]

                #msg.header.stamp = rospy.Time.from_sec(now)
                self.msg.header.stamp.secs = int(now.secs)
                self.msg.header.stamp.nsecs = int(now.nsecs)
                self.msg.header.frame_id = 'IMU1_Frame'
                self.msg.IMU.orientation.x = qx
                self.msg.IMU.orientation.y = qy
                self.msg.IMU.orientation.z = qz
                self.msg.IMU.orientation.w = qw
                self.msg.IMU.linear_acceleration.x = accX
                self.msg.IMU.linear_acceleration.y = accY
                self.msg.IMU.linear_acceleration.z = accZ
                self.msg.IMU.angular_velocity.x = gyroX
                self.msg.IMU.angular_velocity.y = gyroY
                self.msg.IMU.angular_velocity.z = gyroZ
                self.msg.MagField.magnetic_field.x = magX
                self.msg.MagField.magnetic_field.y = magY
                self.msg.MagField.magnetic_field.z = magZ

                self.publisher.publish(self.msg)
                #rate.sleep()

    def parse_data(self, data):
        data = data.split(",")

        return float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[5]), float(data[6]), float(data[7]), float(data[8]), float(data[9]), float(data[10]), float(data[11]), float(data[12][0:9])
    

if __name__ == '__main__':
    try:
        driver = Driver()
        driver.run()
        # driver()
    except rospy.ROSInterruptException:
        pass

