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



