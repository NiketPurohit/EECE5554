#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
import numpy as np
from datetime import datetime
from imu_driver.msg import *
from imu_driver.msg import Vectornav
        
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