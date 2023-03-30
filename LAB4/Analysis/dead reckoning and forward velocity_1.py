import bagpy
import math
import csv
import time
import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from scipy.optimize import fsolve
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal


class deadRecon:

  def __init__(self) -> None:
    sns.set_style("dark")
    sns.color_palette("viridis", as_cmap=True)
    plt.rcParams.update({'font.size': 30})

    self.bag_file = bagreader('/home/niket/n1_catkin_ws/src/LAB4/data/data_driving.bag')
    data = self.bag_file.message_by_topic('/imu')
    self.imu_data_csv = pd.read_csv(data)
    self.gps_data_csv = pd.read_csv('/home/niket/n1_catkin_ws/src/LAB4/data/data_driving/gps.csv')

  
  

  def trajectory(self):
    self.y_observ = self.imu_data_csv['imu.linear_acceleration.y']
    #Trajectory of Vehicle
    orient_w = self.imu_data_csv['imu.orientation.w']
    orient_x = self.imu_data_csv['imu.orientation.x']
    orient_y = self.imu_data_csv['imu.orientation.y']
    orient_z = self.imu_data_csv['imu.orientation.z']

    accel_x = self.imu_data_csv['imu.linear_acceleration.x']
    imu_time = self.imu_data_csv['header.stamp.secs']+self.imu_data_csv['header.stamp.nsecs']*10e-9
    x_2_dot = accel_x
    x_1_dot = integrate.cumtrapz(x_2_dot)
    angz = self.imu_data_csv['imu.angular_velocity.z']
    self.y2dot = angz[1:] * x_1_dot
    
    # Euler from Quaternion(x, y, z, w):
    t_0 = +2.0 * (orient_w * orient_x + orient_y * orient_z)
    t_1 = +1.0 - 2.0 * (orient_x * orient_x + orient_y * orient_y)
    roll_x = np.arctan2(t_0, t_1)

    t_2 = +2.0 * (orient_w * orient_y - orient_z * orient_x)
    pitch_y = np.arcsin(t_2)

    t_3 = +2.0 * (orient_w * orient_z + orient_x * orient_y)
    t_4 = +1.0 - 2.0 * (orient_y * orient_y + orient_z * orient_z)
    yaw_z = np.arctan2(t_3, t_4)

    roll = roll_x
    print('roll', roll)
    pitch = pitch_y
    yaw = yaw_z
    fv = np.unwrap(self.forw_vel_adj)
    mgh = self.imu_data_csv['mag_field.magnetic_field.x']
    mgh_yz = yaw_z
    rot = (-108*np.pi/180)

    u_1 = np.cos(mgh_yz[1:]+rot)*fv
    u_2 = -np.sin(mgh_yz[1:]+rot)*fv
    u_3 = np.cos(mgh_yz[1:]+rot)*fv
    u_4 = np.sin(mgh_yz[1:]+rot)*fv
    rads = (180/np.pi)
    v_e = u_1+u_2
    v_n = u_3+u_4
    x_e = integrate.cumtrapz(v_e)
    x_n = integrate.cumtrapz(v_n)

    f, ax = plt.subplots(3, 1, figsize=(30, 18))
    f.subplots_adjust(hspace=0.4)
    ax[0].plot(self.y_observ, label = 'Y observed')
    ax[0].plot(self.y2dot/1000, label = 'wX(dot)')
    ax[1].plot((x_e/(10**6))/2,-x_n/(10**5), c='crimson')
    ax[2].plot(self.UTMeasting, self.UTMnorthing, c ='palevioletred')
    ax[0].set_xlabel('Samples @ 40Hz')
    ax[0].set_ylabel('Acceleration (m/s^2)')
    ax[0].set_title('y_observed V/S wX(dot)')
    ax[1].set_xlabel('Xe')
    ax[1].set_ylabel('Xn')
    ax[1].set_title('Trajectory of Vehicle')
    ax[2].set_xlabel('UTM Easting')
    ax[2].set_ylabel('UTM Northing')
    ax[2].set_title('UTM Easting V/S UTM Northing')
    plt.show()

    plt.figure(figsize = (8,8))
    plt.plot((x_e/(10**6))/2,-x_n/(10**5), c='crimson')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('Trajectory of Vehicle')
    plt.xlabel('Xe')
    plt.ylabel('Xn')
    plt.plot()
    plt.show()

    plt.figure(figsize = (8,8))
    plt.plot(self.UTMeasting, self.UTMnorthing, c ='palevioletred')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('UTM Easting V/S UTM Northing')
    plt.xlabel('UTM Easting')
    plt.ylabel('UTM Northing')
    plt.show()

  
    
  def imu_vel(self):
    secs = self.imu_data_csv['header.stamp.secs']
    nsecs = np.double(self.imu_data_csv['header.stamp.nsecs'])
    nsecs = nsecs / 1000000000
    time_x = np.double(secs) + nsecs


    #imu Velocity
    raw_val_accelaration = self.imu_data_csv['imu.linear_acceleration.x']
    orient_x = np.mean(raw_val_accelaration)
    lin_accel = raw_val_accelaration - orient_x

    accel_def = []
    for i in range(len(lin_accel)-1):
      accel_def = np.append(accel_def, (lin_accel[i + 1] - lin_accel[i]) / (0.025))
    print(accel_def)

    final = lin_accel[1:] - accel_def
    self.forw_vel_adj = integrate.cumtrapz(final, initial=0)
    self.forw_vel_adj[self.forw_vel_adj<0] = 0
    forw_vel_raw = integrate.cumtrapz(lin_accel, initial=0)


    #GPS Velocity
    tim=self.gps_data_csv['header.stamp.secs']
    self.UTMeasting = self.gps_data_csv['UTM_easting']
    self.UTMnorthing = self.gps_data_csv['UTM_northing']
    latitude = self.gps_data_csv['Latitude']
    longitude = self.gps_data_csv['Longitude']
    self.utm_dist=[]
    vel=[]
    for i in range(1194):
      self.utm_dist = np.append(self.utm_dist, math.sqrt(((self.UTMnorthing[i + 1] - self.UTMnorthing[i]) ** 2) + (self.UTMeasting[i + 1] - self.UTMeasting[i]) ** 2))
    print(len(self.utm_dist))
    gps_vel= self.utm_dist / tim[1:]
    gps_time = self.gps_data_csv['Time']

    #Plot b/w forward velocity from imu to gps velocity before adjustment
    plt.figure(figsize = (16,8))
    plt.plot(time_x, forw_vel_raw, label='imu Raw Velocity', c='palevioletred')
    plt.plot(gps_time[1:], gps_vel*2000000, label='GPS Raw Velocity')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('Forward velocity from imu and GPS before adjustment')
    plt.xlabel('Time (secs)')
    plt.ylabel('Velocity (m/sec)')
    plt.show()
    
    #plot b/w imu velocity and gps velocity after adjustment.
    
    plt.figure(figsize = (16,8))
    plt.plot(time_x[1:], self.forw_vel_adj / 1000, label='imu Adjusted Velocity', c='palevioletred')
    plt.plot(gps_time[1:], gps_vel*2000, label='GPS adjusted Velocity')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('Forward velocity from imu and GPS after adjustment')
    plt.xlabel('Time (secs)')
    plt.ylabel('Velocity (m/sec)')
    plt.show()

  def desplacement(self):

    accel_x = self.imu_data_csv['imu.linear_acceleration.x']
    imu_time = self.imu_data_csv['header.stamp.secs']+self.imu_data_csv['header.stamp.nsecs']*10e-9
    x_2_dot = accel_x
    x_1_dot = integrate.cumtrapz(x_2_dot)
    angz = self.imu_data_csv['imu.angular_velocity.z']
    self.y2dot = angz[1:] * x_1_dot
    t = self.imu_data_csv['header.stamp.secs']
    self.y_observ = self.imu_data_csv['imu.linear_acceleration.y']
    plt.figure(figsize = (8,8))
    plt.plot(self.y_observ, label = 'Y observed', c='steelblue')
    plt.plot(self.y2dot/1000, label = 'wX(dot)', c='orangered')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('Y_observed V/S wX(dot)')
    plt.xlabel('Samples @ 40Hz')
    plt.ylabel('acceleration')
    plt.show()
    


  #Displacement
  

  

    

  

if __name__ == "__main__":
  d = deadRecon()
  d.imu_vel()
  d.trajectory()
  d.desplacement()