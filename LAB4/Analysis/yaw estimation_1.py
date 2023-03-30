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
from quaternion import quaternion, from_rotation_vector, rotate_vectors
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal

class YawEstimate:
  def __init__(self) -> None:
    sns.set_style("dark")
    sns.color_palette("viridis", as_cmap=True)
    plt.rcParams.update({'font.size': 30})

    bag = bagreader('/home/niket/n1_catkin_ws/src/LAB4/data/data_driving.bag')
    data = bag.message_by_topic('/imu')
    self.imu_data_csv = pd.read_csv(data)

    # plt.grid(color='grey', linestyle='--', linewidth=1)
    # plt.scatter(self.imu_data_csv['mag_field.magnetic_field.x'], self.imu_data_csv['mag_field.magnetic_field.y'], marker='.', label='Raw/Uncalibrated Data')
    # doughnut = plt.Circle((0.1, -0.13), 0.2, fill=False, color='black')
    # plt.gca().add_patch(doughnut)
    # plt.gca().set_aspect("equal")

  def yaw_calc(self):
    orient_w = self.imu_data_csv['imu.orientation.w']
    orient_x = self.imu_data_csv['imu.orientation.x']
    orient_y = self.imu_data_csv['imu.orientation.y']
    orient_z = self.imu_data_csv['imu.orientation.z']

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

    print('v1', self.v1, self.v1.shape)
    mag_x = self.v1[:, 0]
    mag_y = self.v1[:, 1]
    print('mag_x', mag_x)
    print('mag_y', mag_y)
    mag_z1 = self.imu_data_csv['mag_field.magnetic_field.z']
    mad_x_read = self.imu_data_csv['mag_field.magnetic_field.x']
    mag_y_read = self.imu_data_csv['mag_field.magnetic_field.y']
    mag_z_read = self.imu_data_csv['mag_field.magnetic_field.z']
    print('data_x', mad_x_read)
    print('mag_z1', mag_z1)
    mag_z2 = mag_z1.to_numpy()
    mag_z = np.reshape(mag_z2, (1, 47809))

    # print(data_x*list(np.cos(pitch)))
    # exit()
    # YAW calculation with calibrated data
    x_a = mag_z*list(np.sin(roll))
    x_b = mag_y*list(np.cos(roll))
    X = x_a - x_b
    y_a = mag_x*list(np.cos(pitch))
    y_b = mag_y*list(np.sin(pitch)*np.sin(roll))
    y_c = mag_z*list(np.sin(pitch)*np.cos(roll))

    Y = y_a+y_b+y_c
    yaw_calib = np.arctan2(-Y, X)
    temp = np.unwrap(yaw_calib)
    max_yaw_calib = max(temp)
    final_yaw_calib = pd.Series(max_yaw_calib)
    final_yaw_calib = final_yaw_calib * (180 / np.pi)


    # YAW calculation with non_calibrated data
    x_ra = mag_z_read*list(np.sin(roll))
    x_rb = mag_y_read*list(np.cos(roll))


    raw_X = x_ra - x_rb
    y_ra = mad_x_read*list(np.cos(pitch))
    y_rb = mag_y_read*list(np.sin(pitch)*np.sin(roll))
    y_rc = mag_z_read*list(np.sin(pitch)*np.cos(roll))

    y_ra = y_ra.squeeze()
    y_rb = y_rb.squeeze()
    y_rc = y_rc.squeeze()

    raw_Y = y_ra + y_rb + y_rc
    raw_X = raw_X.squeeze()

    print('raw_X', raw_X)
    print('raw_Y', raw_Y)

    cal_raw_yaw = np.arctan2(-raw_Y, raw_X)
    print(cal_raw_yaw)
    cal_raw_yaw1 = np.unwrap(cal_raw_yaw)
    final_cal_raw_yaw = pd.Series(cal_raw_yaw)
    final_cal_raw_yaw = final_cal_raw_yaw * (180 / np.pi)


    # print(final_cal_raw_yaw)
    # exit()
    # Integration
    gyro_int = integrate.cumtrapz(self.imu_data_csv['imu.angular_velocity.z'], initial=0)
    gyro_int_wrap = np.unwrap(gyro_int)
    plt.figure(figsize = (16,8))
    plt.plot(gyro_int, label='Gyro Integrated Yaw', c='palevioletred')
    plt.plot(final_yaw_calib, label='Calibrated Yaw')
    plt.plot(final_cal_raw_yaw, label='Raw Yaw', c='lightseagreen')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('Estimation of Yaw for Magnetometer')
    plt.xlabel('Samples @ 40Hz')
    plt.ylabel('Yaw (degrees)')
    plt.show()


    # Filteration
    lpf = signal.filtfilt(*butter(3, 0.1, "lowpass",fs = 40, analog=False), final_yaw_calib)
    hpf = signal.filtfilt(*butter(3, 0.0001, 'highpass', fs = 40, analog=False), gyro_int)
    #print(lpf[0]/10+hpf[0]/10)
    plt.figure(figsize = (16,8))
    plt.plot(lpf, label='LPF Calibrated Yaw')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.plot(hpf, label = 'HPF Gyro Yaw', c='seagreen')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.title('LPF for Magnetic Yaw and HPF for Gyro Yaw')
    plt.xlabel('Samples @ 40Hz')
    plt.ylabel('Yaw (degrees)')
    plt.show()

    #Original Yaw V/S Calibrated Yaw
    alpha = 0.75
    omega = self.imu_data_csv['imu.angular_velocity.z']
    yaw_filtered = []
    yaw_filtered = np.append(yaw_filtered,0)
    for i in range(47808):
      j = i+1
      yaw_filtered = np.append(yaw_filtered, alpha*(yaw_filtered[i] + hpf[j]*0.05) + ((1-alpha)*lpf[j]))
    # lpf1 = 1 - hpf1
    # yaw_filtered = (hpf1*hpf) + (lpf1*lpf)
    plt.figure(figsize=(16, 8))
    plt.plot(yaw_filtered, label='Complementary Filter')
    plt.plot(yaw_z*200, label='Yaw computed by imu')
    plt.legend(loc='lower right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.xlabel('Samples @ 40 Hz')
    plt.ylabel('Yaw (degrees)')
    plt.title('imu Yaw vs Complementary Filter Yaw')
    plt.show()


    #LPF for Yaw v/s HPF gor gyro v/x Complementary Yaw
    plt.figure(figsize = (16,8))
    plt.plot(lpf, label='LPF Calibrated Yaw',c= 'teal')
    plt.legend(loc = "upper right")
    plt.plot(hpf, label = 'HPF Gyro Yaw')
    plt.plot(yaw_filtered, label='Complementary Filter',c= 'crimson')
    plt.legend(loc='upper right', fontsize='x-large')
    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.legend(loc="upper right")
    plt.xlabel('Samples @ 40Hz')
    plt.ylabel('Yaw (degrees)')
    plt.title('LPF for Magnetic Yaw V/S HPF for Gyro Yaw V/S  Complimentary Yaw')
    plt.show()

  def caliberation(self): 
    min_mg_x = min(self.imu_data_csv['mag_field.magnetic_field.x'])
    max_mg_x = max(self.imu_data_csv['mag_field.magnetic_field.x'])
    min_mg_y = min(self.imu_data_csv['mag_field.magnetic_field.y'])
    max_mg_y = max(self.imu_data_csv['mag_field.magnetic_field.y'])

    # HARD-IRON CALIBRATION
    x_offset = (min_mg_x + max_mg_x)/2.0
    y_offset = (min_mg_y + max_mg_y)/2.0
    print("hard-iron x_axis_Offset=", x_offset)
    print("hard-iron y_axis_Offset=", y_offset)
    hard_iron_x = []
    resetp = hard_iron_x.extend((self.imu_data_csv['mag_field.magnetic_field.x']-x_offset))
    hard_iron_y = []
    resetq = hard_iron_y.extend((self.imu_data_csv['mag_field.magnetic_field.y']-y_offset))

    plt.grid(color='grey', linestyle='--', linewidth=1)
    plt.scatter(hard_iron_x, hard_iron_y, marker='+', label='Hard-Iron Calibrated Data', color='crimson')
    doughnut = plt.Circle((0.0, 0.0), 0.2, fill=False, color='black')
    plt.gca().add_patch(doughnut)
    plt.gca().set_aspect("equal")
    plt.title('Hard_Iron_Calibration Plot Of Magnetic Field X vs Y')
    plt.xlabel('Hard_Iron_X (Guass)')
    plt.ylabel('Hard_Iron_Y (Guass)')
    plt.legend()
    plt.show()

    x_maj = float(hard_iron_x[2000])
    y_maj = float(hard_iron_y[2000])

    # SOFT-IRON CALIBRATION
    rad = math.sqrt((x_maj**2) + (y_maj**2))
    print('radius = ', rad)
    theta = np.arcsin((y_maj/rad))
    print('theta = ', theta)

    rotation_mat = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
    iron_vals = [hard_iron_x, hard_iron_y]

    iorn_matrix = np.matmul(rotation_mat, iron_vals)
    print(np.shape(iorn_matrix))
    # plt.grid(color='grey', linestyle='--', linewidth=1)
    # plt.scatter(matrix[0], matrix[1], marker='x', label = 'Soft-Iron Calibrated', color='palevioletred')
    # doughnut = plt.Circle((0.0, 0.0), 0.2, fill=False, color='black')
    # plt.gca().add_patch(doughnut)
    # plt.gca().set_aspect("equal")
    # plt.title('Soft_Iron_Calibration Of Magnetic Field X vs Y')
    # plt.xlabel('Soft_Iron_X (Guass)')
    # plt.ylabel('Soft_Iron_Y (Guass)')
    # plt.legend()
    # plt.show()

    # Find Major and Minor axis using distance formula
    r = 0.2
    resetq = 0.15
    sigma = resetq/r
    print('sigma = ', sigma)

    # Scaling
    iron_matrix2 = [[1, 0], [0, sigma]]
    rotate = np.matmul(iron_matrix2, iorn_matrix)
    theta = -theta
    R1 = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
    self.v1 = np.matmul(R1, rotate)
    self.v1 = np.expand_dims(self.v1, axis=0)
    # plt.grid(color='grey', linestyle='--', linewidth=1)
    # plt.scatter(self.v1[:, 0], self.v1[:, 1], marker='x', label='Hard and Soft Iron Calibrated Data', color='indianred')
    # doughnut = plt.Circle((0.0, 0.0), 0.15, fill=False, color='black')
    # plt.gca().add_patch(doughnut)
    # plt.gca().set_aspect("equal")
    # plt.title('Final Calibrated Plot Of Magnetic Field X vs Y')
    # plt.xlabel('Mx (Guass)')
    # plt.ylabel('My (Guass)')
    # plt.legend()
    # plt.show()


  

if __name__ == "__main__":
  ye = YawEstimate()
  ye.caliberation()
  ye.yaw_calc()