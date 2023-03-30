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

def isInside(cx, cy, rad, datax, datay):

    datax = np.array(datax)
    datay = np.array(datay)
    d = (((cx-datax)**2) + ((cy-datay)**2) <= rad**2)
    d= np.invert(d)
    
    return np.ma.masked_array(datax, d), np.ma.masked_array(datay, d) 


class MagnCaliberation:
    def __init__(self) -> None:
        plt.rcParams.update({'font.size': 30})
        sns.set_style("dark")
        sns.color_palette("viridis", as_cmap=True)

        bag = bagreader('/home/niket/n1_catkin_ws/src/LAB4/data/data_going_in_circles.bag')
        csv = bag.message_by_topic('/imu')
        self.imu_data_csv_ = pd.read_csv(csv)
        plt.grid(color='grey', linestyle='--', linewidth=1)
        # plt.scatter(readings['mag_field.magnetic_field.x'], readings['mag_field.magnetic_field.y'], marker='.', label='Raw/Uncalibrated Data')
        # doughnut = plt.Circle((0.1, -0.13), 0.2, fill=False, color='black')
        # plt.gca().add_patch(doughnut)
        plt.gca().set_aspect("equal")


    def soft_iron_calib(self):                                 

        x_maj = float(self.hard_iron_x[2000])
        y_maj = float(self.hard_iron_y[2000])

        # X_major, Y_major = isInside(0.0, 0.0, 0.09, X_major, Y_major)

        # SOFT-IRON CALIBRATION
        rad = math.sqrt((x_maj**2) + (y_maj**2))
        print('radius = ', rad)
        self.theta = np.arcsin((y_maj/rad))
        print('theta = ', self.theta)

        rotation_mat = [[np.cos(self.theta), np.sin(self.theta)], [np.sin(-self.theta), np.cos(self.theta)]]
        iron_vals = [self.hard_iron_x, self.hard_iron_y]

        self.mat_multiplic = np.matmul(rotation_mat, iron_vals)
        xtemp, ytemp = isInside(0.0, 0.0, 0.000011, self.mat_multiplic[0], self.mat_multiplic[1])

        print(np.shape(self.mat_multiplic))
        plt.grid(color='grey', linestyle='--', linewidth=1)
        plt.scatter(xtemp, ytemp, marker='x', label = 'Soft-Iron Calibrated', color='palevioletred')
        doughnut = plt.Circle((0.0, 0.0), 0.00001, fill=False, color='black')
        plt.gca().add_patch(doughnut)
        plt.gca().set_aspect("equal")
        plt.title('Soft_Iron_Calibration Of Magnetic Field X vs Y')
        plt.xlabel('Soft_Iron_X (Guass)')
        plt.ylabel('Soft_Iron_Y (Guass)')
        plt.legend()
        plt.show()

    def caliberation(self):
        mag_min_x = min(self.imu_data_csv_['mag_field.magnetic_field.x'])
        mag_max_x = max(self.imu_data_csv_['mag_field.magnetic_field.x'])
        mag_min_y = min(self.imu_data_csv_['mag_field.magnetic_field.y'])
        mag_max_y = max(self.imu_data_csv_['mag_field.magnetic_field.y'])

        # HARD-IRON CALIBRATION
        x_offset = (mag_min_x + mag_max_x)/2.0
        y_offset = (mag_min_y + mag_max_y)/2.0
        print("hard-iron x_axis_Offset=", x_offset)
        print("hard-iron y_axis_Offset=", y_offset)
        self.hard_iron_x = []
        resetp = self.hard_iron_x.extend((self.imu_data_csv_['mag_field.magnetic_field.x']-x_offset))
        self.hard_iron_y = []
        resetq = self.hard_iron_y.extend((self.imu_data_csv_['mag_field.magnetic_field.y']-y_offset))


        self.hard_iron_x, self.hard_iron_y = isInside(0.0, 0.0, 0.0000115, self.hard_iron_x, self.hard_iron_y)

        # X_major = float(self.hard_iron_x[2000])
        # Y_major = float(self.hard_iron_y[2000])

        # # SOFT-IRON CALIBRATION
        # radius = math.sqrt((X_major**2) + (Y_major**2))
        # print('radius = ', radius)
        # self.theta = np.arcsin((Y_major/radius))
        # print('self.theta = ', self.theta)

        # R = [[np.cos(self.theta), np.sin(self.theta)], [np.sin(-self.theta), np.cos(self.theta)]]
        # v = [self.hard_iron_x, self.hard_iron_y]

        # matrix = np.matmul(R, v)  

        plt.grid(color='grey', linestyle='--', linewidth=1)
        plt.scatter(self.hard_iron_x, self.hard_iron_y, marker='+', label='Hard-Iron Calibrated Data', color='crimson')
        # doughnut = plt.Circle((0.0, 0.0), 0.2, fill=False, color='black')
        doughnut = plt.Circle((0.0, 0.0), 0.00001, fill=False, color='black')
        plt.gca().add_patch(doughnut)
        plt.gca().set_aspect("equal")
        plt.title('Hard_Iron_Calibration Plot Of Magnetic Field X vs Y')
        plt.xlabel('Hard_Iron_X (Guass)')
        plt.ylabel('Hard_Iron_Y (Guass)')
        plt.legend()
        plt.show()    


    


    def final_iron_calib(self):

        #Find Major and Minor axis using distance formula
        r = 0.2
        resetq = 0.15
        sigma = resetq/r
        print('sigma = ', sigma)

        rat_matrix2 = [[1, 0], [0, sigma]]
        rotate_mat_malti = np.matmul(rat_matrix2, self.mat_multiplic)
        self.theta = -self.theta
        rotation_mat1 = [[np.cos(self.theta), np.sin(self.theta)], [np.sin(-self.theta), np.cos(self.theta)]]
        iroan_vals1 = np.matmul(rotation_mat1, rotate_mat_malti)
        print(np.shape(iroan_vals1))
        plt.grid(color='grey', linestyle='--', linewidth=1)
        tx, ty = isInside(0.0, 0.0, 0.09, iroan_vals1[0], iroan_vals1[1])
        plt.scatter(tx, ty, marker='x', label='Hard and Soft Iron Calibrated Data', color='indianred')
        doughnut = plt.Circle((0.0, 0.0), 0.00001, fill=False, color='black')
        plt.gca().add_patch(doughnut)
        plt.gca().set_aspect("equal")
        plt.title('Final Calibrated Plot Of Magnetic Field X vs Y')
        plt.xlabel('Mx (Guass)')
        plt.ylabel('My (Guass)')
        plt.rcParams.update({'font.size': 22})
        plt.legend()
        plt.show()

if __name__ == "__main__":
    mc = MagnCaliberation()
    mc.caliberation()
    mc.soft_iron_calib()
    mc.final_iron_calib()