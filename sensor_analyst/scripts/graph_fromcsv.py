#!/usr/bin/env python

import matplotlib.pyplot as plt
import csv

def plot_data():
    time = []
    pose_x = []
    pose_y = []
    yaw = []
    vel_x = []
    vel_y = []
    acc_x = []
    acc_y = []
    vel_yaw = []

    with open('aruco_gt.csv', 'r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            time.append(float(row['time']))
            pose_x.append(float(row['pose_x']))
            pose_y.append(float(row['pose_y']))
            yaw.append(float(row['yaw']))
            vel_x.append(float(row['vel_x']))
            vel_y.append(float(row['vel_y']))
            acc_x.append(float(row['acc_x']))
            acc_y.append(float(row['acc_y']))
            vel_yaw.append(float(row['vel_yaw']))

    plt.figure(figsize=(10, 6))
    plt.plot(time, pose_x, label='pose_x')
    plt.plot(time, pose_y, label='pose_y')
    plt.plot(time, yaw, label='yaw')
    plt.plot(time, vel_x, label='vel_x')
    plt.plot(time, vel_y, label='vel_y')
    plt.plot(time, acc_x, label='acc_x')
    plt.plot(time, acc_y, label='acc_y')
    plt.plot(time, vel_yaw, label='vel_yaw')

    plt.xlabel('Time')
    plt.ylabel('Values')
    plt.title('Data from aruco_gt.csv')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    plot_data()
