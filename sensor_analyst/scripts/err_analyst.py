import rospy
import numpy
import pandas as pd
import matplotlib.pyplot as plt

def plot_csv_data(csv_file):
    # Load CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    # Extract data from DataFrame
    timestamp = df['t']
    GPS_x = df['GPS_x']
    GT_x = df['odometry_x']

    # Plotting
    plt.figure(figsize=(10, 6))

    plt.plot(timestamp, GPS_x, label='pos_x')
    plt.plot(timestamp, GT_x, label='GT_x')
    plt.plot(timestamp, GT_x-GPS_x, label='err')

    plt.xlabel('Timestamp')
    plt.ylabel('pos_x')
    plt.title('err-timestamp')
    plt.legend()
    plt.grid(True)

    print("std of GPS_x:", numpy.std(GPS_x))
    print("mean squared error:", (numpy.square(GT_x - GPS_x)).mean())

    # Show the plot
    plt.show()

if __name__ == '__main__':
    try:
        csv_file_path = '/home/jossiewang/eurobot2024_ws/src/Eurobot-2024-Localization/aruco_groundtruth/sensor_analyst/data/matplot_data.csv'
        plot_csv_data(csv_file_path)

    except rospy.ROSInterruptException:
        pass
