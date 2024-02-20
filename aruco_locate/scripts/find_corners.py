import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
import csv

# Search for nearest mea_x and mea_y values
def find_nearest(data, value, key):
    prev = None
    for entry in data:
        if float(entry[key]) >= value:
            if prev:
                return prev, entry
            else:
                return entry, entry
        prev = entry

def find_corners(data, mea_x, mea_y):
    # Find nearest mea_x and mea_y values
    nearest_x_lower, nearest_x_upper = find_nearest(data, mea_x, 'mea_x')
    nearest_y_lower, nearest_y_upper = find_nearest(data, mea_y, 'mea_y')

    # Identify the four corners
    corner1 = next((entry for entry in data if entry["real_x"] == nearest_x_lower["real_x"] and entry["real_y"] == nearest_y_lower["real_y"]), None)
    corner2 = next((entry for entry in data if entry["real_x"] == nearest_x_lower["real_x"] and entry["real_y"] == nearest_y_upper["real_y"]), None)
    corner3 = next((entry for entry in data if entry["real_x"] == nearest_x_upper["real_x"] and entry["real_y"] == nearest_y_lower["real_y"]), None)
    corner4 = next((entry for entry in data if entry["real_x"] == nearest_x_upper["real_x"] and entry["real_y"] == nearest_y_upper["real_y"]), None)

    # Retrieve real_x and real_y values for corners
    corners = [
        {'real_x': float(corner1['real_x']), 'real_y': float(corner1['real_y'])},
        {'real_x': float(corner2['real_x']), 'real_y': float(corner2['real_y'])},
        {'real_x': float(corner3['real_x']), 'real_y': float(corner3['real_y'])},
        {'real_x': float(corner4['real_x']), 'real_y': float(corner4['real_y'])}
    ]

    return corners

# Find the entry with the next higher value for the given key
def find_next_entry(data, current_entry, key):
    for entry in data:
        if float(entry[key]) > float(current_entry[key]):
            return entry

    # If no higher value found, return the last entry
    return data[-1]

# Read CSV file and store contents
csv_data = []
with open('/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/aruco_locate/table/table_test.csv', 'r') as csv_file:
    csv_reader = csv.DictReader(csv_file)
    for row in csv_reader:
        csv_data.append(row)

corners = find_corners(csv_data, 0.54, 0.73) #0,1 1,2 0 0.1 
print(corners) #[{'real_x': 5.0, 'real_y': 7.0}, {'real_x': 5.0, 'real_y': 8.0}, {'real_x': 6.0, 'real_y': 7.0}, {'real_x': 6.0, 'real_y': 8.0}]

