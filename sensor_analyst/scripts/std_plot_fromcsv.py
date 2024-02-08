import matplotlib.pyplot as plt
import csv

# Function to load data from CSV file
def load_data_from_file(file_path):
    data = {}
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            marker_id = int(row['Marker ID'])
            data[marker_id] = {
                'mean': [float(row['Mean X']), float(row['Mean Y']), float(row['Mean Z'])],
                'std': [float(row['Std X']), float(row['Std Y']), float(row['Std Z'])]
            }
    return data

# File path
file_path = '/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/sensor_analyst/data/std.csv'

# Load data from file
markers_data = load_data_from_file(file_path)

# Extracting data for plotting
markers_ids = list(markers_data.keys())
x_means = [markers_data[i]['mean'][0] for i in markers_ids]
y_means = [markers_data[i]['mean'][1] for i in markers_ids]
z_means = [markers_data[i]['mean'][2] for i in markers_ids]

x_stds = [markers_data[i]['std'][0] for i in markers_ids]
y_stds = [markers_data[i]['std'][1] for i in markers_ids]
z_stds = [markers_data[i]['std'][2] for i in markers_ids]

# Plotting
plt.errorbar(markers_ids, x_means, yerr=x_stds, fmt='none', label='X', capsize=5)
plt.errorbar(markers_ids, y_means, yerr=y_stds, fmt='none', label='Y', capsize=5)
plt.errorbar(markers_ids, z_means, yerr=z_stds, fmt='none', label='Z', capsize=5)

# Adding labels and legend
plt.xlabel('Marker ID')
plt.ylabel('Value')
plt.title('Standard Deviation for each Marker')
plt.xticks(markers_ids)
plt.legend()
plt.grid(True)
plt.show()
