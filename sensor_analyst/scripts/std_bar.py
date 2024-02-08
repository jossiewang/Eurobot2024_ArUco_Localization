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
                'std_x': float(row['Std X']),
                'std_y': float(row['Std Y']),
                'std_z': float(row['Std Z'])
            }
    return data

# File path
file_path = '/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/sensor_analyst/data/std.csv'

# Load data from file
markers_data = load_data_from_file(file_path)

# Extracting data for plotting
markers_ids = list(markers_data.keys())
std_x = [markers_data[i]['std_x'] for i in markers_ids]
std_y = [markers_data[i]['std_y'] for i in markers_ids]
std_z = [markers_data[i]['std_z'] for i in markers_ids]

# Plotting
bar_width = 0.2
index = range(len(markers_ids))
plt.bar(index, std_x, bar_width, label='Std X')
plt.bar([i + bar_width for i in index], std_y, bar_width, label='Std Y')
plt.bar([i + bar_width*2 for i in index], std_z, bar_width, label='Std Z')

# Adding labels and legend
plt.xlabel('Marker ID')
plt.ylabel('Standard Deviation')
plt.title('Standard Deviation Comparison for each Marker')
plt.xticks([i + bar_width for i in index], markers_ids)
plt.legend()
plt.grid(True)

# Adding numbers on the bars
for i in index:
    plt.text(i - 0.1, std_x[i] + 0.001, f'{std_x[i]:.4f}', rotation=90, color='black')
    plt.text(i + bar_width - 0.1, std_y[i] + 0.001, f'{std_y[i]:.4f}', rotation=90, color='black')
    plt.text(i + bar_width*2 - 0.1, std_z[i] + 0.001, f'{std_z[i]:.4f}', rotation=90, color='black')

plt.show()
