import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read the data from the CSV file
df = pd.read_csv("/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/sensor_analyst/data/sample.csv")

# Extract relevant columns
real_x = df['real_x']
real_y = df['real_y']
raw_x = df['raw_x']
raw_y = df['raw_y']
raw_z = df['raw_z']

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot raw data points
ax.scatter(raw_x, raw_y, raw_z, c='b', marker='o', label='Raw Data')

# Plot real data points
# ax.scatter(real_x, real_y, 0, c='r', marker='^', label='Real Data')

# Set equal aspect ratio
max_range = max(raw_x.max() - raw_x.min(), raw_y.max() - raw_y.min(), raw_z.max() - raw_z.min())
mid_x = (raw_x.max() + raw_x.min()) * 0.5
mid_y = (raw_y.max() + raw_y.min()) * 0.5
mid_z = (raw_z.max() + raw_z.min()) * 0.5
ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
ax.set_ylim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)
ax.set_zlim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Scatter Plot of Raw')

# Add a legend
ax.legend()

# Show plot
plt.show()
