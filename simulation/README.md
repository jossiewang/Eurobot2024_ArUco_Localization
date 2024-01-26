# Simulation

## About this package

- This package use **Gazebo** to simulation a world with three cylindrical beacons, and a turtlebot with imu and LiDAR. 
- You can simulate by controling the robot, collecting the sensor data and visualize the data with **rviz**.

### The project status

- now we have a world with the 3mm*2mm playground and three fixed beacons, a diff-drive robot, and a robot_steering controller(cmd_vel publisher). We can get /odom, /scan and /imu msgs from the simulated sensors.
- note that it's usable for code testing, but the physical parameters still need to be checked.

## How to use

### launch

```bash=
roscore
# open another terminal
roslaunch simulation sim.launch
```

### control the robot

- To control the robot moving, publish msg in type `geometry_msgs/Twist` on the topic `/cmd_vel`. 
- You can do this by a node, [rqt_robot_steering](http://wiki.ros.org/rqt_robot_steering), or [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard).

### data

- Gazebo would publish: 
  - Odometry data in type `nav_msgs/Odometry` on the topic `/odom`.
  - LiDAR data in type `sensor_msgs/LaserScan` on the topic `/scan`.
  - imu data in type `sensor_msgs/Imu ` on the topic `/imu`.

### parameters setting

- You can decide the initial pose of the robot in the file `simulation/launch/sim.launch`, line 18.
- ~~You can open/close publishing odom->base_footprint tf in the file`simulation/urdf/robot_simulation.gazebo.xacro`, line 64.~~