# Eurobot-2024-Localization

> Eurobot localization workspace for 2024

- elaborate README allow a team to grow!
- clear commit messages are benefitial for all!

## Install ( unupdate )
```bash=1
# Move in ws
chmod 777 install.sh
./install.sh
source ~/.bashrc
```

## Structure

```
.
└──  Your Workspace
     └── build
     └── devel
     └── src
         └── Eurobot-2024-Localization
             ├── .YDLidar-SDK
             ├── docker
             ├── eurobot_localization
             ├── rival_localization
             ├── lidar
             │   ├── lidar_localization
             │   └── ydlidar_ros_driver
             ├── local_filter
             │   ├── imu
             │   │   ├── imu_drive
             │   │   └── phidgets_drivers
             │   ├── local_filter
             │   └── odometry
             │       ├── odometry
             │       ├── rosserial_msgs
             │       └── rosserial_server
             ├── simulation
             └── vive

```


## Architecture
> Local filter ( IMU + Odometry ) + global filter ( LiDAR )

### Local filter

- Place all of the required component in local filter
- Run with rosserial and imu firmmware
```bash=1
roslaunch local_filter local_filter.launch
```
- Run without rosserial but imu firmware
```bash=1
roslaunch local_filter local_filter_no_comm.launch # no such file now!
```
- Run without rosserial and imu firmware
```bash=1
roslaunch local_filter local_filter_no_firmware.launch # no such file now!
```

### Global filter

- Place LiDAR driver and triangle localization in lidar
- Place global filter in eurobot_localization
- Run with only triangle localization
```bash=1
roslaunch lidar_localization lidar_localization_2023.launch # no such file now!
```
- Run with only lidar driver and triangle localization
```bash=1
roslaunch lidar_localization lidar_with_driver.launch
```
- Run global filter with lidar driver
```bash=1
roslaunch eurobot_localization global_ekf.launch
```
- Run global filter without lidar driver
```bash=1
roslaunch eurobot_localization global_ekf_without_lidar.launch # no such file now!
```

### Together

- Run all of the localization component
```bash=1
roslaunch eurobot_localization eurobot_localization.launch
```

### Local machine setup configure

- Port name
- Static TF for laser frame and imu frame
