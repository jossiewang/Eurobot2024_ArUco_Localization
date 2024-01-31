# ArUco for Ground Truth

## Procedure in brief


## camera

### usb_cam
#### publish: `/usb_cam/image_raw` `/usb_cam/camera_info`
1. use camera_calibration package to get calibration parameters
    - refer to: https://www.guyuehome.com/34043 , https://drive.google.com/file/d/1J7elOrq3Q9aIE20FrTNwBHpFSBAlwzIk/view?usp=sharing
    - save parameters in ost.yaml
    - add path to launch file
2. check ```/dev/video*``` in /config/usb_cam.yaml
    - can use ```v4l2-ctl --list-devices```
3. ```roslaunch usb_cam usb_cam-test.launch```

### image_proc
#### subscribe: `/usb_cam/image_raw` `/usb_cam/camera_info`
#### publish: `/usb_cam/image_rect`
1. ```ROS_NAMESPACE=usb_cam rosrun image_proc image_proc```

## aruco_ros
#### subscribe: `/usb_cam/image_rect`
#### publish: `aruco_ros/single/marker`
1. in launch file: check
    - marker size (in m)
        ```
        <arg name="markerSize"      default="0.0137"/>
        ```
    - use calibrated image
        ```
        <remap from="/camera_info" to="/usb_cam/camera_info" />
        <remap from="/image" to="/usb_cam/image_rect" />
        ```
2. ```roslaunch aruco_ros single.launch```

## aruco_locate

## sensor_analyst