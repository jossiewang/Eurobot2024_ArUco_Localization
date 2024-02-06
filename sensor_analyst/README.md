# LiDAR EXP1
## node: 
- lidar_filter_node
    - subscribe: /scan (sensor_msgs/scan)
    - publish: /dist_LD (std_msgs/float32)
    ```roslaunch sensor_analyst lidar_exp1.launch```
    - set param: angle upper bound and lower bound