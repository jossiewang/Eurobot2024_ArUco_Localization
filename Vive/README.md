# Eurobot-Localization/Vive
> Vive workspace for Eurobot

> Contributors: Angus and YuShan

## Install
```bash=1
# create a new catkin_workspace, if you need to.
cd ~/catkin_workspace/src
git clone git@github.com:YuShan122/Vive.git

cd Vive
chmod 777 vive_install.sh
./vive_install.sh

# If error occurs
# sudo apt-get install qtbase5-dev
# sudo apt-get install qtdeclarative5-dev
# sudo apt-get install libarmadillo-dev
```

## complie
```bash=1
cd ~/catkin_workspace
catkin_make
# if error occurs, compile "obstacle_detector" first as follow:
catkin_make -DCATKIN_WHITELIST_PACKAGES="obstacle_detector"
catkin_make -DCATKIN_WHITELIST_PACKAGES="vive"
```

## Calibration
### Getting LH->map
- Set up three lighthouses.
- Check dump path in `vive_calibrate.cpp`, line 278. Remember `catkin_make` if the code is modified.
- Open `vive_calibrate.launch`. Check and modify value of params: `calibrate_tracker` `side`.
- Put the tracker on the playground. 
  - Position: (1.5, 1.0) (the center of the playground), and 
  - Orientation: LED facing right shorter side of the playground(the line x=3). 
- Launch. 
```bash=1
roslaunch vive vive_calibrate.launch
```
- Wait untill the numbers printed to be stable. 
  - If you don't want to replace the last calibration:
  ```bash=1
  # open another terminal
  rosparam set /vive_calibrate/dump_blue false
  rosparam set /vive_calibrate/dump_green false
  ```
- Press `ctrl`+`C` to stop the program. `vive/param/vive_calibrate.yaml` will be replaced by new calibration.
### Calibration of map_origin rotation
- Set up three lighthouses.
- Check dump path in `vive_calibrate2.cpp`, line 380. Remember `catkin_make` if the code is modified.
- Open `vive_calibrate2.launch`, check and modify args: `side_` `tracker_`.
- Put the tracker on the playground.
- Launch.
```bash=1
roslaunch vive vive_calibrate2.launch
# open another terminal
rosrun vive vive_calibrate_contrl
```
- There are four points; the position of the points are set in `vive_calibrate.yaml` -> `pos_true_xandy`.
- Input numbers with `vive_calibrate_contrl`, and put tracker on the position `vive_calibrate2` gives you.
- When finishing one point, **before moving tracker**, input another number in `vive_calibrate_contrl`.
- Then move the tracker to next posistion. 
- If all four points are finished, close `vive_calibrate2` (by input `9` or press `ctrl`+`C`). The calibration data would be written into `vive_calibrate.yaml`.
- Finished.
## Run
### robot
- Open `vive_trackerpose.launch`, check and modify args: 
  - `robot1_active` / `robot2_active` 
  - `robot1_tracker` / `robot2_tracker`
  - `side` : 'g' or 'b'. Note: We didn't use this string param in EUROBOT2023, but an int param `/side`. 1 green, 0 blue. This param is set by other.
  - `ekf_` : running with ekf or not. If true, the program would compare the position of vive and ekf.
  - `print_world` : Print information of `vive_world` or not. Note: SimpleApi logger in `vive_world` would always print information.
- Launch.
```bash=1
rosparam set /side 0 # blue, or
rosparam set /side 1 # green
roslaunch vive vive_trackerpose.launch
```

### rival
- Open `vive_rival_1.launch`/`vive_rival_2.launch`, check and modify args: 
  - `rival1_active` / `rival2_active` 
  - `rival1_tracker` / `rival2_tracker`
  - `side` : 'g' or 'b'. Note: We didn't use this string param in EUROBOT2023, but an int param `/side`. 1 green, 0 blue. This param is set by other.
  - `lowpass_active_` : to determined the lowpass filter active. 
- Launch.
```bash=1
rosparam set /side 0 # blue, or
rosparam set /side 1 # green
roslaunch vive vive_rival_1.launch # or
roslaunch vive vive_rival_2.launch
```
