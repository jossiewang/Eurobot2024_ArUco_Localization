# Eurobot-Localization/Vive
> Vive workspace for Eurobot

> Contributors: Angus and YuShan and modified by weifu-yee

# Process
1. `calibrate.launch`
2. `calibrate2.launch` and `calibrate2_control`
3. `rival.launch`


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
### Getting LH->map (calibrate)
- Open `vive_calibrate.launch`. Check and modify value of params: `num_LH` `calibrate_tracker` `side`.
- Put the tracker on the playground. 
  - Position: (1.5, 1.0) (the center of the playground), and 
  - Orientation: LED facing right shorter side of the playground(the line x=3). 
- Launch. 
  ```bash=1
  $ roslaunch vive calibrate.launch
  ```
- Wait untill the numbers printed to be stable. 
  - If you don't want to replace the last calibration:
    ```bash=1
    # open another terminal
    $ rosparam set /calibrate/dump_blue false
    $ rosparam set /calibrate/dump_yellow false
    ```
- Press `ctrl`+`C` to stop the program. `vive/param/vive_calibrate.yaml` will be replaced by new calibration.
- NOTE: there are two cases:
  - 1.Only **one** LH, then you need only calibrate one side(blue or yellow).
  - 2.Use **two** LHs, then you need to calibrate blue one time and yellow another time. Remember modify the `side` before change the side you want calibrate.
### Calibration of map_origin rotation (calibrate2)
- Open `vive_calibrate2.launch`, check and modify args: `side_` `tracker_`.
- Put the tracker on the playground.
- Launch.
  ```bash=1
  $ roslaunch vive calibrate2.launch
  # open another terminal
  $ rosrun vive calibrate_contrl
  ```
- There are four points; the position of the points are set in `vive_calibrate.yaml` -> `pos_true_x` and `pos_true_y`.
- Input numbers with `vive_calibrate_contrl`, and put tracker on the position `calibrate2` gives you.
- When finishing one point, **before moving tracker**, input another number in `calibrate_contrl`.
- Then move the tracker to next posistion. 
- If all four points are finished, close `calibrate2` (by input `9` in calibrate_control or press `ctrl`+`C`). The calibration data would be written into `vive_calibrate.yaml`.
- Finished.
## Run

### rival
- Open `rival.launch`, check and modify args: 
  - `rival_active` 
  - `rival_tracker`
  - `side` : 'b' or 'y'.
  - `lowpass_active_` : to determined the lowpass filter active. 
- Launch.
  ```bash=1
  # roslaunch vive vive_rival.launch
  ```
- Now, the rostopic `/rival/odom/tracker` will be published.
## config file
- The LH object will be build in the `config.json`
```
~/.config/libsurvive/config.json
```