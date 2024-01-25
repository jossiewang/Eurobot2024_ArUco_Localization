#!/bin/bash -e

PACKAGES_PATH="$( find ~ -name Vive | awk '{print $1}' | head -1)"

cd $PACKAGES_PATH
cd $PACKAGES_PATH/libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo apt-get install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
make
