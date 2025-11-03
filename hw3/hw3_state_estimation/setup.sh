#!/bin/bash

## HW3 Setup Script.
## Places the HW3 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The cs4750_hw3 repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip cs4750_hw3-main.zip
mv ~/Downloads/cs4750_hw3-main/hw3_state_estimation/ ~/homework_ws/src/
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash
roscd arm_particle_filter/
chmod +x test/tracking.py
