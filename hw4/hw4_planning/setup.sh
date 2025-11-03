#!/bin/bash

## HW4 Setup Script.
## Places the HW4 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The hw4_release repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip hw4_release-main.zip
mv ~/Downloads/hw4_release-main/hw4_planning/ ~/homework_ws/src/cs4750_student
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash
