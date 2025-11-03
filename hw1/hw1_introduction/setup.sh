#!/bin/bash

## HW1 Setup Script.
## Places the HW1 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The hw1_release repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip hw1_release-main.zip
cd ~
mkdir -p homework_ws/src
mv ~/Downloads/hw1_release-main/* ~/homework_ws/src/
cd ~/homework_ws
catkin build 
source ~/homework_ws/devel/setup.bash 
source ~/dependencies_ws/devel/setup.bash
