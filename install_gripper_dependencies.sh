#!/bin/bash

mkdir -p ~/ws_gripper_handler/src
cd ~/ws_gripper_handler/src

#dependencies:
git clone https://github.com/Extend-Robotics/DynamixelSDK
wait
git clone https://github.com/Extend-Robotics/ROBOTIS-Framework
wait
git clone https://github.com/Extend-Robotics/ROBOTIS-Framework-msgs
wait
git clone https://github.com/Extend-Robotics/RH-P12-RN-A.git
wait

cd ..
catkin build
wait
