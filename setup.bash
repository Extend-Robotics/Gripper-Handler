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
git clone https://github.com/Extend-Robotics/Gripper-Handler.git
wait

cd ..
catkin build
wait

echo "source ~/ws_gripper_handler/devel/setup.bash # from setup_gripper_handler.bash" >> ~/.bashrc

# Create real-time scheduling priority (rtprio) for USER_GROUP (your user group)
sudo bash -c 'echo "@extend - rtprio 99" > /etc/security/limits.d/robotis-rtprio.conf'
# Register USER_ID (your user ID) to dialout group in order to gain access to /dev/ttyUSB0
sudo usermod -aG dialout extend

# Restart or log out, and log in to validate the change in order to gain access to /dev/ttyUSB0
