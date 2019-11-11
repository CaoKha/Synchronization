# Synchronization
This project was done when I was working in Trimble at Nantes as trainee. My project is to validate the time received data from different sensors on different hardware such as Raspberry Pi, Up board, Udoo x86 which was later used on a mobile robot for navigation. The goal is to synchronize data coming from camera and IMU.
## BNO055 
BNO055 is an already fusion and filtered IMU sensor that has a data rate of 100 Hz. For more info, https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
## MPU9255
MPU9255 is a cheap IMU sensor that give non filtered data.
## D435 
Camera D435 from Intel Realsense.
## DXL 430
Dynamixel motors from ROBOTIS. The script is to debug the motor of the mobile robot
## Info about this repo
This repository is developed using vscode with platformio plugin. 
This repository is also using ROS Kinetic on Ubuntu 16.04 LTS. All the folders have ros in the file name will require ROS to function.
