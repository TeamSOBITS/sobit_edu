# ROS Package for Sobit education

The project is ROS package to control the sobit_education

## Prerequisites

- OS: Ubuntu 16.04  
- ROS distribution: Kinetic Kame

## How to Install

```bash:
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/sobit_education.git
$ git clone https://gitlab.com/TeamSOBITS/sobit_common_msg.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd sobit_education
$ chmod 755 install.sh
$ sudo ./install.sh
$ cd ~/catkin_ws
$ catkin_make
```

## How to use
```bash:
$ roslaunch sobit_education_bringup minimal.launch
$ roslaunch sobit_education_bringup 3dsensor.launch
"""

### service for moving joints
sobit_education/sobit_education_bringup/src/joint_controller.py

### Folder in joint initialization parameters
sobit_education/sobit_education_control/config

### smach sample code folder for moving, grasping
smb://150.37.224.97/ros/sobit_education_smach_sample_files/