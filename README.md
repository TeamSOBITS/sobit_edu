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

### How to use

```bash:
$ roslaunch sobit_education_bringup minimal.launch
$ roslaunch sobit_education_bringup 3dsensor.launch
```

## [How to use sobit_education_library](/sobit_education_library)

### service code for moving joints   

```bash:
sobit_education/sobit_education_bringup/src/joint_controller.py
```

###  Folder in joint parameters
```bash:
sobit_education/sobit_education_control/config
```

### smach sample code for moving, grasping

https://gitlab.com/TeamSOBITS/sobit_education/wikis/smach_sample