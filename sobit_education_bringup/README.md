# sobit_education_bringup

sobit_educationの基本機能ノード起動用パッケージ．

###How to install
```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/sobit_education_bringup.git
$ git clone https://gitlab.com/TeamSOBITS/sobit_education_description.git
$ git clone https://gitlab.com/TeamSOBITS/sobit_common_msg.git
$ git clone https://gitlab.com/TeamSOBITS/sobit_navigation.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd sobit_education_bringup
$ chmod 755 install.sh
$ sudo ./install.sh
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j4 
```

注意
catkin_makeする際は
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```
でなければ上手く動きません。