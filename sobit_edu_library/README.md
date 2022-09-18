# sobit_edu_library
Library to control SOBIT (based on Turtlebot)

---

## How to Install
```bash:
$ sudo apt update 
$ sudo apt install ros-melodic-pybind11-catkin -y
or $ sudo apt install ros-kinetic-pybind11-catkin -y 
$ cm
```

---

## SobitEduController
sobit educationを動かすクラス
### Joint
* sobit educationのジョイント名とその定数名(Enum : Joint)
    ```bash
    * "arm_roll_joint"      :   ARM_ROLL_JOINT = 0
    * "arm_flex_joint"      :   ARM_FLEX_JOINT
    * "elbow_flex_joint"    :   ELBOW_FLEX_JOINT
    * "wrist_flex_joint"    :   WRIST_FLEX_JOINT
    * "hand_motor_joint"    :   HAND_MOTOR_JOINT
    * "xtion_pan_joint"     :   XTION_PAN_JOINT
    * "xtion_tilt_joint"    :   XTION_TILT_JOINT
    ```

### Functions
#### WheelController
1.  controlWheelLinear() :   直線移動
    ```bash
    bool sobit::SobitMiniController::controlWheelLinear( 
        const double distance           :   移動距離[m]
    )
    ```  
2.  controlWheelRotateRad() :   回転移動
    ```bash
    bool sobit::SobitMiniController::controlWheelRotateRad( 
        const double angle_rad           :   回転角度[rad]
    )
    ```  
3.  controlWheelRotateDeg() :   回転移動
    ```bash
    bool sobit::SobitMiniController::controlWheelRotateDeg( 
        const double angle_rad           :   回転角度[deg]
    )
    ```  
    
#### JointController
1.  moveJoint() :   1つのジョイントを動かす関数（ジョイントの指定は定数名(Enum)）
    ```bash
    bool sobit::SobitEduController::moveJoint (
        const Joint joint_num,          :   ジョイント番号
        const double rad,               :   移動角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
2.  moveXtionPanTilt()   :   xtionのパンチルトを任意の角度に動かす
    ```bash
    bool sobit::SobitEduController::moveXtionPanTilt (
        const double pan_rad,           :   パン角度
        const double tilt_rad,          :   チルト角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
3.  moveArm()   :   アームを任意の角度に動かす
    ```bash
    bool sobit::SobitEduController::moveArm ( 
        const double shoulder_roll,
        const double shoulder_flex,
        const double elbow_roll,
        const double hand_motor          
    )
    ```  
4.  movePose()   :   予め設定したポーズに動かす
    ```bash
    bool sobit::SobitEduController::movePose( 
        const std::string &pose_name 
    )
    ```  

    * ポーズのロード方法：sobit_edu_library/launch/load_sobit_edu_pose.launchでポーズをros paramで設定する
    ```bash:
    $ roslaunch sobit_edu_library load_sobit_mini_education.launch
    ```
    
    * ポーズの設定方法：sobit_edu_library/config/sobit_edu_pose.yamlでポーズを設定する
    ```bash
    sobit_edu_pose:
        - { 
            pose_name: "initial_pose",
            arm_roll_joint: 0.00,
            arm_flex_joint: -1.5708, 
            elbow_flex_joint: 1.39, 
            wrist_flex_joint: 0.16, 
            hand_motor_joint: 0.00, 
            xtion_tilt_joint: 0.00, 
            xtion_pan_joint: 0.00 
        }

        - { 
            pose_name: "detecting_pose",
            arm_roll_joint: 0.00,
            arm_flex_joint: 0.00, 
            elbow_flex_joint: 0.00, 
            wrist_flex_joint: 0.00, 
            hand_motor_joint: 0.00, 
            xtion_tilt_joint: 0.53, 
            xtion_pan_joint: 0.00 
        }
    ```  

---

## How to use
### C++

```bash:
#include <sobit_edu_library/sobit_edu_controller.hpp>
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_turtlebot_controller_test");
    sobit::SobitEduController sobit_edu_ctr;
    ros::Rate loop_rate(1.0);
    double pan_ang = 0.8;
    while ( ros::ok() ) {
        pan_ang *= -1.0;
        sobit_edu_ctr.moveJoint( sobit::Joint::XTION_PAN_JOINT, pan_ang, 0.8, false );
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
```

### Python
```bash:
#!/usr/bin/env python
import rospy
from sobit_edu_module import SobitEduController
from sobit_edu_module import Joint
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    edu_ctr = SobitEduController(args[0]) # args[0] : C++上でros::init()を行うための引数
    while not rospy.is_shutdown():
        ang = -1.0 * ang
        edu_ctr.moveJoint( Joint.XTION_PAN_JOINT, ang, 0.8, false )
        r.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
    
```

---