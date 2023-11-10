<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS EDU

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li>
    　<a href="#実行・操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#移動機構のみを使用する場合">移動機構のみを使用する場合</a></li>
        <li><a href="#Rviz上の可視化">Rviz上の可視化</a></li>
      </ul>
    </li>
    <li>
    　<a href="#ソフトウェア">ソフトウェア</a>
      <ul>
        <li><a href="#ジョイントコントローラ">ジョイントコントローラ</a></li>
        <li><a href="#ホイルコントローラ">ホイルコントローラ</a></li>
      </ul>
    </li>
    <li>
    　<a href="#ハードウェア">ハードウェア</a>
      <ul>
        <li><a href="#パーツのダウンロード方法">パーツのダウンロード方法</a></li>
        <li><a href="#ロボットの組み立て">ロボットの組み立て</a></li>
        <li><a href="#ロボットの特徴">ロボットの特徴</a></li>
        <li><a href="#部品リスト（BOM）">部品リスト（BOM）</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#変更履歴">変更履歴</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

![SOBIT EDU](sobit_pro/docs/img/sobit_edu.png)

TurtleBot2をベースとしてSOBITSが開発したモバイルマニピュレータ（SOBIT EDU）を動かすためのライブラリです．

> [!WARNING]
> 初心者の場合，実機のロボットを扱う際に，先輩方に付き添ってもらいながらロボットを動かしましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.0~ |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBIT Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ cd src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/sobit_edu
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd sobit_edu/
   ```
4. 依存パッケージをインストールします．
   ```sh
   $ bash install.sh
   ```
5. パッケージをコンパイルします．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ catkin_make
   ```


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- 実行・操作方法 -->
## 実行・操作方法

1. SOBIT EDUの起動する機能をパラメタとして[minimal.launch](sobit_edu_bringup/launch/minimal.launch)に設定します．
   ```xml
    <!-- Activate Mobile-Base (true), Arm (true), Head (true) -->
    <arg name="enable_mb"           default="true"/>
    <arg name="enable_arm"          default="true"/>
    <arg name="enable_head"         default="true"/>
    ...
    <arg name="open_rviz"           default="true"/>
    ...
   ```
    > [!NOTE]
    > 使用したい機能に応じて，`true`か`false`かに書き換えてください．

2. [minimal.launch](sobit_edu_bringup/launch/minimal.launch)というlaunchファイルを実行します．
   ```sh
   $ roslaunch sobit_edu_bringup minimal.launch
   ```
3. [任意] デモプログラムを実行してみましょう．
   ```sh
   $ rosrun sobit_edu_library test_controll_wheel.py
   ```

> [!NOTE]
> SOBIT EDUの動作方法になれるため，[example](sobit_edu_library/example/)フォルダを確認し，それぞれのサンプルファイルから動作関数を学びましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Rviz上の可視化
実機を動かす前段階として，Rviz上でSOBIT EDUを可視化し，ロボットの構成を表示することができます．

```sh
$ roslaunch sobit_edu_description display.launch
```

正常に動作した場合は，次のようにRvizが表示されます．
![SOBIT EDU Display with Rviz](sobit_edu/docs/img/sobit_edu_display.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ソフトウェア
<details>
<summary>SOBIT EDUと関わるソフトの情報まとめ</summary>


### ジョイントコントローラ
SOBIT_EDUのパンチルト機構とマニピュレータを動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 動作関数
1.  `moveToPose()` : 決められたポーズに動かします．
    ```cpp
    bool moveToPose(
        const std::string& pose_name,   # ポーズ名
        const double sec = 5.0          # 動作時間 (s)
    );
    ```
> [!NOTE]
> 既存のポーズは[sobit_edu_pose.yaml](sobit_edu_library/config/sobit_edu_pose.yaml)に確認でいます．ポーズの作成方法については[ポーズの設定方法](#ポーズの設定方法)をご参照ください．

1.  `moveJoint()` : 指定されたジョイントを任意の角度も動かします．
    ```cpp
    bool sobit::SobitProJointController::moveJoint (
        const Joint joint_num,          # ジョイント名 (定数名)
        const double rad,               # 回転角度 (rad)
        const double sec = 5.0,         # 回転時間 (s)
        bool is_sleep = true            # 回転後に待機するかどうか
    );
    ```
> [!NOTE]
> `ジョイント名`は[ジョイント名](#ジョイント名)をご確認ください．
 
1.  `moveArm()` : アームの関節を任意の角度に動かします．
    ```cpp
    bool sobit::SobitProJointController::moveArm(
        const double arm1,              # ARM_SHOULDER_TILT_JOINTの回転角度 (rad)
        const double arm2,              # ARM_ELBOW_UPPER_TILT_JOINTの回転角度 (rad)
        const double arm3,              # ARM_ELBOW_LOWER_TILT_JOINTの回転角度 (rad)
        const double arm3_pan,          # ARM_ELBOW_LOWER_PAN_JOINTの回転角度 (rad)
        const double arm4,              # ARM_WRIST_TILT_JOINTの回転角度 (rad)
        const double sec = 5.0,         # 回転時間 (s)
        bool is_sleep = true            # 回転後に待機するかどうか
    );
    ```

1.  `moveHeadPanTilt()` : パンチルト機構を任意の角度に動かす
    ```cpp
    bool sobit::SobitProJointController::moveHeadPanTilt(
        const double head_camera_pan,   # パンの回転角度 (rad)
        const double head_camera_tilt,  # チルトの回転角度 (rad)
        const double sec = 5.0,         # 移動時間 (s)
        bool is_sleep = true            # 回転後に待機するかどうか
    );
    ```

1.  `moveGripperToTargetCoord()` : ハンドをxyz座標に動かします（把持モード）．
    ```cpp
    bool sobit::SobitProJointController::moveGripperToTargetCoord(
        const double goal_position_x,       # 把持目的地のx (m)
        const double goal_position_y,       # 把持目的地のy (m)
        const double goal_position_z,       # 把持目的地のz (m)
        const double diff_goal_position_x,  # xyz座標のx軸をシフトする (m)
        const double diff_goal_position_y,  # xyz座標のy軸をシフトする (m)
        const double diff_goal_position_z   # xyz座標のz軸をシフトする (m)
    );
    ```

1.  `moveGripperToTargetTF()` : ハンドをtf名に動かします（把持モード）．
    ```cpp
    bool sobit::SobitProJointController::moveGripperToTargetTF(
        const std::string& target_name,     # 把持目的tf名
        const double diff_goal_position_x,  # xyz座標のx軸をシフトする (m)
        const double diff_goal_position_y,  # xyz座標のy軸をシフトする (m)
        const double diff_goal_position_z   # xyz座標のz軸をシフトする (m)
    );
    ```

1.  `moveGripperToPlaceCoord()` : ハンドをxyz座標に動かします（配置モード）．
    ```cpp
    bool sobit::SobitProJointController::moveGripperToPlaceCoord(
        const double goal_position_x,       # 配置目的地のx (m)
        const double goal_position_y,       # 配置目的地のx (m)
        const double goal_position_z,       # 配置目的地のx (m)
        const double diff_goal_position_x,  # xyz座標のx軸をシフトする (m)
        const double diff_goal_position_y,  # xyz座標のy軸をシフトする (m)
        const double diff_goal_position_z   # xyz座標のz軸をシフトする (m)
    ); 
    ```

1.  `moveGripperToPlaceTF()` : ハンドをtf名に動かします（配置モード）．
    ```cpp
    bool sobit::SobitProJointController::moveGripperToPlaceTF(
        const std::string& target_name,     # 配置目的tf名
        const double diff_goal_position_x,  # xyz座標のx軸をシフトする (m)
        const double diff_goal_position_y,  # xyz座標のy軸をシフトする (m)
        const double diff_goal_position_z   # xyz座標のz軸をシフトする (m)
    );
    ```

1.  `graspDecision()` : 定めた範囲内の電流値を超えた場合，配置判定を返す．
    ```cpp
    bool sobit::SobitProJointController::graspDecision( );
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ジョイント名
SOBIT PROのジョイント名とその定数名を以下の通りです．


| ジョイント番号 | ジョイント名 | ジョイント定数名 |
| :---: | --- | --- |
| 1 | arm_shoulder_1_tilt_joint | ARM_SHOULDER_1_TILT_JOINT |
| 2 | arm_shoulder_2_tilt_joint | ARM_SHOULDER_2_TILT_JOINT |
| 3 | arm_elbow_upper_1_tilt_joint | ARM_ELBOW_UPPER_1_TILT_JOINT |
| 4 | arm_elbow_upper_2_tilt_joint | ARM_ELBOW_UPPER_2_TILT_JOINT |
| 5 | arm_elbow_lower_tilt_joint | ARM_ELBOW_LOWER_TILT_JOINT |
| 6 | arm_elbow_lower_pan_joint | ARM_ELBOW_LOWER_PAN_JOINT |
| 7 | arm_wrist_tilt_joint | ARM_WRIST_TILT_JOINT |
| 8 | hand_joint | HAND_JOINT |
| 9 | head_camera_pan_joint | HEAD_CAMERA_PAN_JOINT |
| 10 | head_camera_tilt_joint | HEAD_CAMERA_TILT_JOINT |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ポーズの設定方法
[sobit_edu_pose.yaml](sobit_edu_library/config/sobit_edu_pose.yaml)というファイルでポーズの追加・編集ができます．以下のようなフォーマットになります．

```yaml
sobit_pro_pose:
        - { 
        pose_name: "pose_name",
        arm_shoulder_1_tilt_joint: 1.57,
        arm_elbow_upper_1_tilt_joint: 1.57,
        arm_elbow_lower_tilt_joint: 0.0,
        arm_elbow_lower_pan_joint: -1.57,
        arm_wrist_tilt_joint: -1.57,
        hand_joint: 0.0,
        head_camera_pan_joint: 0.0,
        head_camera_tilt_joint: 0.0
    }
    ...
```  

### ホイルコントローラ
SOBIT PROの移動機構を動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 動作関数
1.  `controlWheelLinear()` : 並進（直進移動・斜め移動・横移動）を移動させます．
    ```cpp
    bool sobit::SobitProWheelController::controlWheelLinear (
        const double distance_x,            # x方向への直進移動距離 (m)
        const double distance_y,            # y方向への直進移動距離 (m)
    )
    ```  
2.  `controlWheelRotateRad()` : 回転運動を行う(弧度法：Radian)
    ```cpp
    bool sobit::SobitProWheelController::controlWheelRotateRad (
        const double angle_rad,             # 中心回転角度 (rad)
    )
    ```  
3.  controlWheelRotateDeg()   :   回転運動を行う(度数法：Degree)
    ```cpp
    bool sobit::SobitProWheelController::controlWheelRotateDeg ( 
        const double angle_deg,             # 中心回転角度 (deg)
    )
    ```

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ハードウェア
SOBIT EDUはオープンソースハードウェアとして[OnShape](https://cad.onshape.com/documents/4acbecde07fba120a62ec033/w/c6217b66947274dee4e8f911/e/c2e5c16292d7dfc11ee3cc01?renderMode=0&uiState=654a13b8711fc82bedc118e2)にて公開しております．

![SOBIT PRO in OnShape](sobit_pro/docs/img/sobit_pro_onshape.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<details>
<summary>ハードウェアの詳細についてはこちらを確認してください．</summary>

### パーツのダウンロード方法

1. Onshapeにアクセスしましょう．

    > [!NOTE]
    > ファイルをダウンロードするために，`OnShape`のアカウントを作成する必要がありません．ただし，本ドキュメント全体をコピする場合，アカウントの作成を推薦します．

1. `Instances`の中にパーツを右クリックで選択します．
1. 一覧が表示され，`Export`ブタンを押してください．
1. 表示されたウィンドウの中に，`Format`という項目があります．`STEP`を選択してください．
1. 最後に，青色の`Export`ボタンを押してダウンロードが開始されます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボットの組み立て
TBD

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボットの特徴
| 項目 | 詳細 |
| --- | --- |
| 最大直進速度 | 0.7[m/s] |
| 最大回転速度 | 0.229[rad/s] |
| 最大ペイロード | 0.35[kg] |
| サイズ (長さx幅x高さ) | 450x450x1250[mm] |
| 重量 | 16[kg] |
| リモートコントローラ | PS3/PS4 |
| LiDAR | UST-20LX |
| RGB-D | Azure Kinect DK (頭部)，RealSense D405 (アーム) |
| IMU | LSM6DSMUS |
| スピーカー | モノラルスピーカー |
| マイク | コンデンサーマイク |
| アクチュエータ (アーム) | 2 x XM540-W150, 6 x XM430-W320 |
| アクチュエータ (移動機構) | 4 x XM430-W320, 4 x XM430-W210 |
| 電源 | 2 x Makita 6.0Ah 18V |
| PC接続 | USB |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 部品リスト（BOM）

| 部品 | 型番 | 個数 | 購入先 |
| --- | --- | --- | --- |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [x] パラメタによるSOBIT EDUと移動機構のみの切り替え
- [ ] exampleファイルの修正
- [ ] OSS
    - [ ] ドキュメンテーションの充実
    - [ ] コーディングスタイルの統一

現時点のバッグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 変更履歴 -->
## 変更履歴

- 1.0: SOBIT PROと移動機構の設定パラメタ化 (2023-11-07)
  - 詳細 1
  - 詳細 2
  - 詳細 3

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- 参考文献 -->
## 参考文献

* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Noetic](http://wiki.ros.org/noetic)
* [ROS Control](http://wiki.ros.org/ros_control)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_pro/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_pro/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_pro/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_pro/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[license-url]: https://github.com/TeamSOBITS/sobit_pro/blob/master/LICENSE




<!-- 
 sobit_edu_library
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

--- -->