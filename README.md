# ROS Package for SOBIT EDU

SOBIT EDUを動かすために必要なリポジトリです．

※ 初めてロボットを動かす場合は，必ずロボットを動かしたことのある先輩方に付き添ってもらいながらロボットを動かしましょう．

## SOBIT EDU
![](sobit_education_bringup/img/sobit_edu.png)

## Prerequisites
以下の環境で動作します．
- OS: Ubuntu 18.04 
- ROS distribution: melodic Kame

### How to use
まず，以下のコマンドを入力して，SOBIT EDUを動かすための環境設定を行います．
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

※ 開発するPCで，SOBIT MINIやSOBIT PROを動かしたことがある場合も，この作業は必要ありません．

```bash:
$ cd sobit_education
$ bash sobit_setup.sh
```

以下のコマンドを入力することで，SOBIT EDUを起動することができます．
これにより，SOBIT EDUのモータやRGB-Dカメラ，測域センサ(Lidar)などのデバイスが起動します．
また，それと同時にRvizも起動します．

:warning: ロボットをコンテナで動かす場合，動かしたいデバイスをホストPCと接続してから，コンテナを立ち上げてください．
コンテナを立ち上げてからデバイスとの接続を行う場合，ロボットが動かない場合があります．

```bash:
$ roslaunch sobit_edu_bringup minimal.launch
```
