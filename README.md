# beego_control
ビーゴ（２輪ロボット）の制御ノードが記述されているパッケージ

## beego_control_driver
ロボット用のドライバー。命令速度を受け取り, エンコーダ値を配信する。

usage
```
sudo chmod 777 /dev/ttyUSB0
roslaunch beego_control beego_control.launch
```
## beego_control_tf
ロボットのホイールオドメトリをtf_broadcastで配信するノード。

usage
```
rosrun beego_control beego_control_tf
```
## beego_control_teleop
ロボット遠隔操作用のノード。
rqt_reconfigureで速度命令、角速度命令を配信する。

usage
```
rosrun beego_control beego_control_teleop
```
