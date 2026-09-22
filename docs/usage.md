# 起動・終了と動作確認

この文書は、初回セットアップ後に毎回行う操作を説明します。最初は必ずローバ1台、
車輪を床から浮かせた状態で確認してください。

## 起動前の安全確認

- 非常時にすぐ電源を切れる位置にいる
- 車輪の近くに手、服、ケーブルを置かない
- センサーと Arduino の配線を変更していない
- Nano ESP32 と Raspberry Pi が USB 接続されている
- 制御用PCとローバが同じルータに接続されている

## 1. 制御用PCでROSマスターを起動する

MATLAB の ROS Toolbox を使う現在の構成では、制御用PCの MATLAB で ROS マスターを
起動します。

```matlab
rosinit
```

既に起動済みと表示される場合は、そのままで構いません。表示された ROS Master URI の
IPアドレスが `launcherConfig.yaml` の `ros_master_uri` と一致することを確認します。

## 2. PCとローバが互いに見えるか確認する

Raspberry Pi の端末で、`CONTROL_PC_IP` を実際のIPへ置き換えて実行します。

```bash
ping -c 4 CONTROL_PC_IP
```

`0% packet loss` が目安です。応答しなければ ROS を起動する前にネットワークを直します。

## 3. ローバ側ROSノードを起動する

Raspberry Pi のデスクトップにある `ROSGUILauncher.desktop` をダブルクリックします。
確認画面が出た場合は「実行」を選びます。

画面の各欄は次の意味です。

| 欄 | 入れるもの |
| --- | --- |
| `ROS_MASTER_URI` | `http://制御用PCのIP:11311` |
| `ROS_HOSTNAME (auto)` | `<ホスト名>.local`（自動入力・編集不要） |
| `Package` | `harada-tsubakino` |
| `Launch File` | `harada-tsubakino.launch` |
| `Args` | `group:=ホスト名`（自動入力） |
| `Extra Python Script` | 通常は空欄 |

値を直した場合は `Save Config`、起動するときは `Launch` を押します。

起動時にセンサーブリッジが Arduino へ `RESET` を送り、続いて計測を始めます。
この間はローバを数秒間動かさないでください。ログに次が現れるのが目安です。

```text
Opened serial port /dev/ttyACM0 at 115200 baud
```

### GUIが開かない場合の手動起動

Raspberry Pi の端末で次を実行します。

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://CONTROL_PC_IP:11311
unset ROS_IP
export ROS_HOSTNAME="$(hostname -s).local"
roslaunch harada-tsubakino harada-tsubakino.launch group:=$(hostname -s)
```

`CONTROL_PC_IP` は実際の制御用PCのIPへ置き換えます。この端末は実験終了まで閉じません。
手動起動でも、ホスト名が ROS 名前空間になります。

## 4. センサートピックを確認する

別の端末を開き、環境を読み込みます。

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://CONTROL_PC_IP:11311
unset ROS_IP
export ROS_HOSTNAME="$(hostname -s).local"
rostopic list
```

少なくとも次が表示されることを確認します。

以下はホスト名が `pi1` の場合です。`pi3` なら `/pi3/...` と読み替えます。

```text
/pi1/otos_pose
/pi1/hmc6343_rpy
/pi1/hmc6343_accel
/pi1/rover_drive
/pi1/odom
```

搭載OTOSのTFフレームもローバごとに分離され、`pi1/otos_odom` から
`pi1/otos_base_link` のような名前になります。

`/pi1/odom` は車輪エンコーダから計算したオドメトリで、ライトローバーの
`pos_controller.py` が現在速度のフィードバックとして使います。OTOS由来のTFとは
別物です。

値が更新されるか確認します。停止は `Ctrl+C` です。

```bash
rostopic echo /pi1/otos_pose
```

更新周期も確認できます。

```bash
rostopic hz /pi1/otos_pose
rostopic hz /pi1/hmc6343_rpy
```

目安は OTOS が約 50 Hz、HMC6343 が約 5 Hz です。

## 5. MATLABから見えるか確認する

制御用PCの MATLAB で実行します。

```matlab
rostopic list
otosSub = rossubscriber('/pi1/otos_pose');
otosMsg = receive(otosSub, 3)
```

3秒以内にメッセージが表示されれば、ローバから制御用PCまでの通信はできています。

## 6. 最小接続試験の完了条件

ここまでで、次を満たせばソフトウェアとネットワークの最小接続試験は完了です。

- GUIに致命的なエラーが出ていない
- Raspberry Pi で5つの主要トピックが見える
- `/pi1/otos_pose` と `/pi1/hmc6343_rpy` が更新される
- MATLABの `receive` で OTOS のメッセージを受信できる

モーターを実際に回す試験は、配線、車輪方向、緊急停止方法を記載した
ハードウェア手順が整ってから行います。この文書だけを見て走行指令を送らないでください。

## 終了手順

1. 走行指令を送っている MATLAB スクリプトを停止する
2. GUI の `Stop` を押す（手動起動なら端末で `Ctrl+C`）
3. 制御用PCの MATLAB で `rosshutdown` を実行する
4. Arduino と Raspberry Pi の通信が止まったことを確認する
5. Raspberry Pi を通常の手順でシャットダウンしてから電源を切る

電源断だけで終了すると、Raspberry Pi のファイルを壊すことがあります。
