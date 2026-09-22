# harada-tsubakino-rover-agent

VSTONE ライトローバー上で ROS 1 の走行ノードを起動し、追加センサーの値を
Arduino Nano ESP32 から ROS に渡すための、ローバ側リポジトリです。

初めて触る場合は、次の順番で読んでください。

1. [セットアップ手順](docs/setup.md) — 初回だけ行う作業
2. [起動・終了と動作確認](docs/usage.md) — 実験のたびに行う作業
3. [困ったときの確認事項](docs/troubleshooting.md) — エラーの切り分け

## このリポジトリが担当する範囲

データの流れは次のとおりです。

```text
OTOS / HMC6343
      ↓
Arduino Nano ESP32
      ↓ USBシリアル
Raspberry Pi（このリポジトリ）
      ↓ ROS 1 / ネットワーク
制御用PC（MATLAB・ROSマスター）
```

主な構成要素は以下です。

- `harada-tsubakino/`: ライトローバーとセンサーを起動する ROS パッケージ
- `OtosHmcSerialSender/`: Nano ESP32 に書き込むスケッチ
- `ROSGUILauncher.py`: ROS の接続先を設定して起動する画面
- `update.py`: ROS パッケージ、Arduino スケッチ、GUI を更新する補助ツール

## 前提

Raspberry Pi OS、ROS Melodic、ライトローバー標準ソフトウェアは、VSTONE の
公式資料に従って導入済みであることを前提にします。

- [ライトローバーWebDoc](https://vstoneofficial.github.io/lightrover_webdoc/)
- [ソフトウェアセットアップ（Raspberry Pi OS / ROS 1）](https://vstoneofficial.github.io/lightrover_webdoc/setup/softwareSetup/)

参照日: 2026-09-23

## 現在の制約

- センサーやモーターの配線、取付位置、電源構成は未記載です。既存の実機を
  そのまま使う前提で、むやみに配線を変更しないでください。
- この手順の最小試験はローバ1台を対象にします。ROSトピック名とTFフレーム名は
  ホスト名ごとに分離されますが、複数台同時の実機試験はまだ行っていません。
- 制御用PCのIPアドレスは環境ごとに異なります。設定ファイルの `CONTROL_PC_IP` は、
  実際の値に置き換えるまで使えません。ローバ自身は `<ホスト名>.local` を自動使用します。

## ROSで見える主な入出力

ROS名前空間 `group` には、Raspberry Pi の短いホスト名が自動的に使われます。
ホスト名が `pi1` の場合は次の名前になります。

| 名前 | 型 | 内容 |
| --- | --- | --- |
| `/pi1/rover_drive` | `geometry_msgs/Twist` | ライトローバーへの速度指令 |
| `/pi1/odom` | `nav_msgs/Odometry` | 車輪エンコーダによる速度フィードバック |
| `/pi1/otos_pose` | `geometry_msgs/Pose2D` | OTOS の平面位置・方位 |
| `/pi1/hmc6343_rpy` | `geometry_msgs/Vector3Stamped` | HMC6343 の姿勢角 |
| `/pi1/hmc6343_accel` | `geometry_msgs/Vector3Stamped` | HMC6343 の加速度 |
| `/tf` | `tf/tfMessage` | OTOS専用の `pi1/otos_odom` から `pi1/otos_base_link` への姿勢変換 |

OTOS の距離は m、角度は rad、加速度は m/s² です。
`/pi1/odom`は車輪エンコーダ由来、`/pi1/otos_pose`と上記TFは搭載OTOS由来であり、
同じオドメトリとして混同しないでください。
