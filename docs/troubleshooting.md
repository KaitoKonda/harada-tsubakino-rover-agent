# 困ったときの確認事項

エラーが出たら、設定を次々に変えず、上から順番に確認します。作業中のログは
消さずにコピーして残してください。

## まず記録する情報

Raspberry Pi の端末で次を実行し、結果を保存します。

```bash
hostname -I
hostname -s
echo "$ROS_MASTER_URI"
echo "$ROS_HOSTNAME"
ls -l /dev/ttyACM*
arduino-cli board list
```

GUIを使っている場合、`ROS_HOSTNAME (auto)` が `<ホスト名>.local` になっているか確認します。

## GUIが開かない

端末から直接起動すると、原因が表示されます。

```bash
~/ROSGUILauncher/ROSGUILauncher.sh
```

- `No module named yaml`: `sudo apt install python3-yaml`
- `No module named PyQt5`: `sudo apt install python3-pyqt5`
- ファイルがない: `cd ~/harada-tsubakino-rover-agent && python3 update.py -g`
- `yaml.scanner.ScannerError`: 次を実行すると、壊れた設定をバックアップして
  書式を自動修復します。

  ```bash
  cd ~/harada-tsubakino-rover-agent
  python3 update.py -g
  ```

## `CONTROL_PC_IP` という文字がログに出る

仮の文字列が設定に残っています。[セットアップ手順](setup.md#8-ネットワーク設定を書く)
に戻り、`~/ROSGUILauncher/launcherConfig.yaml` を制御用PCの実際のIPへ書き換えます。

## ROSマスターへ接続できない

よくある表示は `Unable to communicate with master` です。

1. 制御用PCで `rosinit` が完了しているか確認する
2. PCとローバが同じルータにいるか確認する
3. ローバから `ping -c 4 制御用PCのIP` を実行する
4. GUI の `ROS_MASTER_URI` が `http://制御用PCのIP:11311` か確認する
5. PCのファイアウォールが ROS の通信を遮断していないか確認する

PCに有線LANとWi-Fiがある場合、ROSが別のネットワーク側IPを広告していることが
あります。MATLABの `rosinit` が示すIPと、ローバから到達できるIPを揃えます。

## ROS_MASTER_URIがlocalhostになっている

`localhost` は、その設定を使っているコンピュータ自身を表します。ローバ側で
`http://localhost:11311` を指定すると、制御用PCではなくローバ自身を参照します。

GUIの `ROS_MASTER_URI` または次の設定ファイルを確認します。

```bash
nano ~/ROSGUILauncher/launcherConfig.yaml
```

次のようになっている場合は、`CONTROL_PC_IP` を制御用PCの実際のIPへ置き換えます。

```yaml
ros_master_uri: http://CONTROL_PC_IP:11311
```

`localhost` や `127.0.0.1` はローバ側では使いません。保存後にGUIの `Load Config` を
押すか、GUIを起動し直してから `Launch` を押します。

なお、制御用PC自身で動くMATLABでは `localhost` が制御用PCを指すため、同じ表記が
正しく使える場合があります。

## rover_driveを送ってもモーターが動かない

`pos_controller.py` は指令値だけでなく、車輪エンコーダのオドメトリを受信したときに
モーター出力を更新します。次を確認します。

```bash
rostopic hz /pi1/odom
rostopic info /pi1/odom
```

ホスト名が `pi3` なら `/pi3/odom` と読み替えます。更新されていない場合は
`rover_odometry`、`rover_i2c_controller`、I2C接続のエラーをGUIログで確認します。

## 制御用PCからローバ名を解決できない

ホスト名が `pi3` の例では、制御用PCから次を実行します。

```text
ping pi3.local
```

通らない場合は、ローバで Avahi の状態を確認します。

```bash
sudo systemctl status avahi-daemon
```

`active (running)` でなければ、同じルータへの接続、ホスト名の重複、Avahiの導入、
PCのファイアウォールを確認します。名前解決できるまではROSノードを起動しません。

## `Resource not found: harada-tsubakino`

ROS パッケージが未配置か、環境を読み込んでいません。

```bash
cd ~/harada-tsubakino-rover-agent
python3 update.py -r
source ~/catkin_ws/devel/setup.bash
rospack find harada-tsubakino
```

最後にパッケージのパスが出れば直っています。

## `/dev/ttyACM0` を開けない

### `Permission denied`

```bash
groups
```

一覧に `dialout` がなければ実行し、再起動します。

```bash
sudo usermod -aG dialout "$USER"
sudo reboot
```

全ユーザーへ `0666` 権限を与えるルールは追加しないでください。

### `No such file or directory`

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
arduino-cli board list
```

USBケーブルを挿し直して再確認します。別名になった場合は
`harada-tsubakino/launch/harada-tsubakino.launch` の `port` を合わせます。

## Arduinoへの書き込みに失敗する

まず接続先を確認します。

```bash
arduino-cli board list
```

ポートを明示して試す場合、`PORT` を表示された値へ置き換えます。

```bash
arduino-cli compile --upload --port PORT \
  --fqbn arduino:esp32:nano_nora \
  ~/harada-tsubakino-rover-agent/OtosHmcSerialSender
```

ライブラリ不足と表示された場合は [初回セットアップ手順](setup.md#5-arduino-cliを入れる)
のライブラリ導入をやり直します。

## トピックはあるが値が来ない

```bash
rostopic info /pi1/otos_pose
rostopic hz /pi1/otos_pose
```

- Publisher が 0: ローバ側ノードが起動していません
- Publisher はあるが 0 Hz: Arduino、USB、センサー初期化を確認します
- GUIログに `Unexpected serial line`: Arduino と ROS 側の通信形式が一致していません
- `OTOS` または `HMC` の初期化失敗が出る: 配線を触らず、ハードウェア担当者へ確認します

## ローバでは見えるがMATLABでは見えない

ローバ自身で `rostopic list` が見えるだけでは、PCまで通信できているとは限りません。

- PCとローバのIPが同じネットワーク帯か
- `ROS_MASTER_URI` が制御用PCを指しているか
- `ROS_HOSTNAME` が `<ホスト名>.local` になっているか
- 制御用PCから `ping <ホスト名>.local` が通るか
- PCのファイアウォール設定
- PCに複数のネットワーク接続がある場合の経路

を確認します。まず PC とローバの双方向 `ping` が通る状態を作ります。

## 更新できない／手元の変更がある

`git pull --ff-only` が停止した場合、次で変更内容だけを確認します。

```bash
cd ~/harada-tsubakino-rover-agent
git status
git diff
```

`git reset --hard` は手元の変更を消すため、内容を確認せず実行しないでください。
判断できない場合は、上の2つの結果を添えて管理者へ相談します。

## 相談時に渡すもの

- 何をしようとしたか
- 実行した命令
- エラー全文（写真よりコピーした文字が望ましい）
- 制御用PCのIPアドレスとローバのホスト名
- `git status` の結果
- GUIログの起動直後からエラーまで
