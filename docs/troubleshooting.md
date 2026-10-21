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
~/ROSGUILauncher/rosGuiLauncher.sh
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
rostopic hz /pi1/odometry/wheel
rostopic info /pi1/odometry/wheel
```

ホスト名が `pi3` なら `/pi3/odometry/wheel` と読み替えます。更新されていない場合は
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

## `sudo: ホスト ... の名前解決ができません`

USBシリアルの権限設定中に出ても、原因はホスト名の設定です。現在の名前と
設定ファイルを確認します。

```bash
hostname
cat /etc/hostname
cat /etc/hosts
```

例えば `hostname` が `pi2` なら、`sudo nano /etc/hosts` で既存の
`127.0.1.1` の行を `127.0.1.1 pi2` に直します。`127.0.0.1 localhost` は
残してください。`/etc/hostname` が異なる名前なら、`sudo nano /etc/hostname` で
こちらも `pi2` に合わせます。
実際のホスト名が `pi2` 以外なら、その名前に読み替えます。

```bash
getent hosts "$(hostname)"
```

アドレスが表示されたら、権限設定を再実行して再起動します。

```bash
sudo usermod -aG dialout "$USER"
sudo reboot
```

警告が出たときも `usermod` 自体は成功していた可能性がありますが、再実行できます。

## Arduinoへの書き込みに失敗する

まず接続先を確認します。

```bash
arduino-cli board list
```

ポートを明示して試す場合、`PORT` を表示された値へ置き換えます。

```bash
arduino-cli compile --upload --port PORT \
  --fqbn arduino:esp32:nano_nora \
  ~/harada-tsubakino-rover-agent/otosHmcSerialSender
```

ライブラリ不足と表示された場合は [初回セットアップ手順](setup.md#5-arduino-cliを入れる)
のライブラリ導入をやり直します。
`SparkFun_Toolkit.h` が見つからない場合は、次を実行してから書き込みを再試行します。

```bash
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_Toolkit.git
```

### `dfu-util: Cannot open DFU device 2341:0070 ... (LIBUSB_ERROR_ACCESS)`

続けて `No DFU capable USB device available` と出る場合も、まず DFU 用 USB
デバイスの権限を確認します。`arduino-cli board list` でシリアルポートが見えていても、
DFU 書き込みは別の `/dev/bus/usb/` デバイスを使います。ホスト名の変更とは
別の問題です。

```bash
id -nG
lsusb -d 2341:0070
```

`id -nG` に `dialout` があることを確認します。なければ上の
[`Permission denied`](#permission-denied) の手順を実行します。
`lsusb` に `Bus 001 Device 011` と表示された場合、対応するデバイスは
`/dev/bus/usb/001/011` です。番号は実際の表示に読み替えてください。

```bash
ls -l /dev/bus/usb/001/011
```

所有グループが `root` のままなら、次のファイルを作成します。

```bash
sudo nano /etc/udev/rules.d/70-arduino-nano-esp32-dfu.rules
```

内容は次の1行だけにします。

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2341", ATTR{idProduct}=="0070", GROUP="dialout", MODE="0660"
```

保存したらルールを読み直します。

```bash
sudo udevadm control --reload-rules
```

Nano ESP32 の USB ケーブルを抜き差ししてから、新しいデバイス番号を確認します。

```bash
lsusb -d 2341:0070
```

`Device` 番号は抜き差し後に変わるため、新しい番号で `/dev/bus/usb/` の権限を
確認します。`crw-rw---- 1 root dialout` と表示されたら書き込みを再試行します。

```bash
cd ~/harada-tsubakino-rover-agent
python3 update.py -a
```

参考: [Arduino の DFU エラーに関する説明](https://support.arduino.cc/hc/en-us/articles/11011849739804-dfu-util-errors-when-uploading-exit-status-74)

## トピックはあるが値が来ない

GUIのLaunchログで、まずセンサーブリッジの表示を確認します。
ブリッジの出力は `output="screen"` と非バッファ出力でGUIへ渡します。

| ログの目印 | 意味 |
| --- | --- |
| `Opened serial port` | ポートを開けた。センサー受信の確認とは別 |
| `Serial sensor stream healthy` | 有効なOTOS・HMCデータを受信。途絶後の復帰時にも表示 |
| `Serial receive timeout` | シリアル受信バイトが途絶。ポート名・経過秒数・受信件数・直近STATUSを併記 |
| `OTOS receive timeout` / `HMC receive timeout` | バイトは来ているが、該当センサーの有効データが途絶 |
| `Arduino sensor initialization error` | `OTOS_INIT_FAILED` / `HMC_INIT_FAILED` / `NO_SENSORS_READY` |
| `Arduino streaming warning` | `BRIDGE_TIMEOUT` / `STREAM_OFF` / `UNKNOWN_COMMAND` |
| `Arduino error` | Arduinoからの `ERROR,...` 行 |
| `Failed to parse line` / `Unexpected serial line` / `Discarding overlong serial line` | 不正な値・形式・長すぎる行 |
| `Serial read failed` / `Serial write failed` | USBシリアルの読み書き例外。書き込みエラーは送信コマンドも表示 |

監視はシリアルの読み書きと別スレッドで行い、ROS時刻とは独立した単調時計を使います。
既定ではポートを開けてから5秒間の起動猶予を設け、以降は1秒間の無受信を警告します。
起動猶予中でも一度正常な両センサー受信を確認した後は、1秒の監視へ移ります。
同じ受信警告は5秒ごとに繰り返し、復帰も表示します。
STATUSや不正な行だけが届いても、センサーの正常受信とは扱いません。
NaN・無限大を含むセンサー行は警告して破棄し、ROSへ配信しません。
ローバーが静止していて位置が変わらないこと自体は異常と判定しません。

受信間隔はブリッジのprivateパラメーター `receive_timeout`（既定1秒）、
`receive_startup_grace`（5秒）、`receive_warning_interval`（5秒）で起動時に設定できます。
今回の監視は状態をログに出す処理です。受信停止時の自動リセット・自動再接続・追加RESETは行いません。

control-stationが接続待ちで繰り返しタイムアウトし、`Serial receive timeout` も出る場合は、
PCとのROS接続だけでなく、Arduino・USB・ブリッジの入出力停止も確認します。
Arduinoのリセットで復帰させる場合は実験・Launchを停止し、リセット後にLaunchを起動し直します。
再起動したArduinoは配信停止状態で始まるため、ブリッジからのSTARTが必要です。

```bash
rostopic info /pi1/odometry/otos
rostopic hz /pi1/odometry/otos
```

- Publisher が 0: ローバ側ノードが起動していません
- Publisher はあるが 0 Hz: Arduino、USB、センサー初期化を確認します
- GUIログに `Unexpected serial line`: 受信行の項目数や形式が不正です。
  例えば `OTOS,3408057,0.345459,-6.8` は方位まで含む5項目が揃っていません。
  旧版のブリッジは読み取りタイムアウトで返った行の断片も解析していたため、
  通信形式の不一致に限らず、受信途中の分断でもこの警告が出ます。
  現行版は改行まで断片を保持し、256バイトを超える行は次の改行まで破棄します。
- `OTOS` または `HMC` の初期化失敗が出る: 配線を触らず、ハードウェア担当者へ確認します

この警告自体は走行ノードを停止しません。ただし、制御用PCがOTOSを状態入力に
使っている場合、有効な計測が設定された有効期限を超えて途絶えると、PC側が
`odometryLost` としてゼロ指令を送って終了します。警告1件だけで停止原因と
決めず、同じ時刻の `rostopic hz /pi1/odometry/otos` とPCの実験ログを確認してください。
`STATUS,BRIDGE_TIMEOUT` が出た場合は、ArduinoがPINGを1秒以上受信できず
センサー配信を停止しています。USB通信とブリッジのログも確認します。

修正版をPi側へ配置する際は、実験とGUIのLaunchを停止してから、更新済みの
リポジトリで `python3 update.py -r` を実行し、GUIでLaunchし直します。
この受信処理の修正だけならArduinoスケッチの再書き込み・GUIアプリ自体の再配置は不要です。

ROSや実機なしで受信処理を検証するには、リポジトリ直下で次を実行します。

```bash
python3 -m unittest discover -s tests -p 'test_serial_sensor_bridge.py' -v
```

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

`git pull --ff-only` が停止した場合、まず次で変更内容を確認します。

```bash
cd ~/harada-tsubakino-rover-agent
git status
git diff
```

`old mode 100644` / `new mode 100755` だけが表示される場合は、実行権限の差分です。
旧版の `update.py` は更新用 `.sh` に実行権限を付けていたため、更新するたびに
この差分が発生していました。現在は `bash` で直接実行し、リポジトリ内の権限を変更しません。
ROSノードやGUIの配置先には、引き続き必要な実行権限を付けます。

ローバ上のこのリポジトリだけで権限差分を無視するには、次を一度実行します。
ファイル内容やステージ済みの変更は引き続き検出されます。

```bash
git config --local core.filemode false
git status --short
```

何も表示されなくなった場合は、`git pull --ff-only` を再実行できます。
まだ変更が表示される場合は `git diff` と `git diff --cached` で内容を確認します。
`--ff-only` は履歴が分岐している場合にも停止するため、その場合は権限設定では解消しません。

`git reset --hard` は手元の変更を消すため、内容を確認せず実行しないでください。
判断できない場合は、エラー全文と上の確認結果を添えて管理者へ相談します。

## 相談時に渡すもの

- 何をしようとしたか
- 実行した命令
- エラー全文（写真よりコピーした文字が望ましい）
- 制御用PCのIPアドレスとローバのホスト名
- `git status` の結果
- GUIログの起動直後からエラーまで
