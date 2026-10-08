# 初回セットアップ手順

この文書は、Linux や ROS に慣れていない人が、既に組み立て済みのライトローバーを
起動できる状態にするための手順です。ハードウェアの配線や取付は扱いません。

## 0. 作業前に確認すること

次のものを用意します。

- VSTONE 公式手順でセットアップ済みの Raspberry Pi
- Raspberry Pi に接続する画面・キーボード、または SSH / VNC 接続
- インターネット接続（初回インストール時のみ）
- 制御用PCとローバを同じネットワークに接続できるルータ
- 制御用PCのIPアドレス

この文書のコード枠にある命令は、Raspberry Pi の「端末」に1行ずつ入力します。
行頭の `$` は説明用なので入力しません。

Raspberry Pi のホスト名は、ローバを識別する ROS 名前空間にも使われます。
例えばホスト名が `pi1` なら、センサートピックは `/pi1/odometry/otos` になります。
OSのログインユーザー名は全ローバで `pi` に統一して構いません。

## 1. VSTONE公式セットアップを終える

[VSTONEのソフトウェアセットアップ](https://vstoneofficial.github.io/lightrover_webdoc/setup/softwareSetup/)
に従い、少なくとも次を完了させます。

- Raspberry Pi OS と I2C の設定
- ROS Melodic の導入
- `~/catkin_ws` の作成
- `catkin build` が使える状態
- `lightrover_ros` パッケージの導入

### Buster の Raspbian 取得先を切り替える

Raspberry Pi OS (Legacy) の Buster 版を使っている場合は、VSTONE 公式手順で
パッケージをインストールする前に Raspbian の取得先を切り替えます。まず OS を確認します。

```bash
cat /etc/os-release
```

`VERSION_CODENAME=buster` と表示される場合だけ、次を実行します。
Buster 以外の OS では、この切り替えは行いません。

```bash
sudo apt edit-sources
```

開いたファイルの1行目にある Raspbian の取得先を、次の1行に書き換えて保存します。
ほかの配布元の行は変更しません。

```text
deb https://legacy.raspbian.org/raspbian/ buster main contrib non-free rpi
```

パッケージ一覧を更新してから、VSTONE 公式手順に戻ります。

```bash
sudo apt-get update
```

### rosdep の参照先を固定する

VSTONE 公式手順の ROS Melodic 導入中、`sudo rosdep init` の直後、
`rosdep update` の前に rosdep の参照先を固定します。現在の参照先のままだと、
公式手順で必要な Python モジュールの依存関係が適切に解決されず、
ROS のビルドでエラーになる場合があります。2022 年 1 月 1 日より前の
最後の [`ros/rosdistro` コミット](https://github.com/ros/rosdistro/commit/d573eab3a166e3aa3642b82260108608e5f468b4)
（`d573eab3a166e3aa3642b82260108608e5f468b4`）を使います。

```bash
sudo sed -i 's@https://raw.githubusercontent.com/ros/rosdistro/master/@https://raw.githubusercontent.com/ros/rosdistro/d573eab3a166e3aa3642b82260108608e5f468b4/@g' /etc/ros/rosdep/sources.list.d/20-default.list
grep rosdistro /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update
```

`grep` の結果で、`20-default.list` 内の `rosdistro` の URL がすべて上記の
コミット ID を含むことを確認してから、公式手順の `rosdep install` 以降に戻ります。

このリポジトリは上記を置き換えません。古いメモを見て OS の配布元や
APT の取得先を一括置換しないでください。

## 2. 必要なソフトウェアを入れる

ローバ用のソフトウェアを入れます。

```bash
sudo apt install -y git curl python-serial python3-yaml python3-pyqt5 avahi-daemon
```

途中でパスワードを求められたら、Raspberry Pi のログインパスワードを入力します。
入力中は文字が表示されませんが正常です。

## 3. このリポジトリを取得する

```bash
cd ~
git clone https://github.com/KaitoKonda/harada-tsubakino-rover-agent.git
cd ~/harada-tsubakino-rover-agent
```

既にフォルダがある場合は `git clone` を繰り返さず、次で更新します。

```bash
cd ~/harada-tsubakino-rover-agent
git pull --ff-only
```

手元の変更があると `git pull --ff-only` は停止します。その場合は変更を消さず、
[トラブル対応](troubleshooting.md)を確認してください。

## 4. ROSパッケージと起動GUIを配置する

```bash
cd ~/harada-tsubakino-rover-agent
python3 update.py -r -g
```

この命令は次を行います。

- ROS パッケージを `~/catkin_ws/src/harada-tsubakino` にコピーしてビルド
- GUI を `~/ROSGUILauncher` にコピー
- デスクトップに `rosGuiLauncher.desktop` を配置

2回目以降の更新では、入力済みの `~/ROSGUILauncher/launcherConfig.yaml` は
正常な場合は上書きしません。YAMLの書式が壊れている場合だけ、制御用PCのIPを
可能な限り引き継いで自動修復します。修復前のファイルは
`launcherConfig.yaml.invalid-日時` という名前で同じフォルダに残ります。

最後に赤い `Error` が出なければ次へ進みます。

### 既存ローバを最新版へ更新する

既にセットアップ済みのローバでは、GUIの `Stop` を押して終了してから、次の順番で
更新します。

```bash
cd ~/harada-tsubakino-rover-agent
git status --short
```

`git status --short` が何も表示しないことを確認します。ファイル名が表示された場合は、
ローバ上に未保存の変更があります。そこで止めて、変更を消す操作はせず、
[トラブル対応](troubleshooting.md#更新できない手元の変更がある)を確認してください。

何も表示されなかった場合だけ、続けて次を実行します。

```bash
git pull --ff-only
python3 update.py -r -g
```

`-r` はROSパッケージの配置とビルド、`-g` は起動GUIとデスクトップアイコンの更新を
行います。正常な `~/ROSGUILauncher/launcherConfig.yaml` と、そこに設定した制御用PCの
IPアドレスは保持されます。設定ファイルのYAML書式が壊れている場合は自動修復され、
修復前の内容は `launcherConfig.yaml.invalid-日時` として保存されます。

更新後、デスクトップの `ROS GUI Launcher` を起動します。GUIだけを更新する場合は
`python3 update.py -g`、ROSパッケージだけを更新する場合は `python3 update.py -r` を
使用できます。オプションを付けずに `python3 update.py` を実行するとArduinoへの
書き込みも含む全処理が走るため、通常のソフトウェア更新では使用しません。

## 5. Arduino CLIを入れる

Arduino Nano ESP32 への書き込みに使います。

```bash
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
arduino-cli version
```

最後の行でバージョン番号が表示されれば導入成功です。

続けて、Nano ESP32 の定義とセンサーライブラリを入れます。

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install arduino:esp32
arduino-cli config set library.enable_unsafe_install true
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_Toolkit.git
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library.git
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_HMC6343_Arduino_Library.git
```

`unsafe` という語は、任意の Git リポジトリからライブラリを入れる機能を許可する
という意味です。上の3つは SparkFun 公式リポジトリです。
OTOS ライブラリが必要とする `SparkFun_Toolkit.h` は、先に入れた
`SparkFun Toolkit` に含まれます。Git URL からのライブラリ導入では依存先が
自動で入らないため、Toolkit の行を省略しないでください。

## 6. USBシリアルとDFU書き込みの権限を設定する

```bash
sudo usermod -aG dialout "$USER"
```

`dialout` はシリアルポートの権限です。Nano ESP32 の DFU 書き込みでは別の
USB デバイスを使うため、次のルールも作成します。

```bash
sudo nano /etc/udev/rules.d/70-arduino-nano-esp32-dfu.rules
```

ファイルに次の1行を書いて保存します。Arduino Nano ESP32 の DFU デバイス
`2341:0070` だけを `dialout` グループに許可します。

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2341", ATTR{idProduct}=="0070", GROUP="dialout", MODE="0660"
```

ルールを読み直して再起動します。再起動後に Nano ESP32 を USB で接続すれば、
新しいルールが適用されます。

```bash
sudo udevadm control --reload-rules
sudo reboot
```

再起動後、`id -nG` に `dialout` が含まれることと、接続先を確認します。

```bash
id -nG
arduino-cli board list
```

`/dev/ttyACM0` と `Arduino Nano ESP32` が表示されるのが目安です。表示されなければ
[トラブル対応](troubleshooting.md)へ進みます。

## 7. Nano ESP32へスケッチを書き込む

センサーが既に正しく接続された実機でだけ行います。

```bash
cd ~/harada-tsubakino-rover-agent
python3 update.py -a
```

`Upload` の完了が表示されれば成功です。接続先が `/dev/ttyACM0` 以外の場合は、
次のように実際のポートを一時指定します。

```bash
ARDUINO_PORT=/dev/ttyACM1 python3 update.py -a
```

`/dev/ttyACM1` は `arduino-cli board list` に表示された値へ置き換えてください。

## 8. ネットワーク設定を書く

設定ファイルを開きます。

```bash
nano ~/ROSGUILauncher/launcherConfig.yaml
```

制御用PCのIPだけを実際の値へ置き換えます。`ros_hostname: auto` は変更しません。

```yaml
ros_master_uri: http://CONTROL_PC_IP:11311
ros_hostname: auto
```

例として制御用PCが `192.168.11.53` なら次の形です。

```yaml
ros_master_uri: http://192.168.11.53:11311
ros_hostname: auto
```

この例の値をそのまま使わず、必ず現在のネットワークで確認してください。
保存は `Ctrl+O`、Enter、終了は `Ctrl+X` です。

ローバ名に相当する `group` は、起動時にホスト名から自動設定されます。
設定ファイルをローバごとに編集する必要はありません。次の命令で確認できます。

```bash
hostname -s
```

`pi1`、`pi3` のように英字で始まり、英小文字と数字だけを使った、ローバごとに
重複しないホスト名を使用してください。ハイフンやピリオドは ROS 名前空間に
使わないでください。

ホスト名を変更する場合は、`NEW_NAME` を割り当てる名前へ置き換えて実行します。

```bash
sudo hostnamectl set-hostname NEW_NAME
sudo reboot
```

再起動後、`hostname -s` と、制御用PCからの `ping NEW_NAME.local` の両方を確認します。
GUIは短いホスト名が `pi3` なら `ROS_HOSTNAME=pi3.local` を自動設定し、古い
`ROS_IP` 設定は使用しません。

## 9. 初回セットアップの完了確認

次がすべて満たされればセットアップ完了です。

- `~/catkin_ws/src/harada-tsubakino` がある
- `~/ROSGUILauncher/rosGuiLauncher.py` がある
- デスクトップに ROS GUI Launcher がある
- `arduino-cli board list` で Nano ESP32 が見える
- `launcherConfig.yaml` に制御用PCのIPアドレスを書いた
- `hostname -s` で、そのローバに割り当てた `pi1`、`pi3` などが表示される
- 制御用PCから `ping <ホスト名>.local` が通る

続いて [起動・終了と動作確認](usage.md) を実施します。

## 参考にした公式資料

- [Arduino CLIのインストール](https://arduino.github.io/arduino-cli/latest/installation/)
- [Arduino CLIの設定](https://arduino.github.io/arduino-cli/latest/configuration/)
- [SparkFun Qwiic OTOS Arduino Library](https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library)
- [SparkFun HMC6343 Arduino Library](https://github.com/sparkfun/SparkFun_HMC6343_Arduino_Library)
