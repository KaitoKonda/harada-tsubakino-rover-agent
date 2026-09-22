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
例えばホスト名が `pi1` なら、センサートピックは `/pi1/otos_pose` になります。
OSのログインユーザー名は全ローバで `pi` に統一して構いません。

## 1. VSTONE公式セットアップを終える

[VSTONEのソフトウェアセットアップ](https://vstoneofficial.github.io/lightrover_webdoc/setup/softwareSetup/)
に従い、少なくとも次を完了させます。

- Raspberry Pi OS と I2C の設定
- ROS Melodic の導入
- `~/catkin_ws` の作成
- `catkin build` が使える状態
- `lightrover_ros` パッケージの導入

このリポジトリは上記を置き換えません。古いメモを見て OS の配布元や
APT の取得先を一括置換しないでください。

## 2. 必要なソフトウェアを入れる

端末で実行します。

```bash
sudo apt update
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
- デスクトップに `ROSGUILauncher.desktop` を配置

2回目以降の更新では、入力済みの `~/ROSGUILauncher/launcherConfig.yaml` は
上書きしません。

最後に赤い `Error` が出なければ次へ進みます。

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
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library.git
arduino-cli lib install --git-url https://github.com/sparkfun/SparkFun_HMC6343_Arduino_Library.git
```

`unsafe` という語は、任意の Git リポジトリからライブラリを入れる機能を許可する
という意味です。上の2つは SparkFun 公式リポジトリです。

## 6. USBシリアルを使う権限を設定する

```bash
sudo usermod -aG dialout "$USER"
sudo reboot
```

再起動したら、Nano ESP32 を USB で Raspberry Pi に接続します。接続先を確認します。

```bash
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
- `~/ROSGUILauncher/ROSGUILauncher.py` がある
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
