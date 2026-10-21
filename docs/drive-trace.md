# ローバー内部の走行ログ

既存の走行処理に計測を追加したノードを、このリポジトリから配置する。
メーカーの `lightrover_ros` 内のファイルは編集しない。追加のI2C読み取り、制御式・
ゲイン変更、指令タイムアウトの追加は行わない。既存のPython 2 / ROS Melodicを対象とする。

## 配置・起動

この機能は実機検証用の `feature/drive-trace` ブランチにある。`main` を更新するだけでは
導入されない。実験とGUIのLaunchを停止してから、ローバーで以下を行う。

```bash
cd ~/harada-tsubakino-rover-agent
git config --local core.filemode false
git status --short
```

何も表示されないことを確認する。変更が表示された場合はそこで止め、変更を消さずに
[トラブル対応](troubleshooting.md#更新できない手元の変更がある)を確認する。
続けてブランチを選ぶ（`git checkout` は既存のRaspberry Pi OSのGitでも使用できる）。

```bash
git fetch origin
git checkout feature/drive-trace
git symbolic-ref --short HEAD
```

表示が `feature/drive-trace` であることを確認してから更新・配置する。
初回のcheckoutは `origin/feature/drive-trace` からローカルブランチを作る。

```bash
git pull --ff-only origin feature/drive-trace
python3 update.py -r
```

各コマンドが成功した場合だけ次へ進む。`python3 update.py -r` は現在checkout中の
作業ツリーをcatkinワークスペースへコピーするため、ブランチ変更後には必ず再実行する。
今回の更新ではArduinoの再書き込みやGUIの更新は不要。

新しく取得する場合は以下で検証ブランチを選び、その後 [セットアップ手順](setup.md) の
ROSパッケージ配置以降を行う。

```bash
cd ~
git clone --branch feature/drive-trace https://github.com/KaitoKonda/harada-tsubakino-rover-agent.git
cd ~/harada-tsubakino-rover-agent
git config --local core.filemode false
```

GUIのLaunchを起動し直す。既定では計測版が選ばれる。トピック・ノード名、
`odom` のremap、TFの分離は維持する。I2Cサーバーはメーカー版を使う。

| GUIのArgsへ追記する値 | 動作 |
| --- | --- |
| 指定なし | 計測版、ログ有効 |
| `trace_enabled:=false` | 計測版、ログのROS配信・時刻計測を無効化 |
| `instrumented_drive:=false` | メーカー版の制御・オドメトリへ戻す |

手動起動例（既存のROS環境設定後）:

```bash
roslaunch harada-tsubakino harada-tsubakino.launch group:=pi1
```

メーカー版と計測版を同時に起動しない。上記の起動引数は排他的にノードを選択する。
引数を変えるときも、既存のLaunchを停止して起動し直す。

### mainの配置へ戻す

実験とLaunchを停止し、作業ツリーに変更がないことを確認してから行う。

```bash
cd ~/harada-tsubakino-rover-agent
git status --short
```

何も表示されなければ以下を実行し、成功後にGUIのLaunchを起動し直す。

```bash
git checkout main
git pull --ff-only origin main
python3 update.py -r
```

`main` 側のlaunchには検証用の起動引数がないため、GUIのArgsから
`instrumented_drive:=...`、`trace_enabled:=...`、`trace_queue_capacity:=...`、
`trace_publish_hz:=...` を外す。ホスト名の `group:=...` は残す。
Gitのブランチを戻すだけでは既にコピー済みのROSパッケージは切り替わらない。

## PCでの保存

4トピックともROS標準の `std_msgs/String`。内容はスキーマ版1のJSONで、独自ROS型の
生成は不要。MATLABのROS接続を確立したうえで、PC側のこのリポジトリの `tools` を
パスに追加する。以下は指令を送信せず、内部ログだけを保存する。

```matlab
addpath('C:/Users/Kaito/Desktop/harada-tsubakino-rover-agent/tools')
recorder = DriveTraceRecorder("pi1", fullfile(pwd,"logs"));
% この後、通常どおり runControlStation でマウス操作の実験を実行する。
% 実験終了後:
recorder.close();
logFile = recorder.logFile;
disp(recorder.messageCount)
```

複数台なら `DriveTraceRecorder(["pi1","pi3"])` とする。記録開始はすぐに戻るので、
同じMATLABセッションで既存の実験を実行できる。終了時に `close` を呼び、ファイルを閉じる。
中断した場合も `recorder.close()` を実行する。実験を関数でまとめる場合は、
エラーや `Ctrl+C` に備えて `onCleanup(@()recorder.close())` を併用する。
別セッション等から60秒だけ記録する場合には `recordDriveTrace("pi1",60)` も使える。
初期化時の書き込みも必要なら、**PC記録開始後にローバーのLaunchを起動**する。
既に起動済みの場合、設定メタデータは受け取れるが、過去のイベントは復元されない。
時間指定の `recordDriveTrace` は `Ctrl+C` で打ち切った場合もファイルを閉じる。

JSONLの各行は `topic`、`pc_received_at`（PCのUTC時計）、`data`（ローバーのJSON原文）。
読み取り例:

```matlab
fid = fopen(logFile,'r');
row = jsondecode(fgetl(fid));
payload = jsondecode(row.data);
fclose(fid);
```

これは既存のMAT実験ログとは別ファイル。OTOSとPCの指令は従来どおり実験ログに記録する。
ROS 1のrosbag記録環境があるPCでは、代わりに次のようにも保存できる。

```bash
rosbag record -O pi1-drive.bag /pi1/drive_trace/controller /pi1/drive_trace/controller_metadata /pi1/drive_trace/wheel /pi1/drive_trace/wheel_metadata /pi1/rover_drive /pi1/odometry/wheel /pi1/odometry/otos
```

## イベントと項目

車輪配列の並びは常に基板の **motor0, motor1**。取り付け方向の符号はメーカー処理どおり。

| トピック（先頭に `/pi1` 等） | 内容 |
| --- | --- |
| `drive_trace/controller` | 指令受信・制御・書き込みのイベントをまとめたJSON |
| `drive_trace/wheel` | エンコーダ読み取り・速度計算・オドメトリ配信のイベント |
| `drive_trace/controller_metadata` | 制御側の起動識別子、実行ソースSHA256、元ファイルのGit blob、設定 |
| `drive_trace/wheel_metadata` | 車輪側の起動識別子、実行ソースSHA256、換算係数・設定 |

メタデータはlatched配信し、約1秒ごとにも再配信する。設定はソフトウェアが書き込む値・
計算に使う値で、基板からの読み戻し結果ではない。起動時の書き込み結果は別イベントに残す。

| `event` | 主な項目 |
| --- | --- |
| `command_received` | 指令連番、Pi受信時刻、v/omega、配信元caller ID、変換後の左右目標速度 |
| `control_cycle` | 周期、開始時の指令情報・経過時間、終了時の指令連番、車輪odomのstamp/seq、実測・目標・加減速後の左右速度、実際に使った加減速/積分時間、整数加算値、小数繰越、3回のI2C要求・応答・所要時間、成功/例外 |
| `initialization_write` | 起動時の既存7回の書き込みの要求・応答・所要時間 |
| `wheel_sample` | 計測連番、前回/今回の生カウント、差分、差分採用判定、実際に使用した差分・時間差、算出v/omega、2回の読み取りの要求・応答・所要時間、成功/例外 |
| `wheel_published` | 対応する計測連番、odomのstamp、配信した位置・方位・速度 |

`service_calls` の `success` はサービスが応答したことを意味し、物理的なモーター動作の保証ではない。
3回の書き込み途中で失敗した場合、そこまでの成功・失敗を記録し、元の例外を再送出する。
失敗後に追加のトリガや再試行を行わない。車輪のサービス例外に対する戻り値も元の処理を維持する。
`wheel_sample` の最初の1回は元コードのウォームアップ計算であり、odom配信が対応しない。

## 時刻と欠落

- 各イベントの `monotonic_s` はPiの単調時計。Python 2ではLinuxの `clock_gettime` を使う。
  同一Pi・同一起動の両ノード間で比較できる。`boot_id` を確認する。
- `ros_time_ns` はイベントをキューへ入れる時点のROS時刻。処理開始/受信の時刻は
  `monotonic_s`、I2Cはその開始・終了時刻を使う。PCの受信時刻とは区別する。
- 元の制御計算の `time.time()` は変更しない。制御で使用した時間差と単調時計の所要時間を別々に残す。
- ノードごとに `run_id` を発行する。`seq` はイベント連番、`batch_seq` は送信バッチ連番。
  再起動時にリセットされるため、ノード名だけで連結しない。
- メモリキューは既定2048イベント。満杯なら古い記録を落とし、`queue_dropped` を累積する。
  JSON変換・ROS送信は別スレッドで既定20 Hz、最大128イベントずつ処理する。
  ROSの送信キューも有限。バッチ/イベント連番の欠落でPC側の受信欠落も確認する。
- `capture_errors`、`publish_errors` は内部の記録/送信失敗数。送信失敗時のバッチは再送しない。
  通信断、購読開始前、異常終了・終了直前の未送信分はPCに残らない。確実な永続保存の仕組みではない。
- `trace_queue_capacity` と `trace_publish_hz` はlaunch引数。変更前後で制御周期・I2C所要時間を比較する。

メーカー版の目標値更新方式を維持しており、別コールバックの指令更新と制御計算が重なることが
ある。開始/終了の指令連番・目標値を併記するが、全内部値を同時刻の原子的なスナップショットとは
みなさない。`command_at_start` はコールバック開始時の観測で、競合時の厳密な使用指令の保証ではない。
サービス所要時間もクライアントからの往復全体で、サーバー待ちとI2C実通信の分離は行わない。

## 今回取得しない値

基板の目標位置・出力・電圧の追加読み取り、電流・実トルク、OTOSの時刻付き診断トピックは
今回追加していない。内部で計算した増分の送信値から基板の実際の位置誤差や出力は断定できない。
OTOSは既存ブリッジと既存トピックのまま使う。

## 検証

PCのPython 3で、ROS・実機なしの回帰テストを実行できる。

```bash
python3 -m unittest discover -s tests -v
```

固定したメーカー版へ同じ指令・エンコーダ値・計算時刻を与え、出力・小数繰越・位置計算・
I2C要求列を比較する。ログ有効/無効、サービス失敗、キュー満杯と送信失敗も検証する。
Pi上のcatkinビルド、Python 2 / ROS実行、実機での周期・負荷の比較は別途必要。

PC記録ツールも、ROS接続を作らないMATLABテストで検証できる。

```matlab
results = runtests('tests/matlab/testDriveTraceRecorder.m');
assertSuccess(results)
```

`tests/matlab/helpers` はテスト用のROS代替関数を含む。実験時のMATLABパスへ追加しない。

移植元と変更範囲は [upstream-drive.md](upstream-drive.md) に記載する。
