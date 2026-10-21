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
| 指定なし / `trace_mode:=full` | 計測版、詳細ログ＋共通の時間集計を配信 |
| `trace_mode:=minimal` | 計測版、時間計測・集計だけを行い、既定5秒ごとに配信 |
| `trace_mode:=off` | 計測版、追加の時間計測・ROSログ配信を無効化 |
| `instrumented_drive:=false` | メーカー版の制御・オドメトリへ戻す |

`trace_mode` を明示すると、互換用の `trace_enabled` より優先する。
`trace_mode` が空欄（既定）のときは、従来の `trace_enabled:=true/false` を
それぞれ `full/off` として扱う。`trace_enabled:=minimal` は使わない。
`trace_mode` の綴り違いなど、不正なモード指定は起動エラーになる。

`minimal` では詳細レコード、イベントキュー、毎周期のJSONを作らず、件数・合計・最小・最大・
最初・最後の値だけを一定量のメモリで集計する。`full` も同じ計測点・同じ集計方法を使う。
`off` は比較用の内部時間も取得しない。モード変更はLaunchを停止して起動し直す。

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
`instrumented_drive:=...`、`trace_enabled:=...`、`trace_mode:=...`、
`trace_queue_capacity:=...`、`trace_publish_hz:=...`、`trace_summary_interval:=...`
を外す。ホスト名の `group:=...` は残す。
Gitのブランチを戻すだけでは既にコピー済みのROSパッケージは切り替わらない。

## PCでの保存

各トピックはROS標準の `std_msgs/String`。内容はスキーマ版1のJSONで、独自ROS型の
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
rosbag record -O pi1-drive.bag /pi1/drive_trace/controller /pi1/drive_trace/controller_metadata /pi1/drive_trace/controller_timing /pi1/drive_trace/wheel /pi1/drive_trace/wheel_metadata /pi1/drive_trace/wheel_timing /pi1/rover_drive /pi1/odometry/wheel /pi1/odometry/otos
```

## イベントと項目

車輪配列の並びは常に基板の **motor0, motor1**。取り付け方向の符号はメーカー処理どおり。

| トピック（先頭に `/pi1` 等） | 内容 |
| --- | --- |
| `drive_trace/controller` | 指令受信・制御・書き込みのイベントをまとめたJSON |
| `drive_trace/wheel` | エンコーダ読み取り・速度計算・オドメトリ配信のイベント |
| `drive_trace/controller_metadata` | 制御側の起動識別子、実行ソースSHA256、元ファイルのGit blob、設定 |
| `drive_trace/wheel_metadata` | 車輪側の起動識別子、実行ソースSHA256、換算係数・設定 |
| `drive_trace/controller_timing` | 共通の時間集計。full/minimalで配信 |
| `drive_trace/wheel_timing` | 車輪側の共通の時間集計。full/minimalで配信 |

詳細イベントの2トピックは `full` のみで配信する。`minimal` は時間集計とメタデータの
4トピックを配信し、`off` はいずれも配信しない。PC記録器は6トピックすべてを購読するため、
full/minimalで記録方法を変える必要はない。PC側の記録器もこの版へ更新して使う。

メタデータはlatched配信し、fullでは約1秒、minimalでは集計間隔ごとにも再配信する。
`trace_mode` と `summary_interval_s` が実際に動作している設定を表す。
設定はソフトウェアが書き込む値・
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

## fullとminimalの時間比較

1. PCの記録器を開始し、実験・Launchを停止した状態から始める。
2. GUIのArgsに `group:=pi1 trace_mode:=full` を指定してLaunchする。
3. ゼロ指令を同じ周期で送り続け、起動後10秒程度を除いて、その後60秒程度を記録する。
4. 実験・Launchを停止し、`trace_mode:=minimal` に変える。新しくLaunchして同じ操作を行う。
5. 両方の `_timing` を比較する。開始時の古い受信待ちを揃えるため、各条件でLaunchを起動し直す。

詳細ログの配信周期だけを下げても記録の生成量は変わらないため、比較にはモードを切り替える。
`instrumented_drive`、制御設定、指令周期、ネットワーク、実験時間、PC側の記録条件は揃える。
`off` では内部時間を測れない。今回の比較対象は `full` と `minimal`。

`_timing` のJSONには以下を含む。

| 項目 | 意味 |
| --- | --- |
| `trace_mode`, `run_id`, `node` | 動作モードとノードの起動識別子 |
| `summary_seq` | 集計の連番。受信欠落を検出するために使用 |
| `window_start_monotonic_s`, `window_end_monotonic_s`, `window_duration_s` | 実際の集計期間。ノード起動からの絶対秒数ではなく、Piの単調時計の値 |
| `metrics` | 各計測の `count`, `sum`, `mean`, `min`, `max`, `first`, `last`, `failures` |
| `counters` | 区間内の指令受信・制御・車輪計測・失敗等の件数 |
| `partial` | 終了時の端数区間ならtrue（終了時送信はベストエフォート） |

主な計測名:

- `control_period_s`: 制御コールバック開始間隔。最初の1回は値がないのでcountに含めない。
- `control_duration_s`: 制御開始から3回のサービス呼び出し終了まで。集計・詳細レコード生成の前に終点を取る。
- `wheel_odom_age_at_start_s`: 制御開始時のROS時刻と受信した車輪odomのstampとの差。
- `command_age_at_start_s`: 制御開始時点の最後の指令からの経過時間。指令未受信なら集計しない。
- `wheel_sample_duration_s`, `wheel_calculation_dt_s`: 車輪の取得・計算時間と、元の計算で使う時間差。
- `service_w_0x48_s`, `service_w_0x4c_s`, `service_w_0x11_s`: 既存3回の書き込みサービス往復。
- `service_r_0x60_s`, `service_r_0x64_s`: 既存2回の読み取りサービス往復。

値の単位は秒。集計は送信ごとにリセットする。処理が区間をまたぐ場合は、値を集計した区間に
件数を含める。処理停止中は空の集計が届く場合もあり、0件を所要時間0秒と解釈しない。
各サービスの `failures` はその呼び出しが例外になった件数。
`counters.control_failures` / `wheel_sample_failures` は制御・計測全体の失敗件数。
該当するカウンターが区間にない場合は0件。

複数区間の平均は `sum` の合計を `count` の合計で割る。窓ごとの平均を単純平均しない。
最大値は各区間の `max` の最大を使う。中央値は取得しないので、fullでも同じ時間集計の
平均・最大値を比較する。開始後10秒などの除外区間はメタデータの `run_id` と集計窓の時刻で選ぶ。
端数の `partial=true` を除き、実際の `window_duration_s` と件数も確認する。

既定の集計間隔は5秒。`trace_summary_interval:=5.0` で指定でき、両条件で同じ値を使う。
`minimal` でも計測・小さな集計・低頻度のメタデータ配信の負荷は残る。
サービス往復時間だけではサーバー待ちとI2C実通信を分離できない点は、fullと同じ。

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
- `trace_queue_capacity` と `trace_publish_hz` はfullの詳細ログ用のlaunch引数。
  `trace_summary_interval` はfull/minimal共通の集計間隔。メタデータと集計を別スレッドで配信する。

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
I2C要求列を3モードで比較する。full/minimalの共通集計、従来オプションとの互換性、
サービス失敗、キュー満杯と送信失敗も検証する。
Pi上のcatkinビルド、Python 2 / ROS実行、実機での周期・負荷の比較は別途必要。

PC記録ツールも、ROS接続を作らないMATLABテストで検証できる。

```matlab
results = runtests('tests/matlab/testDriveTraceRecorder.m');
assertSuccess(results)
```

`tests/matlab/helpers` はテスト用のROS代替関数を含む。実験時のMATLABパスへ追加しない。

移植元と変更範囲は [upstream-drive.md](upstream-drive.md) に記載する。
