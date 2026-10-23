# ローバー内部の走行ログ

位置P制御を外して直接出力を指定できるか確認し、その後にモデルを同定する計画は
[モータ直接出力の確認と B_m 同定の設計案](motor-output-identification-design.md) を参照する。
段階A〜Cの専用試験モードは [確認用ノードの手順](motor-output-probe.md) を参照する。
同モードの追加読み取り・停止処理は、以下の通常ログとは区別する。

既存の走行処理に計測を追加したノードを、このリポジトリから配置する。
メーカーの `lightrover_ros` 内のファイルは編集しない。追加のI2C読み取り、制御式・
ゲイン変更、指令タイムアウトの追加は行わない。既存のPython 2 / ROS Melodicを対象とする。

## 配置・起動

内部ログと走行処理の改善は `main` に含まれる。以前の `feature/drive-trace` を
配置している場合も、実験とGUIのLaunchを停止してから、ローバーで以下を行う。

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
git checkout main
git symbolic-ref --short HEAD
```

表示が `main` であることを確認してから更新・配置する。
ローカルにmainがない場合、初回のcheckoutは `origin/main` からローカルブランチを作る。

```bash
git pull --ff-only origin main
python3 update.py -r
```

各コマンドが成功した場合だけ次へ進む。`python3 update.py -r` は現在checkout中の
作業ツリーをcatkinワークスペースへコピーするため、ブランチ変更後には必ず再実行する。
今回の更新ではArduinoの再書き込みやGUIの更新は不要。

新しく取得する場合は以下でmainを選び、その後 [セットアップ手順](setup.md) の
ROSパッケージ配置以降を行う。

```bash
cd ~
git clone --branch main https://github.com/KaitoKonda/harada-tsubakino-rover-agent.git
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

サービス接続方式は `i2c_persistent:=true`（既定）で永続接続、
`i2c_persistent:=false` で従来の非永続接続を選ぶ。ログのモードとは独立で、
`instrumented_drive:=true` の計測版ノードに適用する。
メーカー版を選ぶ `instrumented_drive:=false` ではこの引数は適用されない。

手動起動例（既存のROS環境設定後）:

```bash
roslaunch harada-tsubakino harada-tsubakino.launch group:=pi1
```

メーカー版と計測版を同時に起動しない。上記の起動引数は排他的にノードを選択する。
引数を変えるときも、既存のLaunchを停止して起動し直す。

### メーカー版の走行処理へ切り替える

実験とLaunchを停止し、GUIのArgsへ以下を追加して起動し直す。

```bash
group:=pi1 instrumented_drive:=false
```

`pi1` は実機のホスト名に置き換える。この切替ではメーカー版の制御・車輪odomを使用し、
内部ログ、I2C永続接続、車輪odom購読の滞留・転送待ち対策、エンコーダ折り返し補正は適用されない。
シリアルセンサーの監視は引き続き動作する。

改善版へ戻す場合は、Launchを停止して `instrumented_drive:=false` を外すか
`instrumented_drive:=true` に変える。`trace_mode:=off` は追加計測を止めるだけで、
走行処理は改善版のままである。mainへ切り替えてもメーカー版には戻らない。

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
- `service_lookup_w_0x48_s`, `service_lookup_w_0x4c_s`, `service_lookup_w_0x11_s`:
  各書き込みでのROSサービスURI検索時間。起動時の他アドレスも同じ命名で集計する。
- `service_lookup_r_0x60_s`, `service_lookup_r_0x64_s`: 各読み取りでの同検索時間。

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

## サービス検索時間の切り分け

PC側は従来の実験と `DriveTraceRecorder` のままでよい。指令を
`linear.x=0`、`angular.z=0` に保ち、従来と同じ周期で送信し続ける。
記録器の更新や新しいトピックの購読は不要。新しい値は既存の
`controller_timing` / `wheel_timing` に含まれる。

1. 実験・Launchを停止し、mainを更新して `python3 update.py -r` でPiへ配置する。
2. PCの記録器を開始し、GUIのArgsを `group:=pi1 trace_mode:=minimal i2c_persistent:=false` としてLaunchを起動し直す。
3. 従来どおりPCの実験を開始し、ゼロ指令のまま約80秒記録する。
   最初の約10秒（集計連番1・2）を除いて、完全な集計窓を60秒分以上確保する。
4. 実験を停止した後に記録器を閉じる。起動条件とネットワークを揃えて2～3回取得すると、
   試行間の揺らぎも確認できる。

`trace_mode=full/minimal` のどちらも検索を測り、`off` は検索の追加計測も行わない。
上記は従来の非永続接続で検索時間を測る手順。メーカーのI2Cサーバー・ROS本体は編集しない。
追加のサービス検索・I2C要求・再試行も行わない。

メタデータの `service_lookup_timing=ServiceProxy._get_service_uri` が計測対応版の目印。
`service_proxy_persistent`（実際の永続接続設定）と `ros_master_uri`（Piプロセスの環境変数）も記録する。
これで、実際に指定されていたmasterの接続先をログから確認できる。

ROS Melodicの `ServiceProxy` をこちらのパッケージ内で継承し、既存の `_get_service_uri`
の前後だけを単調時計で測る。検索・接続・送受信・切断はROSの元の処理へ委ねる。
この検索処理には要求の検査、masterへの問い合わせなどを含み、TCPROSサービス接続の確立と
I2C処理は含まない。非永続接続では通常、各サービス呼び出しでこの検索が行われる。
Pi側のmaster指定がPCなら、この検索はPCとの通信を伴う。
実装根拠: [ROS MelodicのServiceProxy](https://github.com/ros/ros_comm/blob/melodic-devel/clients/rospy/src/rospy/impl/tcpros_service.py)。

同じ操作・アドレスについて `service_lookup_*` と既存の `service_*` の平均・件数を比較する。
検索時間が全体時間の大部分なら、master問い合わせの寄与が大きい。
検索が短く全体が長いなら、接続確立・送受信・サーバー内処理等を次に切り分ける。
全体から検索を引いた値だけで、通信・I2Cのどちらに時間がかかったかは断定しない。

検索の集計は検索終了時、全体の集計はサービス呼び出し終了時に行う。
窓をまたぐと件数がずれることがあるので、複数の完全な窓を合算し、件数と `failures` を確認する。
平均の差は傾向を見るための値であり、1件ごとに対応付けた厳密な差ではない。
最大値同士を引いて残り時間を算出しない。
検索側の `failures` はその検索処理が例外になった件数であり、検索後の接続・I2C処理の失敗は
既存のサービス全体の `failures` に記録する。`wait_for_service` による起動時の待機は対象外。
検索の集計処理自体の小さな負荷は、既存のサービス全体時間にも含まれる。

## 永続接続との比較

実験とLaunchを停止し、Piのmainを更新・再配置してから行う。
非永続接続の記録と同じPC側の実験・記録器を使い、以下のArgsでLaunchを起動する。

```text
group:=pi1 trace_mode:=minimal i2c_persistent:=true
```

ゼロ指令を従来と同じ周期で送り、約80秒記録する。
最初の約10秒を除き、完全な集計窓を60秒分以上比較する。
比較条件へ戻す場合は `i2c_persistent:=false` に変え、必ずLaunchを起動し直す。
既定がtrueなので、永続接続を使う場合は `i2c_persistent:=true` の明示を省略してもよい。
ノードを直接起動する場合も、パラメーター未指定なら永続接続になる。
両条件とも新しく起動して、古い車輪odomの滞留を持ち越さない。
PC記録器・Arduino・メーカーのパッケージを更新する必要はない。

メタデータの `service_proxy_persistent=true/false` で実際の条件を確認する。
制御側と車輪側はそれぞれ専用のサービスクライアントを持つ。
永続接続では初回の検索・接続後に同じ通信路を再利用するため、通常の継続区間に
`service_lookup_*` がないのは正常。この場合は検索が0件であり、検索時間0秒のサンプルではない。
起動時の初期化要求で接続することもあるため、最初の検索アドレスが走行中のアドレスとは限らない。

既存の `service_*` の平均・最大・件数・失敗数、`control_period_s`、
`wheel_calculation_dt_s`、`wheel_odom_age_at_start_s` を比較する。
永続接続は毎回の検索と接続確立の両方を省くため、改善の全量を検索だけの効果とは呼ばない。

同じクライアントへの呼び出しは永続接続時のみロックで直列化し、応答の取り違えを防ぐ。
例外になった呼び出しは元の例外を返し、その接続を閉じて保存済み通信路を破棄する。
**失敗した要求は自動再送しない。次の通常の呼び出しがあったときに、検索・接続を作り直す。**
応答が失われただけで位置加算が適用済みの場合、再送すると二重加算になるため。
閉じる処理の失敗も元の呼び出し例外を置き換えない。
失敗後の制御・オドメトリの例外処理、ゲイン、計算式、I2C要求の値・順序は従来どおり。
今回はサービス呼び出しのタイムアウトや追加の停止指令・復旧時の初期化書き込みは追加しない。

## 車輪odomの滞留対策の確認

制御側の車輪odom購読に `queue_size=1` を指定し、処理が追いつかない場合に
受信バッチ内の古いメッセージを間引く。メタデータの `wheel_odom_queue_size=1` が目印。
この設定はログのfull/minimal/offにかかわらず、計測版の制御ノードに適用する。
メーカー版を選ぶ `instrumented_drive:=false` の処理は変更しない。

実験とLaunchを停止し、mainを更新して `python3 update.py -r` でPiへ再配置する。
PCの記録器を開始し、次のArgsでLaunchを起動し直す。

```text
group:=pi1 trace_mode:=minimal
```

永続接続は既定で有効。以前と同じ周期のゼロ指令で約80秒記録する。
PCの実験・記録器、Arduino、メーカーのパッケージは従来どおり。
最初の約10秒を除き、以下を前回の永続接続の記録と比較する。

- `wheel_odom_age_at_start_s`: 遅れが時間とともに増え続けないか。平均・最大・先頭・末尾を確認する。
- `control_period_s` / `control_duration_s` / `service_*`: 制御周期・処理時間とサービス失敗件数。
- `wheel_samples` / `wheel_published` / `control_cycles`: 生成数と制御回数。
  古いodomを間引くため、生成数と制御回数が一致する必要はない。

受信コールバック内でI2Cサービスを待つ構成は維持している。
`queue_size=1` はTCP受信バッファを含めた常時最新の観測を保証する設定ではないため、
実際に遅れの増大が止まるかをログで確認する。
車輪odomの生成・配信、制御式、ゲイン、I2C要求の内容・順序は変更していない。
制御の加減速・位置加算で使う時間差は、従来どおり実際の処理時刻から算出する。
新しい設定の効果を確認するため、既存の滞留があるLaunchを継続せず、必ず起動し直す。

## 車輪odom転送と制御周期の確認

制御側の車輪odom購読は `queue_size=1` に加えて `tcp_nodelay=True` を指定する。
TCPROSの接続要求を通じて配信側ソケットのNagleアルゴリズムを無効にし、
小さなメッセージをまとめるための送信待ちを抑える。
この指定はログのfull/minimal/offにかかわらず計測版の制御ノードに適用する。
制御側メタデータの `wheel_odom_tcp_nodelay=true` が指定の目印になる。
これは接続時に要求する設定の記録であり、ソケット状態の実測値ではない。

これまでのログでは制御処理そのものが約16 msなのに対し、制御間隔の約半数が約50 msで、
処理終了後から次の制御開始までの空白が増えていた。
TCP転送待ちは原因候補であり、この設定による改善は実機で確認する。
制御を固定周期化する変更ではなく、車輪odomの受信を契機に制御する構成を維持する。

1. 実験とLaunchを停止し、mainを更新して `python3 update.py -r` でPiへ再配置する。
2. PCの記録器を開始し、Argsを `group:=pi1 trace_mode:=full` としてLaunchを起動し直す。
   I2C永続接続は既定で有効。PC側は従来と同じ周期でゼロ指令を送り、約80秒記録する。
3. 実験終了後に記録器を閉じる。最初の約10秒を除き、60秒分以上を前の設定のログと比較する。

`control_period_s` の約50 msへの集中と、各 `control_cycle` の
`monotonic_s` から直前の `end_monotonic_s` を引いた待ち時間が減るかを確認する。
`control_duration_s`、`wheel_calculation_dt_s`、`wheel_odom_age_at_start_s`、
サービス失敗数も比較する。minimalでも共通集計の平均・最大・件数は比較できるが、
周期の分布と周期間の待ち時間を調べるにはfullを使う。

## エンコーダの折り返し補正

計測版の車輪odomでは、カウンタの現在値と前回値の差を、32 bitの範囲内で最小の
符号付き差分へ補正する。例えば `0 -> 0xffffffff` は-1 count、
`0xffffffff -> 0` は+1 count。符号付き整数として返る場合の境界も同じように扱う。
左右車輪の符号変換は、その補正後の差分に従来どおり適用する。

メタデータの `encoder_delta_mode=signed_modulo_32` が対応版の目印。
fullの `wheel_sample` は、受信した値の単純な差 `raw_delta_counts` と、
補正後・車輪符号変換前の差 `wrapped_delta_counts` を両方記録する。
`delta_accepted` は補正後の差を既存の `diff_count_limit` で判定した結果。
`used_delta_counts` は、従来と同じ車輪符号を適用して実際に計算へ使った値。
full/minimal/offの全モードにこの補正を適用する。

実験とLaunchを停止し、mainを更新して `python3 update.py -r` で再配置する。
Argsはこれまでどおり `group:=pi1 trace_mode:=full` を使い、
小さい前後進・旋回で0付近をまたぐときに補正後の差分が連続するか確認する。

この変更はカウンタの正しい折り返しに対する補正。
ログにあった `0xffffffff -> 0x0000ffff -> 1` のように途中で不自然な値を返す場合は、
補正後も+65536、-65534 countとなり、既存の閾値1048575より小さいため受理される。
±250 count程度のペアもこの補正の対象外で、読み取り整合性や外れ値判定の対策が別途必要。
不採用時の `diff_count` の保持と `pre_count` の更新は現時点ではメーカー由来の処理を維持する。
I2C読み取りは各周期2回のままで、追加の再読み取りは行わない。

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
サービス検索の時間・失敗・同時呼び出し、永続接続の再利用・直列化・失敗後の次回接続、
ノード起動時の設定とメタデータ、サービス失敗、キュー満杯と送信失敗も検証する。
Pi上のcatkinビルド、Python 2 / ROS実行、実機での周期・負荷の比較は別途必要。

PC記録ツールも、ROS接続を作らないMATLABテストで検証できる。

```matlab
results = runtests('tests/matlab/testDriveTraceRecorder.m');
assertSuccess(results)
```

`tests/matlab/helpers` はテスト用のROS代替関数を含む。実験時のMATLABパスへ追加しない。

移植元と変更範囲は [upstream-drive.md](upstream-drive.md) に記載する。
