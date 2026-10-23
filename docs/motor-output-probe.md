# モータ直接出力の確認（段階A〜C）

2026-10-23時点で試験ノード・ログ・復旧処理を実装済み。実機でのT_OUTの作用とWDT停止は未確認。
[設計書](motor-output-identification-design.md)の段階A〜Cを実行するための文書であり、
接地した同定実験・モデル推定・DOBの実装は、その実機確認後に行う。

## 変更する範囲

専用ノード `tracedMotorOutputProbe.py` が、出力禁止中に設定を保存し、位置Pゲインと
パンチをゼロにして出力オフセットT_OUTを書き込む。通常の `pos_controll()` と
`harada-tsubakino.launch` は変更していない。ファームウェアの書き換えは行わない。
`rover_drive` は試験ノードの指令入力ではない。

出力は符号付きの **T_OUTレジスタ値** で指定する。m/sや確定したデューティ比ではない。
配列は左右ではなく基板の `[motor0,motor1]` の順である。

メーカーのI2C実装は2/4 bytesのレジスタを下位byteから別々に書くため、
**試験ノードは毎回O_EN=0を確認し、左右の出力を書いて読み戻してからWDTを設定し、
O_EN=3にする。** この間の出力禁止時間も応答に含まれる。現時点の試験波形を、そのまま
連続PWMのモータモデルと解釈しない。将来の同定ではEnableの操作時刻も入力再構成に使う。

## 1. 配置と準備

この変更は `feature/drive-system-identification` で配置する。Piでの既存の更新手順に従い、
通常Launchと実験を止めてからブランチを更新し、`python3 update.py -r` で配置する。
Python 2 / ROS Melodic、catkinの実行確認はPiで行う。

初回は車輪を浮かせて行う。通常の位置制御ノードを両方とも停止し、
`rover_drive` を購読する他の制御ノードも動かさない。
試験ノードはROSグラフの既知の位置制御ノード・速度指令購読者との競合を検出し、
同一Pi内の試験ノードはファイルロックで二重起動を防ぐ。ただし、外部から直接I2Cを
書くクライアントまで排除できるものではない。
ROSグラフの監視は別スレッドで0.5秒ごとに行い、出力の周期処理でROSマスターへ問い合わせない。
監視エラーまたは2秒を超える監視停止を検出した場合もFAULTにする。

通常Launchをすべて止めてI2Cサービスと車輪odomが存在しない場合だけ、後述の
`start_i2c:=true start_wheel:=true` を指定する。すでに両方が存在し、通常位置制御だけを
止めた構成では両引数を指定しない。メーカーI2Cサーバーは起動時に基板を初期化する。
復旧時に新たなI2Cサーバーを起動すると、復旧前の設定を失うので再利用する。

PCではROSを接続し、従来の `DriveTraceRecorder` をLaunch前から開始する。

```matlab
addpath("harada-tsubakino-rover-agent/tools") % 実際の配置先に合わせる
recorder = DriveTraceRecorder("pi1", fullfile(pwd,"logs"));
```

## 2. 試験設定を作る

`harada-tsubakino/config/motor_output_probe.yaml` をPiの編集用ファイルへコピーする。
配布設定は `wheels_lifted: false`、上限ゼロ、入力列なしのため、**試験ノードは基板へアクセスする前に
設定エラーで終了する**。別途起動したメーカーI2Cサーバー自身の初期化とは区別する。
非ゼロ出力を既定で実行しない。

例として次のように設定する。上限・振幅・周期は実機で選定する。
以下の `10` は小さな試験要求の例であり、回転開始や実デューティを保証する値ではない。
初回は出力確認より先に、開始しない段階Aで通信・復旧ログを確かめる。

```yaml
trace_mode: full
wheels_lifted: true
output_limits: [10, 10]
experiment: pulse
period_s: 0.1
command_timeout_s: 0.3
watchdog_ms: 500
sensor_timeout_s: 0.3
max_duration_s: 10.0
watchdog_wait_s: 1.0
snapshot_path: /home/pi/.ros/pi1-output-probe-001.json
sequence:
  - {duration_s: 0.5, t_out: [0, 0]}
  - {duration_s: 0.5, t_out: [10, 0]}
  - {duration_s: 0.5, t_out: [0, 0]}
  - {duration_s: 0.5, t_out: [-10, 0]}
  - {duration_s: 0.5, t_out: [0, 0]}
```

`0 < period_s < command_timeout_s < watchdog_ms/1000` が必要。
出力上限は各輪1〜4096、入力列の振幅は上限以内、各区間はperiod_s以上とする。
試験全体（WDT観測待ちを含む）はmax_duration_s未満にする。
実際のI2C処理が期限を消費すればFAULTになる。WDTだけを別スレッドで延命しない。
設定は要求値であり、ハードウェアで停止が成立することは段階Cで確認する。

スナップショットは作成専用・上書き禁止で、設定変更前にPiへfsyncしてから進む。
復旧完了を確認したファイルも残すため、次の試験は別名を使う。
異常終了した試験のファイルを新しい試験のために消したり上書きしたりしない。
車両名、基板識別値、ファームウェア版、採取時刻と元設定が保存される。
基板に固有のシリアル番号は確認できていないため、同じ版の別個体への流用は避ける。

## 3. 起動し、出力禁止状態を確認する（段階A）

以下は通常Launchを止め、既存のI2Cサービスと車輪odomもない場合の例。

```bash
roslaunch harada-tsubakino motor-output-identification.launch group:=pi1 config:=/home/pi/output-probe.yaml start_i2c:=true start_wheel:=true
```

設定の保存、Pゲイン・パンチ・上限の読み戻し、出力禁止中の `-1,1,0` のT_OUT読み戻しを
行った後、`READY` で待つ。この時点では出力を有効にしない。
`output_configuration` の結果とスナップショットを確認する。
段階Aだけで終了する場合も、次のstopを呼んで復旧する。

```bash
rosservice call /pi1/rover_output_probe/stop "{}"
```

ROSサービスの返答は要求の受付を意味し、I2C操作完了ではない。
`output_restore.success=true`、`output_disabled=true`、両輪のゼロ化と最終 `DISABLED` を確認する。

## 4. 有限の正負出力を確認する（段階B）

別のスナップショット名で起動し、READYから明示的に開始する。
新鮮で有限な車輪odomがない場合、またはfullログが使えない場合は開始しない。

```bash
rosservice call /pi1/rover_output_probe/start "{}"
```

入力列を一度だけ実行し、出力禁止・T_OUTゼロ・設定復元後に終了する。
再度startして同じプロセスを再利用することはできない。
このLaunchで起動したI2Cサーバーと車輪odomは、試験ノード終了後もLaunchを閉じるまで残る。
復旧が必要なら両者を維持したまま復旧専用の起動を行う。
T_OUT書き込みに位置加算用TRIGは送らない。
T_OUTとM_OUTの倍率、符号、ゼロ指令、回転方向をログと実機で比較する。
M_OUTが要求と一致することや、`0x1000`換算がT_OUTにも使えることを自動で合格扱いしない。
M_OUTが設定した試験上限を超える場合、またはT_OUTの読み戻しが指令と異なる場合は中止する。

試験中もstopを呼べる。I2Cは単一ワーカーが直列実行し、コールバックから割り込んで
書き込まない。I2C処理が停止した場合はPythonのstop操作だけでは止められないため、
WDTと実機の停止手段を確認する。FAULT後の自動再Enableは行わない。

## 5. 位置偏差への依存とWDTを確認する（段階C）

位置偏差への依存は `experiment: position` にし、各区間に
`target_offset_counts: [正負の小さな値, 0]` を追加する。区間開始時に出力禁止中の
M_POSを読み、T_POSを `M_POS + target_offset_counts` に設定する。
同じT_OUTで目標偏差だけ異なる区間と、T_OUTゼロの区間を比較する。
T_POSの更新は各区間の最初だけで、通常の位置増分書き込みとは異なる。
終了時に保存していたT_POSも出力禁止中に復元する。

WDTは `experiment: watchdog` とし、入力列を次のように**非ゼロの一区間だけ**にする。
watchdog_wait_sはWDT期限より長くし、試験時間上限には区間と待ち時間の両方を含める。

```yaml
experiment: watchdog
watchdog_wait_s: 1.0
sequence:
  - {duration_s: 0.5, t_out: [10, 0]}
```

区間終了後は出力とWDTの書き込みを止め、T_OUT/M_OUT/Enableだけを読む。
期限後にM_OUT両輪ゼロを観測できたかを `output_watchdog_result` に残す。
ゼロを観測できなければFAULTとして復旧する。ただし、M_OUTがゼロにならないことだけで
実際に回転が止まらないとは断定できず、その場合も基板の作用を確認する。
**この結果は計算出力の観測であり、物理停止の確認ではない。** エンコーダと実機の停止を
別に確認する。記録の `physical_stop_verified` は常にfalseであり、実験者が結果を評価する。
再始動条件は別起動で確認し、試験中にWDT期限切れから自動復帰させない。

## 6. 正常終了と異常終了後の復旧

終了時は出力禁止の読み戻しに成功した場合だけ、保存したPゲイン・パンチ・上限・T_POSを
復元する。T_OUTは元が非ゼロでもゼロとし、Enableも元に戻さない。
出力禁止と両輪ゼロを確認できなければWDTを無効にせず、未復旧項目をログに残す。
通信エラーの要求を盲目的に再送しない。

異常終了後は、既存のI2Cサービスを使い、スナップショットを指定して復旧専用で起動できる。
この場合は入力列や出力上限を使わず、startサービスも作らない。

```bash
roslaunch harada-tsubakino motor-output-identification.launch group:=pi1 restore_snapshot:=/home/pi/.ros/pi1-output-probe-001.json
```

車両名、レジスタ集合、システム名・ファームウェア版を確認してから保存設定を適用する。
既存I2Cサービスが失われている場合は、自動で新サーバーを起動せず、基板初期化の影響を
確認して復旧する。現行の通常位置制御ノードはT_OUTをゼロにする初期化を持たない。
**通常Launchを再起動するだけでは復旧したとみなさない。**

PCで記録を閉じ、保存先を確認する。

```matlab
recorder.close();
disp(recorder.logFile)
```

## 記録される項目と確認範囲

既存の6トピックで記録できる。試験中の追加I2C読取は、この専用ノードだけが行う。

| イベント | 主な内容 |
| --- | --- |
| `output_probe_state` | READY/RUNNING/FAULT等、理由、区間番号 |
| `output_configuration` | 元設定と保存先、試験上限、設定・符号確認のサービス履歴 |
| `output_command` | 指令連番、区間、要求・送信試行値、目標偏差、予定/実時刻、操作完了の成否 |
| `output_readback` | T_OUT/M_OUTの生値と符号付き値、Enable、各読取の時刻・成否 |
| `output_watchdog_wait/result` | WDT更新停止、最後の更新時刻、期限後の計算出力の観測 |
| `output_restore` | 出力禁止・両輪ゼロ化の成否、未復旧項目 |

`output_command.update_completed=false` のとき、送信試行値が両輪へ適用されたと扱わない。
左右の操作と各レジスタ内のbyte書き込みは非原子的であり、実適用時刻はサービス往復の範囲で
しか分からない。異なる時刻のT_OUT/M_OUT/エンコーダを同時測定と扱わない。
M_POSは既存のwheelイベントとPiの単調時計・boot_idで照合する。
メタデータ `probe_source_sha256` は新規2モジュールのハッシュを含む。

ログのエラー・キュー欠落を検出した場合も試験を中止するが、PCでの保存の成功までは保証しない。
解析前にはイベント/バッチ連番の欠落と、開始・復旧結果が記録されているか確認する。

オフライン検証は `python3 -m unittest discover -s tests -v` で実行できる。
WindowsのPython 3での偽サービス試験は実施済み。Pi上のPython 2、ROS、catkin、I2C、
実機停止・回転方向・T_OUTの意味は別途確認する。

参照したメーカーI2C実装:
[サービス](https://github.com/vstoneofficial/lightrover_ros/blob/main/scripts/i2c_controller.py)、
[byte単位の送受信](https://github.com/vstoneofficial/lightrover_ros/blob/main/scripts/vs_wrc201_i2c.py)。
2026-10-23取得時のGit blobは、それぞれ `0980e7286469af789f98069a08161a24c40cdca8`、
`8707db09d416b561627f0722a09e1c5b96793609`。実機側に導入された版も確認する。
