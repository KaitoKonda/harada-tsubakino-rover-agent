# 移植元と変更範囲

移植元: https://github.com/vstoneofficial/lightrover_ros （vstoneofficial）。
公開版を2026-10-21に取得。以下はcommit SHAではなく各ファイルの **Git blob SHA**。
元パッケージの `package.xml` はライセンスをMITと宣言している。
元ファイルのコメント・出典を保持し、回帰テスト用の原文を `tests/upstream` に収録する。

| 元ファイル | Git blob SHA | 配置先 |
| --- | --- | --- |
| `scripts/pos_controller.py` | `618bb67db1044e75fbde482903250f14aa94adb9` | `src/rover_drive_trace/controller.py` |
| `scripts/odometry.py` | `8fd864156607c2a2a87455a27bea5b210ef1f1a7` | `src/rover_drive_trace/odometry.py` |
| `scripts/vs_wrc201_motor.py` | `599c2eedeec64bc68008ad2aedce1f45e3dfe8d3` | `src/rover_drive_trace/motor.py` |

配置先は `harada-tsubakino` 以下。変更内容:

- 制御/オドメトリへ計測点を追加し、既存サービス呼び出しを時間計測付きで呼ぶ。
- モーターライブラリは既存のローカル変数 `elapsed_time` を2箇所で属性にコピーするだけ。
  時刻取得回数、加減速、整数化・小数繰越の計算は変更しない。
- オドメトリの速度計算本体を `_calSpeed()` とし、`calSpeed()` で前後の値を記録する。
- エンコーダ差分は32 bitの巡回カウンタとして最小の符号付き差分へ補正する。
  例: `0 -> 0xffffffff` は-1 count、逆は+1 count。補正後に既存の差分閾値判定を行う。
  詳細ログは補正前の `raw_delta_counts` と補正後の `wrapped_delta_counts` を持つ。
  読み取りの不整合や外れ値の除去はこの補正だけでは行わない。
- インポート先をローカルのPythonパッケージに変更する。I2Cサービス型はメーカー版を使う。
- 未使用のPID制御なども元コードを保持する。今回その動作変更・不具合修正は行わない。
- 計測基盤とPC記録ツールはこのリポジトリで追加したもの。
- 計測基盤は `full/minimal/off` を持ち、fullとminimalで同じ時間集計を使う。
  minimalでは毎周期の詳細データのコピー・レコード・イベントキューを省く。
  制御式とI2C要求順は各モードとも維持する。車輪odomの差分には上記の折り返し補正を含む。
- サービスクライアントはROS Melodicの `ServiceProxy` を継承し、既存の
  `_get_service_uri` の前後で検索時間を計測する。検索処理そのものと接続・要求・応答処理は
  ROSの実装へ委ねる。既定は永続接続。`i2c_persistent:=false` で非永続接続へ戻せる。
  永続接続ではROS標準の接続再利用を使い、
  同じクライアントの呼び出しを直列化する。失敗した要求を再送せず、保存した通信路を破棄して
  次の通常呼び出しで再接続する。ROS本体のコピー・編集は行わない。
  I2Cサーバーは引き続きメーカー版を使う。
- 制御側の車輪odom購読に `queue_size=1` を指定し、処理が遅れたときに
  受信バッチ内の古いメッセージを間引く。受信コールバック内で制御する構成と
  車輪odomの生成・配信処理は維持する。TCP受信バッファを含む滞留の解消は実機で確認する。
- 同じ車輪odom購読に `tcp_nodelay=True` を指定し、配信側とのTCPROS接続で
  Nagleアルゴリズムを無効にするよう要求する。制御周期への効果は実機ログで確認する。
  メタデータに `wheel_odom_tcp_nodelay` を記録する。

MIT License (upstream package declaration)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
