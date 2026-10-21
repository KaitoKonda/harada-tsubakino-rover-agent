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
- インポート先をローカルのPythonパッケージに変更する。I2Cサービス型はメーカー版を使う。
- 未使用のPID制御なども元コードを保持する。今回その動作変更・不具合修正は行わない。
- 計測基盤とPC記録ツールはこのリポジトリで追加したもの。

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
