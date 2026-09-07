# Front discovery (`fronts/`)

オラクル本体（`oracle/`）は \(\theta=(v_{\max},d_{\min})\) を受けて足場滞在 \(\mathrm{TT}\) [s] と実現最小近接 \(S_{\min}\) [m] を返す。第2目的は \(v_{\max}\) そのもの。このパッケージはサンプルを集め、ISO 実行可能な 3 目的非劣解を前線 \(P\) とする。

制約は完了かつ \(S > S_p\)（有効なあいだ。ISO の \(\mathrm{SI}\ge 1\) と同じ）。コントローラは \(S < S_p^{\mathrm{ISO}}(v)\) または \(S < d_{\min}\) で停止し、人間は退避点へ行く。\(S_{\min}\) は Spot が動いているティックだけ。

提案接地は `grounding.py` の加重和 `proposed(α, β, P)`。絶対語は 3×3、目盛は 5×5。言語は `run_language.py`（Hugging Face）。比較は B1–B5 と SafeOpt（単目的: min TT s.t. \(v_{\max}\le d_{\lim}\)）。

## 手法の要点

### Grid（格子掃引）— 前線 \(P\) の主手法

パラメータ箱を等間隔に切る。2 変数なら抜け漏れが少なく、決定空間の図が最も均一に埋まる。コストは \(n_v \times n_d\)。

### NSGA-II — 前線 \(P\) の主手法

多目的 GA。非劣ソート（ランク）と混雑距離。交叉は SBX、突然変異は polynomial。目的は min TT [s], min \(v_{\max}\), max \(S_{\min}\) [m]。3 目的の混雑距離を使う。

### SafeOpt — 比較ベースライン（前線法ではない）

TT（符号反転）と T_SSM にそれぞれ RBF GP。保守的な種から、予測上側信頼 \(\mu_{\mathrm{SSM}}+\beta\sigma \le d_{\lim}\) の点だけをクエリする。返すのは incumbents の **1 点**（その TT と T_SSM）。ISO の \(\mathrm{SI}_{\min}\ge 1\) はプラント側。

LHS / 加重和 / ε-制約モジュールは残してあるが、既定の `run_fronts.py` では走らせない。

## 実行

    MPLBACKEND=Agg python dev/scaffold_hrc/fronts/run_fronts.py
    MPLBACKEND=Agg python dev/scaffold_hrc/fronts/run_grounding.py --quick --alpha 0.8 --beta 0.5
    python dev/scaffold_hrc/fronts/run_language.py --text "ゆっくり動いて"

`--quick` はテスト用の少点数。

出力（`out/YYYYMMDDHHMMSS/`）:

    grid/theta.png
    grid/objectives.png
    nsga2/...
    safe_bo/...   (incumbent を samples.json に記録)
    comparison_theta.png
    comparison_objectives.png
    ab_table.png
    fronts.json

各 `*/theta.png` は決定空間 \((v_{\max},d_{\min})\) の全サンプル（色 = TT）と非劣解（星）。`*/objectives.png` は 3 目的の pairwise と 3D。接続線は描かない（2 次元射影の偽前線を避ける）。
