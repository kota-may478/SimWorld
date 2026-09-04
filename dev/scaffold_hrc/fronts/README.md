# Front discovery (`fronts/`)

オラクル本体（`oracle/`）は変えません。このパッケージは \(\theta=(d_{\min},v_{\max})\) を渡して \((J_{\mathrm{eff}}, J_{\mathrm{safe}})\) を集め、非劣解を前線とします。

\(T_{\mathrm{ref}}\) は `REF_THETA = (0.35 m, 1.0 m/s)` の makespan です。TT と Jsafe は 1 を超えてよい。

## 手法の要点

### Grid（格子掃引）

パラメータ箱を等間隔に切る。数学的には \(\Theta\) 上の直積格子。抜け漏れが少なく、決定空間の図が最も均一に埋まる。工学的には「まず全領域を見る」ベースライン。最適性の保証は格子幅まで。コストは \(n_d \times n_v\)。

### LHS（Latin hypercube）

各軸を \(n\) 区間に割り、各区間からちょうど 1 点を取る層化乱数。格子より少ない点数で周辺まで届きやすい。空間充填サンプリングの定番。一様乱数より成層が良く、格子の「格子線バイアス」を避ける。

### NSGA-II

多目的 GA。非劣ソート（ランク）と混雑距離で「良い前線」と「前線上のばらけ」を同時に保つ。交叉は SBX、突然変異は polynomial。決定空間を進化で埋めるので、良い前線付近に点が集まる。大域探索だが乱数依存。

### Weighted sum（加重和）

\(J = J_{\mathrm{eff}} - w J_{\mathrm{safe}}\) をいくつかの \(w\) で最大化（多スタート山登り）。凸な前線なら加重和で端点が取れる。凹な前線は取れない（スカラー化の限界）。実装は単純で、ペナルティ \(w_3\) と同じ形。

### ε-constraint

各 \(\varepsilon\) について \(J_{\mathrm{safe}} \le \varepsilon\) のもとで \(J_{\mathrm{eff}}\) を最大にする。凹な前線も追える。\(\varepsilon\) の刻みが前線の解像度になる。実行不能な \(\varepsilon\) では点が減る。

### Safe BO（Safe UCB）

\(J_{\mathrm{eff}}\) と \(J_{\mathrm{safe}}\) にそれぞれ RBF GP。候補格子のうち上側信頼 \( \mu_{\mathrm{safe}}+\beta\sigma \le d_{\lim} \) だけをクエリし、その中で \( \mu_{\mathrm{eff}}+\beta\sigma \) 最大を取る（Sui / Berkenkamp 系の SafeOpt の簡易版）。既知の安全シードから外へ広げる。事故側を抑えたいときの工学的動機が論文の Safe BO に近い。逐次なので点数は他より少ないが、最後に予測安全集合を追加評価して図を厚くする。

## 実行

    MPLBACKEND=Agg python dev/scaffold_hrc/fronts/run_fronts.py

`--quick` はテスト用の少点数。既定は密な予算（格子 16×16、LHS 200 など）。

出力（`out/YYYYMMDDHHMMSS/`）:

    grid/theta.png
    grid/objectives.png
    lhs/...
    nsga2/...
    weighted_sum/...
    epsilon_constraint/...
    safe_bo/...
    comparison_theta.png
    comparison_objectives.png
    fronts.json

各 `*/theta.png` は決定空間の全サンプル（色 = Jeff）と非劣解（星）。`*/objectives.png` は目的空間の全サンプルとパレート前線。
