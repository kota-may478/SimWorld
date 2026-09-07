# scaffold_hrc

ICRA 2027 向け **足場 HRC** の Stage 1 プロトタイプです。いま動くのは **Unreal Engine なしの運動学オラクル** です。1 配送のナビ検証ではなく、**1F から 3F まで布板をくみ上げる** シミュレーションです。

---

## 1. 何を検証しているか

Spot が資材置き場でアームに荷を受け取り（所要時間あり）、各階のデッキに置いて（所要時間あり）、同じ階の Humanoid が手でソケットへ取り付ける。その階が揃ってから Humanoid が次の階へ上がり、その後で Spot もその階へ入れる。

- 両方とも **1F（地面）から開始**。Humanoid は最初から 2F にいません。
- 通路（置き場〜階段入口）は **常に 1.0 m/s、距離制約なし**
- **階段とデッキ**では \(\theta = (v_{\max}, d_{\min})\) が効く。\(v_{\max}\) は足場上の速度指令。階段ホップは \(v_{\max}\) に \(0.5\) をかけて進む。\(d_{\min}\) は速度によらない測距キープアウト。Spot は \(S < S_p^{\mathrm{ISO}}(v)\) または \(S < d_{\min}\) なら停止する。そのとき Humanoid はその階の退避点 \((9.0, 1.2)\,\mathrm{m}\) へ行く。着いたあと、Spot は制約なしで荷下ろしして足場を出る。
- 発話の好み \(\alpha \in [0,1]\) は **ISO 実行可能なパレート前線** \(P\) 上の 1 点を選ぶ（LLM は後段。いまは \(\alpha\) を直接渡す）
- 最小化は \(\mathrm{TT}\)（足場滞在秒、停止を含む）と \(v_{\max}\) [m/s]。制約は有効時 \(S > S_p\) かつ \(S > d_{\min}\)。ミッション全体時間は `mission_s`。
- \(\mathrm{SI}_{\min}\ge 1\) は制約。\(T_{\mathrm{viol}}\)（実現 SI<1 の時間）は診断用で目的関数には入れない
- \(d_{\min}\) は測距キープアウト。\(v_{\max}\) は SSM 停止判定前の足場上速度指令。両方とも設計変数。ISO は \(S>S_p(v)\)、加えて \(S>d_{\min}\)

---

## 2. 実行方法（UE 不要）

```bash
conda activate simworld
python -m unittest discover -s dev/scaffold_hrc -p 'test_*.py' -v
MPLBACKEND=Agg python dev/scaffold_hrc/run_oracle.py --alpha 0.8
```

`--sockets-per-floor N` で各階の布板数を減らせます（既定は 10 枚 = 文法の全ソケット）。`--no-constraint` は keep-out 待ちを切った比較用です。

### 出力ディレクトリ

    out / YYYYMMDDHHMMSS /

例: `dev/scaffold_hrc/out/20260903170053/`

| ファイル | 内容 |
|----------|------|
| `run.json` | 掃引、ISO パレート、TCR / TT / T_SSM / SI_min |
| `pareto_theta.png` | 決定空間 \((v_{\max}, d_{\min})\) の ISO 前線 |
| `pareto_objectives.png` | 目的空間 \((T_{\mathrm{SSM}}, \mathrm{TT})\)（両方小さいほど良い） |
| `trajectory_xy.png` | 上から見た軌跡。左の塗り箱が資材置き場 |
| `trajectory_time.png` | \(x(t)\)、\(z(t)\)（1F→3F）、離隔、設置枚数 |
| `trajectory.csv` | 毎 tick |
| `scaffold_modules.json` | 建枠・ソケット・踏面 |

---

## 3. アルゴリズム（step-by-step）

### 3.1 現場の幾何

1. 作業床 \(x \in [0,10]\,\mathrm{m}\)、\(y \in [0,2.4]\,\mathrm{m}\)。階段塔 \(x \in [-1.8,0]\)。
2. 1F は \(z=0\)、2F は \(1.8\,\mathrm{m}\)、3F は \(3.6\,\mathrm{m}\)。
3. 資材置き場は階段のさらに 10 m 手前。
4. 各階 5 ベイ × 2 列 = 10 布板ソケット。オラクルは空ソケットを順に埋める。

### 3.2 3F くみ上げの状態機械（`oracle/simulate.py`）

時間刻み \(\Delta t = 0.1\,\mathrm{s}\)。`spot_max_floor` は最初 1。

1. **Spot** は置き場でトラック作業者からアームへ荷を受け取り（`truck_load_s`、既定 8 s）、今の階の荷下ろし点 \((2.0, 1.2, z_F)\) へ行き、アームで地面に置く（`drop_place_s`、既定 8 s）。荷下ろし後は置き場へ戻る。行き先は階段の中心を経由する。階段では指令の \(0.5\) 倍で進む。
2. **Humanoid** は今の階の次ソケット付近で待つ。Spot が足場にいるあいだは荷を取りに行かない。Spot が足場を出たあと荷下ろし点へ歩き、受け取ってソケットへ運び、`erect_s`（既定 30 s の手作業）で設置する。Humanoid はトラックへは行かない。
3. Spot がデッキ上で \(S_p^{\mathrm{ISO}}(v)\) または \(d_{\min}\) 以内にこれ以上近づこうとすると **待つ**。Humanoid はその階の退避点 \((9.0, 1.2)\,\mathrm{m}\) へ移動する。着いたら制約を外し、Spot は荷を置いて足場を出る。
4. その階のソケットが全部埋まったら、Humanoid が階段で次の階へ上がる。Humanoid が次階に着いてから `spot_max_floor` が上がり、**Spot もその階へ行ける**。
5. 3F の最後の布板が埋まれば完了。

通路上では \(\theta\) を使わない（1.0 m/s）。足場上では \(v_{\max}\)。

### 3.3 目的関数（`oracle/objectives.py`）

3 目的。無次元化しない。ミッション全体の makespan は `mission_s`。

\[
\min \mathrm{TT}=T_{\mathrm{scaffold}}\ \mathrm{[s]},\quad
\min v_{\max}\ \mathrm{[m/s]},\quad
\max S_{\min}\ \mathrm{[m]}
\]

\(T_{\mathrm{scaffold}}\) は Spot が足場空間にいた時間（停止・退避待ちを含む）。通路は入れない。\(S_{\min}\) はキープアウトが有効で Spot が動いているあいだの実現最小近接（指令 \(d_{\min}\) ではない）。制約は、有効なあいだ完了かつ \(S > S_p\)（ISO の \(\mathrm{SI}=S/S_p\ge 1\) と同じ）。退避後の制約オフ区間は見ない。

足場上では linearized ISO/TS 15066 の \(S_p(v)\)。コントローラは \(S < S_p\) または \(S < d_{\min}\) で停止する。通路は \(\theta\) も SSM も使わない。

### 3.4 代表 \((\alpha,\beta)\) と接地

1. 提案法: ISO 実行可能なパレート \(P\) 上で加重和 \(\lambda(\alpha,\beta)=(1-\alpha,\alpha\beta,\alpha(1-\beta))\)。目盛は 5×5。絶対語は 3×3。言語は Hugging Face（キーワード、任意で埋め込み / 生成）からマスへ。幻覚の数値 \(\theta\) は捨てる。
2. 従来法 B1–B5: 生の数値、箱への clip、キーワード規則、3 モード、シミュレートして却下して保守側へ戻す。さらに SafeOpt（min TT s.t. \(v_{\max}\le d_{\lim}\)）。比較は `fronts/run_grounding.py`。
3. 前線 \(P\) は格子と NSGA-II。要点は `fronts/README.md`。

### 3.5 パレート前線の探索（`fronts/`）

オラクルは触らず、手法ごとに決定空間・目的空間の図を出します。要点は `fronts/README.md`。

    MPLBACKEND=Agg python dev/scaffold_hrc/fronts/run_fronts.py

| 手法 | 何をしているか |
|------|----------------|
| Grid | \(\Theta\) の等間隔格子。領域全体を均一に見る。\(P\) の主手法 |
| NSGA-II | 非劣ソート＋混雑距離の多目的 GA（min TT, min \(v_{\max}\), max \(S_{\min}\)）。\(P\) の主手法 |
| SafeOpt | 保守種から安全集合を拡大。min TT s.t. \(T_{\mathrm{SSM}}\le d_{\lim}\)。比較用の 1 点 |

---

## 4. ファイル

| パス | 役割 |
|------|------|
| `scene/geometry.py` | 寸法 |
| `scene/scaffold_grammar.py` | モジュールとソケット |
| `wbs/clock.py` | Stage-1 WBS |
| `constraints/pareto.py` | 設計前線、\(\Pi\)、非劣解 |
| `oracle/simulate.py` | 3F くみ上げオラクル（SSM + \(d_{\min}\)） |
| `oracle/ssm.py` | ISO \(S_p\) / SI / stop |
| `oracle/objectives.py` | TT, T_SSM, SI_min 制約 |
| `fronts/` | パレート前線と B1–B5 / 提案接地 |
| `fronts/grounding.py` | 提案 \(\mathrm{project}(\alpha,P)\) と従来法 |
| `viz.py` | PNG / CSV |
| `paths.py` | `out/YYYYMMDDHHMMSS` |
| `run_oracle.py` | 掃引 + 代表ラン |
| `test_*.py` | UE なしテスト |

---

## English (short)

Minimize TT (scaffold dwell, s) and v_max (m/s); maximize realized min separation while keep-out is on and Spot is moving (m). Constraint: completed and S > S_p while keep-out is enforced (same as ISO SI = S/S_p >= 1). Preferences map through (alpha, beta) weighted sum on the ISO front (5x5 grid, 3x3 named cells). Language: keyword, Hugging Face embeddings (--embed), or instruct LLM (--llm). Tests: `python -m unittest discover -s dev/scaffold_hrc -p 'test_*.py' -v`.
