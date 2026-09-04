# scaffold_hrc

ICRA 2027 向け **足場 HRC** の Stage 1 プロトタイプです。いま動くのは **Unreal Engine なしの運動学オラクル** です。1 配送のナビ検証ではなく、**1F から 3F まで布板をくみ上げる** シミュレーションです。

---

## 1. 何を検証しているか

Spot が資材置き場から各階の荷下ろし点へ運び、同じ階の Humanoid が受け取ってソケットへ置く。その階が揃ってから Humanoid が次の階へ上がり、その後で Spot もその階へ入れる。

- 両方とも **1F（地面）から開始**。Humanoid は最初から 2F にいません。
- 通路（置き場〜階段入口）は **常に 1.0 m/s、距離制約なし**
- **階段とデッキ**では \(\theta = (d_{\min}, v_{\max})\) が効く。人が荷下ろし点の近くにいると Spot は \(d_{\min}\) 以内へこれ以上近づかない（待ち）。人が道を空けると荷を置ける。
- 発話の好み \(\alpha \in [0,1]\) は前線上の 1 点を選ぶ
- \(J_{\mathrm{safe}}\) は Safe BO の硬制約ではなく、近接時間を無次元化した **ペナルティ**
- \(J_{\mathrm{eff}}\) の TT は \(\tilde{T}=T/T_{\mathrm{ref}}\) で無次元化し、TCR と同じ次元にする

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
| `run.json` | 掃引、非劣解、TCR / TT / Jeff / Jsafe / J |
| `pareto_theta.png` | 決定空間 \((d_{\min}, v_{\max})\) の設計曲線 |
| `pareto_objectives.png` | 目的空間 \((J_{\mathrm{safe}}, J_{\mathrm{eff}})\) |
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

1. **Spot** は置き場で荷を積み、今の階の荷下ろし点 \((2.0, 1.2, z_F)\) へ行く。荷下ろし後は置き場へ戻る。行き先は階段の中心を経由する。
2. **Humanoid** は今の階の次ソケット付近で待つ。荷がデッキに置かれたら荷下ろし点へ歩き、受け取ってソケットへ運び、`erect_s`（既定 1.5 s のスナップ）で設置する。
3. Spot がデッキ上で \(d_{\min}\) 以内にこれ以上近づこうとすると **待つ**。その間 Humanoid はスタンドオフ \(x=6\,\mathrm{m}\) へ一時退避し、荷を置ける隙間を空ける。
4. その階のソケットが全部埋まったら、Humanoid が階段で次の階へ上がる。Humanoid が次階に着いてから `spot_max_floor` が上がり、**Spot もその階へ行ける**。
5. 3F の最後の布板が埋まれば完了。

通路上では \(\theta\) を使わない（1.0 m/s）。足場上では \(v_{\max}\)。

### 3.3 目的関数（`oracle/objectives.py`）

論文 (2) は \(J_{\mathrm{eff}}=w_1\mathrm{TCR}-w_2\mathrm{TT}\) だが、TT が秒のままだと TCR と次元が合わない。\(T_{\mathrm{ref}}\) は **参照設定** \(\theta=(0.35\,\mathrm{m},\,1.0\,\mathrm{m/s})\) の makespan で、timeout ではない。1 より大きくてよい。

\[
\mathrm{TCR}=\frac{N_{\mathrm{filled}}}{N_{\mathrm{sockets}}},\quad
\mathrm{TT}=T/T_{\mathrm{ref}},\quad
J_{\mathrm{eff}}=w_1\,\mathrm{TCR}-w_2\,\mathrm{TT}
\]

\(J_{\mathrm{safe}}=T_{\mathrm{viol}}/T_{\mathrm{ref}}\) も同様に 1 を超えてよい（スタックすると大きくなる）。最大化するのは

\[
J=J_{\mathrm{eff}}-w_3\,J_{\mathrm{safe}}\qquad (w_3>0)
\]

\(J_{\mathrm{safe}}\) は硬制約ではなくペナルティ。\(T_{\mathrm{viol}}\) は足場上で離隔 \(< d_{\mathrm{safe}}=1.0\,\mathrm{m}\) だった時間。

### 3.4 代表 \(\alpha\) と \(\Pi\)

1. LLM の数字は使わない。\(\alpha\) が前線の添字を選ぶ。
2. 単調な設計曲線は `run_oracle.py` のデモ用。**精緻な前線**は `fronts/` を使う。

### 3.5 パレート前線の探索（`fronts/`）

オラクルは触らず、手法ごとに決定空間・目的空間の図を出します。要点は `fronts/README.md`。

    MPLBACKEND=Agg python dev/scaffold_hrc/fronts/run_fronts.py

| 手法 | 何をしているか |
|------|----------------|
| Grid | \(\Theta\) の等間隔格子。領域全体を均一に見る |
| LHS | 各軸を層化した乱数。格子より少ない点で箱を覆う |
| NSGA-II | 非劣ソート＋混雑距離の多目的 GA |
| Weighted sum | \(J=J_{\mathrm{eff}}-w J_{\mathrm{safe}}\) を複数 \(w\) で山登り |
| ε-constraint | \(J_{\mathrm{safe}}\le\varepsilon\) のもと Jeff 最大 |
| Safe BO | 2 GP の Safe UCB。予測安全集合だけクエリ |

---

## 4. ファイル

| パス | 役割 |
|------|------|
| `scene/geometry.py` | 寸法 |
| `scene/scaffold_grammar.py` | モジュールとソケット |
| `wbs/clock.py` | Stage-1 WBS |
| `constraints/pareto.py` | 設計前線、\(\Pi\)、非劣解 |
| `oracle/simulate.py` | 3F くみ上げオラクル |
| `oracle/objectives.py` | 無次元 Jeff / Jsafe / ペナルティ J |
| `fronts/` | パレート前線（手法ごとに `theta.png` / `objectives.png`） |
| `viz.py` | PNG / CSV |
| `paths.py` | `out/YYYYMMDDHHMMSS` |
| `run_oracle.py` | 掃引 + 代表ラン |
| `test_*.py` | UE なしテスト |

---

## English (short)

Jeff = w1 TCR − w2 (T / T_ref) with T_ref = makespan of (0.35 m, 1.0 m/s). TT and Jsafe may exceed 1. J = Jeff − w3 Jsafe is maximized. Pareto search lives in `fronts/` (grid, LHS, NSGA-II, weighted sum, ε-constraint, Safe UCB BO). Tests: `python -m unittest discover -s dev/scaffold_hrc -p 'test_*.py' -v`.
