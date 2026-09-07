# Language-to-Pareto 手法の新規性と従来法

枠組みの固定（ISO-feasible / intended safety、Text2Interaction との差）は `proposed_method.md`。

このメモは Exa と academic paper search（Semantic Scholar / arXiv）による 2026-09-04 時点の調査である。フル Obsidian @survey ではない。

## 提案法（実装中の核）

1. NSGA-II で得た ISO 実行可能パレート \(P\) 上で、\(\theta=(v_{\max}, d_{\min})\) を選ぶ。
2. 目的は \(\min TT\), \(\min v_{\max}\), \(\max S_{\min}\)（keep-out 中かつ移動中）。
3. 言語の絶対指令は 3×3 セル（\(\alpha=0\) は \(\beta\) 崩壊のため命名は 7 セル）へ射影する。
4. 「もう少し／かなり」は 5×5 上で、その軸の **次の異なる \(\theta\)** へ 1/2 hop（同じ \(\theta\) のマスは飛ばす）。程度語の無い「ゆっくり」は絶対。
5. LLM は \(\theta\) も連続 \(\lambda\) も出さない。カタログのラベル 1 語だけを出し、正規表現で取る。
6. \(\theta^*\) は \(\lambda(\alpha,\beta)=(1-\alpha, \alpha\beta, \alpha(1-\beta))\) の加重和で \(P\) から決まるので、ISO 実行可能性は構成的に保たれる。

## 取れるか（NSGA-II ISO, run `20260904203058`）

- NSGA-II ISO: 356 点。combined ISO: 379 点。
- 5×5 表の相異なる \(\theta\) は **12 / 25**。
- \(\alpha=0\) の 5 セルは同一 \(\theta\)（\(v_{\max}=1.00\), \(d_{\min}=0.78\), \(TT=639.7\)）。
- \(\alpha=0.5\) と \(\alpha=1.0\) では \(\beta\) 方向に 4 点ずつ分かれる。

よって「パレート上の点を \((\alpha,\beta)\) で取る」は可能。ただし格子の 25 点がすべて異なる \(\theta\) になるわけではない。これは加重和とフロント形状の結果であり、言語の 3×3 命名を 7 セルに落とす根拠になる。

## 新規性の判定

**完全に未踏ではない。** 言語 → 多目的の好みベクトル → ナビ／制御、という型は 2024–2026 に複数ある。主張できる差分は次である。

| 近い従来法 | 何をするか | 提案法との差 |
|---|---|---|
| Sethuraman et al., arXiv:2603.17510 | LLM が連続 MORL ベクトル \(\lambda\)（効率・障害物距離・人距離・速度）を出す | 連続 \(\lambda\)。ISO 15066 のプラント \(\theta\) パレートではない。3×3/5×5 量子化なし。VLM+ルールメモリ |
| Text2Interaction, arXiv:2408.06105 | LLM が ISO 10218/15066 安全制御の離散 \(\xi\)（beginner/expert 等）を選ぶ | マニピュレータ。離散モードは 3–5 個で、パレート加重和でも相対 5×5 でもない |
| IROSA / Knauer et al., arXiv:2603.03897 | LLM が speed-modulation ツールを呼び、軌跡速度を何 % 変える | パレートに戻さない。ISO 実行可能性は構成的でない |
| LaMPC-CBF, Song et al. | 言語の安全意図を CBF パラメータにする | オンライン安全制御。SSM プラント最適化ではない |
| Hey Robot! (Martinez-Baselga, ICRA) | LLM が MPC コスト項と重みを書く | コスト生成。量子化グリッドなし |
| PREDILECT, Holk et al., arXiv:2402.15420 | 選好テキストから特徴（速度・人距離）を LLM 抽出して報酬学習 | オフライン選好学習。実行時の 3×3 射影ではない |
| Freeform Preference Learning, Torne et al., arXiv:2606.32027 | 言語軸ごとのペア選好で報酬条件付き方策 | 学習。ISO パレート検索ではない |
| QuickLAP, arXiv:2511.17855 | 言語+身体補正から報酬 \(\theta\) をベイズ更新 | 運転。グリッド射影ではない |
| Glogowski et al., 2021 | SSM 下で速度適応しサイクルタイムを最小化 | **言語インターフェースなし** |

Semantic Scholar で `ISO/TS 15066` × `natural language` × `Pareto` は **0 件**。この交差が論文の空白に近い。

**主張してよい優位性（過大にしない）**

1. **構成的 ISO 実行可能性**: LLM はラベルだけ。\(\theta\) は実行可能パレートから選ぶ。連続 \(\lambda\) や直接 \(\theta\) 回帰はフロントから外れる。
2. **二尺度言語**: 絶対は 3×3（作業者の語彙）、相対は 5×5（「もう少し」）。Sethuraman の連続ベクトルにも Text2Interaction の 3 モードにも、この二段はない。
3. **プラント SSM の \((TT, v_{\max}, S_{\min})\)**: 足場 HRC の keep-out 付き Spot 滞在時間。ナビ MORL やマニピュレータ \(\xi\) とは目的が違う。
4. **1.5B 級ローカル制約デコード**: ツール呼び出しやコード生成より、ラベル集合が小さい。IROSA が主張する「構造化インターフェース」と同じ精神だが、出力は速度%ではなくパレート索引である。

**主張しにくいこと**

- 「LLM で言語をロボット選好にする」自体は新規でない。
- 「多目的パレート上で重みを動かす」自体は MORL / interactive MOO の古典。
- 3×3 の 9 点がすべて異なる \(\theta\) になるわけではない（\(\alpha=0\) 崩壊）。

## 比較に入れる従来法（本リポジトリで同一ゴールド指令）

実装:

1. **keyword** — 正規表現 3×3/相対（従来の音声コマンド表）
2. **embed** — MiniLM 最近傍フレーズ
3. **discrete_3mode** — Text2Interaction 風の efficient/normal/safe のみ（\(\beta=0.5\)）
4. **alpha_1d** — 1 次元 \(\alpha\) のみ
5. **speed_scale** — IROSA 風の相対スピード／距離ステップ（パレートへは後で載せる）
6. **qwen_direct_theta** — Sethuraman/直接回帰風に \(v_{\max},d_{\min}\) を出して最近傍 ISO 点
7. **qwen_lambda** — 連続 \(\lambda\) 加重和（スナップなし MORL ベクトル）
8. **qwen_continuous_ab** — 連続 \((\alpha,\beta)\) を LLM に出させ、3×3 ラベルを経由しない
9. **proposed** — 規則（相対＋コマンド語）→ MiniLM 言い換え。θ は加重和
10. **proposed_qwen** — 段階 LLM ラベル（比較用）

Llama-3.1-8B 同プロンプトは API 依存のため本評価の第一段からは外し、ローカル Qwen と非 LLM を先に揃える。

## ゴールド指令

閉じた分類（`fronts/gold_language.py`）: 3×3 の JP/EN 言い換え、弱/強相対、リセット、OOD、複合。成功は絶対なら 3×3 セル、相対なら 5×5 の 1/2 ステップ方向。複合は許容ラベル集合。

## 評価結果（78 指令、NSGA-II ISO 356 点、Qwen2.5-1.5B-Instruct ローカル）

数値は `out/language_eval/results.json`（カタログ 1 語）と `out/language_eval/results_staged.json`（段階質問 + kind ゲート + 異なる θ hop）。成功はカタログ ID の一致（strict）。相対ラベルは 1/2 hop の向きと強さ。同じ θ のマスは飛ばす。

| 手法 | 全体 strict | 絶対 | 相対 | θ 一致 |
|---|---:|---:|---:|---:|
| **proposed（規則 + 選好ゲート + MiniLM）** | **0.87** | **0.76** | **1.00** | **0.87** |
| keyword 規則のみ | 0.76 | 0.51 | **1.00** | 0.77 |
| proposed Qwen 段階 + kind ゲート | 0.53 | 0.32 | 0.83 | 0.60 |
| proposed Qwen（カタログ 1 語） | 0.63 | 0.68 | 0.62 | 0.73 |
| embed MiniLM 純最近傍 | 0.46 | 0.92 | 0.00 | 0.58 |
| discrete 3-mode | 0.27 | 0.32 | 0.00 | 0.36 |
| alpha 1D | 0.54 | 0.51 | 0.41 | 0.46 |
| speed_scale（IROSA 風） | 0.53 | 0.16 | 0.90 | 0.56 |
| Qwen 直接 θ | 0.01 | 0.00 | 0.00 | 0.04 |
| Qwen 連続 λ | 0.04 | 0.00 | 0.00 | 0.14 |
| Qwen 連続 (α,β) | 0.00 | 0.00 | 0.00 | 0.21 |

読み取り:

- 閉じたカタログへの索引は、1.5B LLM より **規則 + MiniLM** が強い（全体 0.83 vs Qwen 0.53）。提案の NLU はこちら。
- keyword の相対 1.00 は、ゴールド句がコマンド表と同じだから。`much slower` を strong に入れたあと、閉じた相対は規則で足りる。
- MiniLM はキーワードが外れる言い換えを拾う。埋め込みの前に選好ゲート（語彙、だめなら選好/雑談プロトタイプ対比）があり、OOD は 5/5 で unchanged。絶対 0.51 → 0.76、全体 0.87。
- 連続 λ / 直接 θ は 1.5B ではほぼ使えない。カタログ制約の価値はここにある。
- 3-mode は相対 5×5 を表現できない。

Llama-3.1-8B 同プロンプトは未実行（ローカル 1.5B を先に固定）。
