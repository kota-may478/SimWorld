# 提案法の整理（ISO-feasible × safety preference）

このメモは、共同研究者フィードバックと 2026-09-07 の文献調査（Exa、academic paper search / Crossref、主要論文の全文）を踏まえて、提案法を論文用に固定する。言語射影の数値評価とベースライン一覧は `language_method_novelty.md`。フル Obsidian `@survey` ではない。

## 1. 一文

提案法は、**ISO-feasible なプラントパレート \(P\) をオフラインで用意し、実行時は LLM が \(P\) を 3×3（絶対）または 5×5（相対）で索引する**ことで、規格安全を構成的に保ったまま safety preference を調整する。LLM は \(v_{\max}\) も \(d_{\min}\) も連続重み \(\lambda\) も出さない。

## 2. 二層の安全

| 層 | 名前（論文での呼び方） | 何を保証／何を測るか | いまの実装 |
|---|---|---|---|
| 1 | **ISO-feasible safety** | 動いているあいだ \(\mathrm{SI}=S/S_p \ge 1\)。衝突回避の規格の床 | \(\theta\) は ISO 実行可能な非劣解 \(P\) からしか選ばない |
| 2 | **Intended safety**（safety preference） | 作業者が望んだ「遅さ／遠さ」の点に乗ったか | 発話 → カタログ／\((\alpha,\beta)\) → \(P\) 上の \(\theta^*\) |

層2は層1のうえでしか成立しない。床のない選好（連続 \(\lambda\)、直接 \(\theta\) 回帰）は意図に近くても規格を破れるので、同じ概念ではない。

「Pareto が安全を保証する」は層1だけに使う。LLM が言い間違えても層1は保たれる。層2は落ちる。評価もこの二つに分ける（§8）。

## 3. オンラインの流れ

足場上の資材搬送で、作業者がロボットに声をかける。

1. 発話を絶対（全体スタイル）、相対（いまの設定の微調整）、リセット、無関係に分ける。相対は「もう少し / a bit / かなり / much」などの程度語があるときだけ。程度語のない「ゆっくり / go slow / 減速」は絶対（safe_slow）に固定し、kind の取り違えを防ぐ。
2. 絶対なら効率–安全 \(\alpha\) と、安全の内側の距離–遅さ \(\beta\) を 3 段階に取り、3×3 セルへ載せる（\(\alpha=0\) は \(\beta\) が潰れるので命名は 7 セル）。
3. 相対なら方向（slower / faster / farther / safer）と強さ（a bit = 異なる \(\theta\) へ 1 hop、much = 2 hop）で、**その軸上の次の異なる \(\theta\)** へ動く。同じ \(\theta\) のマスは飛ばす。\(\alpha=0\) の行が 1 点に潰れているときは、先に \(\alpha\) を安全側へ 1 段開いてから同じ規則を続ける。
4. 発話→ラベルは **規則が先**（相対の程度語＋軸、絶対のコマンド語）。残った文は選好発話ゲート（語彙、だめなら選好/雑談プロトタイプの対比）を通してから MiniLM 最近傍。ゲート落ちと類似度しきい値以下は unchanged。LLM は比較用であり、提案の索引器ではない。
5. \(\theta^*\) は \(\lambda(\alpha,\beta)=(1-\alpha,\;\alpha\beta,\;\alpha(1-\beta))\) の加重和で、正規化した \((\mathrm{TT},\,v_{\max},\,-S_{\min})\) 上の点を \(P\) から取る。
6. 得た \(\theta=(v_{\max},d_{\min})\) で足場上を走る。\(S < S_p(v)\) または \(S < d_{\min}\) なら停止し、人間は退避点へ行く。

通路は \(\theta\) も SSM も使わない。\(\mathrm{TT}\) は足場滞在（待ちを含む）。

## 4. パレートの目的：ISO の床と、その上の余裕

制約は実行可能性だけである。完了かつ keep-out 有効中に \(S > S_p\)。

\[
\min \mathrm{TT},\quad \min v_{\max},\quad \max S_{\min}
\]

ISO/TS 15066 の SSM は、疼痛や安心ではなく **当たる前に止まれるか** の運動学である（\(S_p = S_h+S_r+S_s+C+\cdots\)、未計測なら \(v_h=1.6\,\mathrm{m/s}\)、\(T_s\) は速度と荷重の関数）。\(\mathrm{SI}\ge 1\) は二値の床であって、「作業者が十分安心」ではない。PFL（接触時の力・圧力）は別モードで、本オラクルは使わない。

追加の \(\min v_{\max}\) と \(\max S_{\min}\) は、同じ実行可能集合の中で **より遅く／より遠く** という余裕を張る。\(d_{\min}\) は速度に依存しない測距キープアウトで、\(S_p(v)\) とは別チャネルである。HRI では速度と距離が perceived / psychological safety の代表的なロボット側因子である（Story et al. 2022; Akalin et al. 2023; Rubagotti et al. 2022）。ただし本実装は質問紙も生理指標も無いので、論文では心理的安全性の**測定**とは書かない。書くなら *preference-level extra margin along the axes associated with perceived safety*。

荷重（payload）は未モデルである。`spot_loaded` はブール、`truck_load_s` は積み込み待ち、\(T_s\) は定数。定格荷重は hard constraint、ISO の \(T_s(\mathrm{load})\) は物理チャネル、\(\gamma\) で \(v_{\max}\gamma\) と \(S_{\min}/\gamma\) を歪める案は選好の代理にすぎない。今回は limitation。

## 5. 従来研究：部品はある、交差は無い

`ISO/TS 15066 × Pareto × LLM × 3×3/5×5 索引` の論文は、2026-09-07 時点で見つからない。言語で安全寄りにする型、ISO を守る制御、MORL の選好ベクトルはそれぞれある。

| 論文 | ISO を床にするか | 選好を調整するか | LLM か | 計算された ISO パレートの索引か |
|---|---|---|---|---|
| **提案法** | \(P\) からしか選ばない | \(P\) 上で \((\alpha,\beta)\) | ラベルのみ | ある（3×3 / 5×5） |
| Text2Interaction (Thumm et al., CoRL 2024; arXiv:2408.06105) | 証明付き制御が ISO 10218 / TS 15066 に従う | LLM が \(\xi\) と運動選好コード | ある | **ない**。beginner / intermediate / expert は著者定義 |
| Sethuraman et al., arXiv:2603.17510 | なし（MORL 報酬の重み） | LLM が連続 \(\lambda\) | ある | ない |
| Hey Robot (Martinez-Baselga et al., arXiv:2409.13393) | MPC 制約で衝突回避 | LLM がコスト関数コード | ある | ない |
| LaMPC-CBF (Song et al., IEEE Access 2026) | CBF | 言語の safety intent を CBF パラメータへ | ある | ない。LLM が数値を出す |
| PRO-MIND (Lagomarsino et al., arXiv:2409.06864) | ISO 15066 を議論しゾーン適応 | 注意・ストレス | ない | ない |
| Fujii & Pham, arXiv:2306.05197 | ISO SSM を時間最適で保証 | 言語なし。最速のみ | ない | ない |
| Pupa & Secchi, ICRA 2024 | ISO 15066 を MPC で守る | 言語なし | ない | ない |
| Tian & Zou, JIC 2025 | 建設 HRC | 人間選好で行動スコア学習 | 実行時のパレート索引ではない | ない |
| PREDILECT (Holk et al., HRI 2024) | ソーシャルフォース | 言語で報酬学習 | オフライン | ない |
| Cosner et al., arXiv:2112.08516 | CBF フィルタ | ペア選好で CBF パラメータ学習 | 言語ではない | ない |
| IROSA / Knauer et al., arXiv:2603.03897 | 構成的ではない | 速度を何 % 変えるツール | ある | ない |

**最接近は Text2Interaction** である。彼らは

\[
S_{\mathrm{user}} = S_{\mathrm{preference}} \land S_{\mathrm{feasible}}
\]

と書き、ISO に従う安全制御の上で制御選好 \(\xi\) を離散化している。足りないのは、(1) 床の集合が計算された ISO-feasible plant Pareto ではない、(2) 相対 5×5 がない、(3) プラントがマニピュレータのシールドであり足場 SSM の \((v_{\max},d_{\min})\) ではない、(4) intended の主評価が Likert で、指定 \(\theta\) への一致ではない。

## 6. “Intended safety” という語

HRC / ISO 15066 で **intended safety** は定着した術語ではない。論文で定義する。既存の言い換えは次である。

- Text2Interaction: \(S_{\mathrm{preference}} \mid S_{\mathrm{feasible}}\)（同じ論理。床の作り方が違う）
- LaMPC-CBF: language-expressed *safety intent* → CBF パラメータ（床は CBF。LLM が数値を出す）
- PRO-MIND: physical safety vs *cognitive-grounded safety*（言語もパレートもない）
- Pandey et al., arXiv:2507.06700: absolute vs perceived physical safety（観察。索引法はない）
- Story et al., IJSR 2022: ISO 封筒の内側でも心理指標が変わる（実験事実）

新規なのは概念名ではなく、**ISO-feasible 集合の上での intended safety を、パレート索引として実装したこと**である。「intended safety という概念を初めて提案した」とは書かない。

## 7. 主張してよいこと / いけないこと

してよい:

- ISO は衝突回避の下限。同じ実行可能集合に効率寄りと余裕寄りがある。
- 作業者の「ゆっくり」「離れて」は、その余裕軸への指示である。
- LLM を \(P\) の索引に閉じると、層1は構成的に保たれる。
- この交差（ISO 実行可能パレート × 言語格子）は、調査した範囲では先行が無い。
- Text2Interaction と同じ二層だが、床と索引が違う。

いけない:

- 前線上の点が心理的安全性を保証する。
- 言語でロボット選好にする、またはパレート上で重みを動かす、こと自体が新規。
- 3×3 の 9 点がすべて異なる \(\theta\) になる（\(\alpha=0\) は崩壊。5×5 でも一意 \(\theta\) は 12/25。run `20260904203058`）。
- 荷重・定格を考慮している。
- 「safety guaranteed」で層1と層2を混ぜる。

## 8. 評価も二層

| 層 | 成功条件 | 本リポジトリでの測り方 |
|---|---|---|
| ISO-feasible | 選ばれた \(\theta\) が常に \(P\) 上 | 構成。直接 \(\theta\) / 連続 \(\lambda\) ベースラインは外れ得る |
| Intended | 発話が意図したセル／相対ステップに乗る | `fronts/gold_language.py` の strict と \(\theta\) 一致 |

「速めに動いて」で効率側の点が選ばれることは層2の成功である。絶対の「ゆっくり」を相対 `a_bit_slower` に倒す誤りは、層1は保ち層2を落とす典型である。

ベースラインの対応（詳細と数値は `language_method_novelty.md`）:

- **proposed**: 規則（相対＋コマンド語）→ MiniLM 言い換え。LLM は索引器に使わない。
- keyword / embed_pure: 規則のみ、埋め込みのみ
- discrete 3-mode: Text2Interaction 風。相対 5×5 を持たない
- speed_scale: IROSA 風。% 変更で \(P\) に戻らない
- qwen_direct_theta / qwen_lambda: Sethuraman / 直接回帰風。層1が構成的でない
- proposed: カタログ → 5×5 スナップ → 加重和

## 9. オフライン探索について

パレートは実行ループの外で計算する。以前検討した Safe BO は比較用の 1 点探索に残し、前線 \(P\) の主手法は格子掃引と NSGA-II である。オンラインは最近傍／加重和の索引だけで、最適化は走らせない。

## 10. 調査の範囲と日付

- 2026-09-04: 言語→パレートの初回調査（`language_method_novelty.md`）
- 2026-09-07: ISO 背景、perceived safety、intended safety、ISO×Pareto×LLM×格子の再調査（Exa、Crossref、Text2Interaction / PRO-MIND / Hey Robot / LaMPC-CBF 等の HTML）
- Semantic Scholar 横断は 2026-09-07 にタイムアウトが多かった。交差「0 件」は 09-04 の SS クエリと 09-07 の Exa / Crossref に基づく
