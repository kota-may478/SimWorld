# Carry bone/socket attach — UE setup for site_transport_20m

> **全体の作業順序・E2E 確認**は `SIGHT_PERCEPTION_UE_SETUP.md` の **Phase 9** にも **省略なし**でまとめています。  
> 本ファイルは **Blueprint グラフのピン単位**の詳細手順です。

Python は pickup 後に次を呼びます:

```text
vbp GridEnv_SpotRobot AttachCarryActor site20_carry
vbp GridEnv_SpotRobot DetachCarryActor site20_carry
```

**UE 側に vbp 関数が無い場合**は従来どおり Python の `set_location` 同期にフォールバックします。

---

## 前提

| 項目 | 値 |
|------|-----|
| Robot Pawn | `GridEnv_SpotRobot`（`BP_SpotRobot` / `BP_SpotRobot_Child`） |
| Carry actor | `site20_carry`（`BP_Crate_01a` 等） |
| Socket 名 | **`CarrySocket`**（Python 定数 `CARRY_SOCKET_NAME` と一致必須） |
| 参照 | `SIGHT_PERCEPTION_UE_SETUP.md` Phase 3 方法 B（Skeleton Socket） |

---

## Phase 1: Skeleton に `CarrySocket` を追加

1. **PIE を停止**
2. `BP_SpotRobot_Child`（または Level で使っている SpotDog BP）を開く
3. **Mesh → Skeletal Mesh Asset** を Browse して Skeleton Editor を開く
4. **Skeleton Tree** で背中付近のボーン（例: `spine_03`, `body`, モデル依存）を右クリック → **Add Socket**
5. 名前を **`CarrySocket`** に変更
6. Viewport でソケット位置を調整（目安）:
   - ロボット後方 **20 cm**（`CARRY_FORWARD_CM = -20` 相当）
   - 背中上面 **約 50–70 cm**（クレート pivot が床+88cm 付近になるよう微調整）
7. **Save** して Skeleton Editor を閉じる

---

## Phase 2: `BP_SpotRobot` に vbp 関数を追加

対象: Level PIE で `vbp GridEnv_SpotRobot ...` が届く Pawn Blueprint  
（`Move_Speed` / `Rotate_Angle` と同じ BP）

> **いまの画面の状態**  
> 左 **My Blueprint → Functions** に `AttachCarryActor` があり、中央グラフに紫の入口ノード **Attach Carry Actor** だけがある状態で合っています。  
> 右 **Details** が空で「All results have been filtered」と出る場合は、**検索欄の `Skeletal Mesh` を消す**（× をクリック）と Inputs / Outputs が表示されます。

---

### 2-A. `AttachCarryActor` — Inputs / Outputs の設定

#### A-0. 関数を選択する

1. **あなた**は 左パネル **My Blueprint** の **Functions** 一覧で **`AttachCarryActor`** を **1 回クリック**する。  
2. **UE**は 中央のグラフエディタに **`AttachCarryActor`** の関数グラフを **表示**する。

#### A-1. Details のフィルタを解除する

1. **あなた**は 右パネル **Details** 上部の検索欄に **`Skeletal Mesh`** などが入っていたら、検索欄右端の **×（Clear）** を **クリック**する。  
2. **UE**は **Graph** / **Inputs** / **Outputs** などのセクションを **表示**する。

#### A-2. Input（入力引数）を追加する

1. **あなた**は **Details** パネル内の **Inputs** 行の右にある **＋（プラス）ボタン** を **クリック**する。  
2. **UE**は 新しい入力行（`NewParam` など）を **1 行追加**する。  
3. **あなた**は その行の **Name** 欄を **クリック**し、キーボードで **`CarryActorName`** と **入力**する。  
   - Python が `vbp GridEnv_SpotRobot AttachCarryActor site20_carry` と呼ぶとき、**2 番目の文字列**がこの引数に入ります。名前は **完全一致**が必要です。  
4. **あなた**は 同じ行の **Type** ドロップダウン（デフォルトは Boolean 等）を **クリック**する。  
5. **あなた**は 検索欄に **`string`** と **入力**し、一覧から **`String`** を **選択**する。  
6. **あなた**は **Compile** ボタン（ツールバー左上の緑チェック）を **1 回クリック**する。  
7. **UE**は 中央グラフの紫ノード **Attach Carry Actor** の左側に、**青い `Carry Actor Name` ピン**を **追加**する。

#### A-3. Output（戻り値）を追加する

1. **あなた**は **Details** パネル内の **Outputs** 行の右にある **＋（プラス）ボタン** を **クリック**する。  
2. **UE**は 新しい出力行を **1 行追加**する。  
3. **あなた**は **Name** を **`ReturnValue`** のまま（または **`Return Value`** 表示でも可）にし、**Type** を **`Boolean`** に **設定**する。  
4. **あなた**は 再度 **Compile** を **クリック**する。  
5. **UE**は グラフに **Return Node**（関数の出口）を **自動追加**する（無い場合は次の A-4 で手動追加）。

#### A-4. Return Node が無い場合に追加する

1. **あなた**は グラフの空白所を **右クリック**する。  
2. **あなた**は コンテキストメニューで **`Add Return Node...`**（または **Return Node**）を **選択**する。  
3. **UE**は **`Return Node`** を **配置**する。

#### A-5. 作業用ローカル変数を 2 つ追加する（任意だが推奨）

名前検索ループで使います。

1. **あなた**は **My Blueprint → Variables** の **＋** を **クリック**し、変数 **`FoundCarryActor`** を **追加**する。  
   - **Type**: **`Actor`**（**Object Reference** → **Actor**）  
2. **あなた**は 同様に変数 **`AttachOk`** を **追加**する。  
   - **Type**: **`Boolean`**  
   - **Default Value**: **`false`**  
3. **あなた**は **Compile** を **クリック**する。

---

### 2-A（続）. `AttachCarryActor` — グラフの配線

以下、**白い実行ピン**＝処理の順序、**青いデータピン**＝値の受け渡しです。

#### B-1. ロボットの Mesh コンポーネント参照を置く

1. **あなた**は 左 **Components** 一覧の **`Mesh (CharacterMesh0)`** を **グラフの空白所へドラッグ**する。  
2. **UE**は **`Mesh`** 参照ノード（青い出力ピン付き）を **作成**する。  
   - この **Mesh** が、あとで Carry Actor の **親コンポーネント**になります。

#### B-2. レベル内の全 Actor から名前一致で Carry Actor を探す

1. **あなた**は グラフ空白所を **右クリック**し、検索欄に **`Get All Actors Of Class`** と **入力**して、そのノードを **選択**する。  
2. **あなた**は **`Get All Actors Of Class`** の **Actor Class** ピンを **クリック**し、ドロップダウンで **`Actor`** を **選択**する。  
   - **重要:** ここが **`None` のまま**だと **Out Actors が空**になり、以降の処理が動きません。必ず **`Actor`** に設定してください。  
3. **あなた**は 再度 右クリック → **`For Each Loop`** を **追加**する。  
4. **あなた**は **白い実行線**で  
   **`Attach Carry Actor`（入口）の白ピン** → **`Get All Actors Of Class` の白ピン** → **`For Each Loop` の白い Exec ピン**  
   を **順に接続**する。  
5. **あなた**は **青い線**で  
   **`Get All Actors Of Class` の `Out Actors`** → **`For Each Loop` の `Array`**  
   を **接続**する。

#### B-3. ループ内で Actor 名を比較する

> **よくある誤解:** **`Array Element` ピンを右クリック**しても、メニューには  
> `Promote to Variable` / `Watch This Value` など **ピン操作だけ**が出ます。  
> **`Get Object Name` はここには出ません。** 次の **方法 A** または **方法 B** でノードを追加してください。

##### 方法 A（推奨）: ピンをドラッグしてノードを出す

1. **あなた**は **`For Each Loop` の `Array Element`（青いピン）** を **左クリックしたまま**、グラフの **空白所**へ **ドラッグ**する。  
2. **あなた**は マウスボタンを **離す**。  
3. **UE**は **コンテキストメニュー**（ノード検索付き）を **表示**する。  
4. **あなた**は 検索欄に **`object name`** または **`get name`** と **入力**する。  
5. **あなた**は 一覧から次のいずれかを **選択**する（環境により表記が異なります）:  
   - **`Get Object Name`**（最優先）  
   - **`Get Actor Name`**  
   - **`Get Display Name`**  
6. **UE**は **`Array Element` が自動的に `Get Object Name` の Target（入力）** に **接続**されたノードを **作成**する。

**`Get Object Name` は正しいノードですか？ → はい、正しいです。**

| 確認項目 | 内容 |
|----------|------|
| ノード名 | **`Get Object Name`**（`KismetSystemLibrary::GetObjectName`） |
| 入力 | **Object**（青）← `Array Element`（Actor 参照） |
| 出力 **Return Value** | **String 型**（**マゼンタ / ピンク**のピン） |
| Python との対応 | UnrealCV で `site20_carry` とスポーンした Actor の **内部オブジェクト名**と一致する文字列が返る |

> **ピンの色の見分け（UE 5）**  
> - **青** = Object 参照  
> - **紫** = Name  
> - **マゼンタ（ピンク）** = String  
> - **水色** = Text（`FText`。今回は **使わない**）

##### 方法 B: グラフ空白を右クリックしてから配線する

1. **あなた**は **ピンではなく** グラフの **空白所**を **右クリック**する。  
2. **あなた**は 検索欄に **`Get Object Name`** と **入力**してノードを **配置**する。  
3. **あなた**は **`Array Element` ピン**から **`Get Object Name` の Target（Object）ピン**へ **青い線をドラッグ**して **接続**する。

##### 名前の比較（Equal）と分岐（Branch）

**`Equal (Name)` が検索に出ない理由**

- **`Get Object Name` の戻り値は String** です（スクショの **マゼンタの Return Value**）。  
- そのため **`Equal (Name)` は出ません**（Name 型ピンから検索したときだけ出るノードです）。  
- **`Equal (String)` / `Equal Exactly (String)` が出るのは正常**です。  
- **`Equal (Text)`** は `FText` 用なので **選ばない**でください。

**比較ノードの作り方（推奨）**

1. **あなた**は **`Get Object Name` の `Return Value`（マゼンタ）ピン**を **ドラッグ**して空白所で **離す**。  
2. **あなた**は 検索欄に **`equal`** と **入力**する。  
3. **あなた**は **String** カテゴリの **`Equal Exactly (String)`**（または **`Equal (String)`**）を **選択**する。  
   - メニュー右上の **Context Sensitive** が **ON** だと、String 向けだけに絞られます（推奨）。  
   - **Operators → `Equal`** だけが出る場合もありますが、**String 同士**になるよう配線してください。

**入口 `Carry Actor Name` が Name 型（紫ピン）のとき**

Python は `site20_carry` を **文字列**で渡しますが、BP 入力が **Name** でも問題ありません。比較は次のどちらかです。

| 方法 | 操作 |
|------|------|
| **A（推奨）** | **`Carry Actor Name` ピン**をドラッグ → **`To String (Name)`** / **`Conv_NameToString`** を追加 → その出力を **`Equal (String)`** のもう一方に接続 |
| **B** | A-2 に戻り、入力 **`CarryActorName` の Type を `String` に変更**（Python vbp とも整合しやすい） |

**配線の完成形（String 比較）**

```text
Array Element → Get Object Name → Return Value (String) ─┐
                                                        ├→ Equal Exactly (String) → Branch (Condition)
Carry Actor Name → Conv_NameToString (任意) ────────────┘
```

4. **あなた**は グラフ空白を **右クリック** → **`Branch`** を **追加**する。  
5. **あなた**は **白い線**で **`For Each Loop` の `Loop Body`** → **`Branch` の Exec** を **接続**する。  
6. **あなた**は **青い線**で **`Equal` の戻り値（Boolean）** → **`Branch` の Condition** を **接続**する。

**名前が取れない場合の代替:** `Get Object Name` が無いときは、`Array Element` から **`Get Actor Label`** を試してください（通常は **Object Name** の方が Python スポーン名 `site20_carry` と一致します）。

#### B-4. 一致した Actor を変数に保存する

1. **あなた**は **`Branch` の True** ピンから **`Set FoundCarryActor`** ノードを **追加**する（Variables からドラッグでも可）。  
2. **あなた**は **`Set FoundCarryActor` の値ピン**に **`Array Element`** を **接続**する。

#### B-5. ループ完了後 — Carry Actor が見つかったか分岐する

この節では **2 種類の線**を使います。

| 線の色 | 種類 | 役割 |
|--------|------|------|
| **白** | **実行線**（Exec） | 「いつ処理するか」の順序 |
| **青** | **データ線** | Actor 参照などの値 |
| **赤** | **データ線** | Boolean（true / false） |

**`Is Valid` には白い実行ピンがありません（Pure ノード）。**  
したがって **`Completed` を `Is Valid` に接続する必要はありません。** あなたの理解どおりです。

---

##### B-5 で追加するノード（B-3 の Branch とは別）

| ノード | この節での呼び方 |
|--------|------------------|
| **`For Each Loop`** | 既存（B-2 で配置済み） |
| **`Get FoundCarryActor`** | 変数の **Get** ノード（新規配置） |
| **`Is Valid`** | Object が null でないか調べるノード（新規配置） |
| **`Branch`（2 つ目）** | **B-5 用の Branch**（B-3 ループ内の Branch とは **別ノード**） |
| **`Return Node`** | 関数の出口（既存） |

> **注意:** B-3 にある **ループ内の `Branch`（1 つ目）** と、ここで追加する **`Branch`（2 つ目）** は **別物**です。  
> B-5 の False / True は **必ず 2 つ目の Branch** のピンを使います。

---

##### 使う `Is Valid` はどれか

検索で **`Is Valid`** と入れると多数出ますが、選ぶのは **次の 1 種類だけ**です。

| 選ぶノード | 見た目 |
|------------|--------|
| **`Is Valid`**（Utilities 等） | 入力 **`Input Object`（青）** が **1 つ**、出力 **`Return Value`（赤）** が **1 つ**。**白ピンは無い** |

**選ばない例:** `Is Valid Index` / `Is Valid Timer Handle` / `Is Valid Class` など

---

##### 手順 1 — `Get FoundCarryActor` と `Is Valid` を置く

1. **あなた**は **My Blueprint → Variables** の **`FoundCarryActor`** を **グラフへドラッグ**する。  
2. **あなた**は メニューで **`Get FoundCarryActor`** を **選択**する。  
3. **あなた**は **`Get FoundCarryActor` の青ピン**を **ドラッグ**して空白所で **離す**。  
4. **あなた**は 検索欄に **`is valid`** と **入力**し、**`Input Object` 入力だけの `Is Valid`** を **選択**する。  
5. **UE**は **`Get FoundCarryActor` の青ピン**と **`Is Valid` の `Input Object` 青ピン**を **自動接続**する。

---

##### 手順 2 — 実行線（白）を 1 本だけ引く

| 接続元（主語） | ピン | 接続先（目的語） | ピン |
|----------------|------|------------------|------|
| **`For Each Loop`** | 右側 **`Completed`（白）** | **`Branch`（2 つ目）** | 左側 **`Exec`（白）** |

- **あなた**は **`Completed` の白ピン**から **白線**を引き、**`Branch`（2 つ目）の左 `Exec`（白）** に **接続**する。  
- **`Is Valid` には白ピンが無い**ので、**`Completed` → `Is Valid` の接続は不要**です。

---

##### 手順 3 — データ線（赤）で分岐条件を渡す

| 接続元（主語） | ピン | 接続先（目的語） | ピン |
|----------------|------|------------------|------|
| **`Is Valid`** | **`Return Value`（赤）** | **`Branch`（2 つ目）** | **`Condition`（赤）** |

- **あなた**は **`Is Valid` の赤ピン**から **赤線**を引き、**`Branch`（2 つ目）の `Condition`（赤）** に **接続**する。

**B-5 時点の配線まとめ（図）**

```text
[For Each Loop]
    Completed (白) ──────────────────────→ [Branch 2つ目] Exec (白)
                                                    ↑ Condition (赤)
[Get FoundCarryActor] (青) → [Is Valid] Return Value (赤) ─┘
         (青) → [Is Valid] Input Object (青)
```

---

##### 手順 4 — False 側（Actor が見つからなかったとき）

**接続する Branch:** **B-5 で追加した 2 つ目の `Branch`**（ループ内 1 つ目ではない）。

| 接続元（主語） | ピン | 接続先（目的語） | ピン |
|----------------|------|------------------|------|
| **`Branch`（2 つ目）** | 右側 **`False`（白）** | **`Return Node`** | 左側 **白い実行入力** |

- **あなた**は **`Branch`（2 つ目）の `False` 白ピン**から **白線**を引き、**`Return Node` の左の白入力**に **接続**する。  
  → ご質問のとおり、**「直近で追加した 2 つ目の Branch の False → Return Node の左（白）」で正しい**です。

**`Return Value` に `false` を入れる方法（Boolean リテラル）**

| 方法 | 操作 |
|------|------|
| **A（推奨）** | **`Return Node` の `Return Value`（赤）** を **左クリック**する → 表示されるチェックボックスの **チェックを外す**（= **false**） |
| **B** | グラフ空白を **右クリック** → 検索 **`false`** または **`boolean false`** → **`false` リテラル**を **配置** → その **赤ピン**を **`Return Node` の `Return Value`（赤）** に **接続**する |
| **C** | **`Return Node` の `Return Value`（赤）** を **ドラッグ**して空白で離す → 検索 **`false`** → リテラルを **選択** |

---

##### 手順 5 — True 側（Actor が見つかったとき）

| 接続元（主語） | ピン | 次の処理 |
|----------------|------|----------|
| **`Branch`（2 つ目）** | 右側 **`True`（白）** | **B-6** の **`Set Actor Enable Collision`** へ **白線を続ける** |

- **あなた**は **`Branch`（2 つ目）の `True` 白ピン**から **白線**を引き、**B-6** の処理チェーンを **接続**する。

---

##### B-5 のよくある誤解

| 誤解 | 正しい理解 |
|------|------------|
| `Completed` を `Is Valid` に繋ぐ | **不要**。`Is Valid` に白ピンは無い |
| False はループ内 `Branch` の False | **違う**。B-5 用 **2 つ目の `Branch` の False** |
| `Is Valid` が実行を止める | **違う**。`Is Valid` は **Condition 用の赤い値**を出すだけ |

#### B-6. True 側 — 衝突と物理を切る

**`Branch`（2 つ目）の `True`（白）** から続けます（B-5 の分岐ノード）。

> **重要:** ここで操作する対象は **`BP_SpotRobot` 自身の Mesh / Capsule ではなく**、変数 **`FoundCarryActor`（運搬物 Actor）** です。

##### 手順 1 — `Set Actor Enable Collision`

1. **あなた**は **`FoundCarryActor`** 変数を **グラフへドラッグ**し、**`Get FoundCarryActor`** を **配置**する（未配置なら）。  
2. **あなた**は **`Get FoundCarryActor` の青ピン**を **ドラッグ**して空白所で **離す**。  
3. **あなた**は 検索欄に **`Set Actor Enable Collision`** と **入力**し、そのノードを **選択**する。  
   - **Target** が **`FoundCarryActor`（運搬物）** になっていることを **確認**する。  
4. **あなた**は **`New Actor Enable Collision`** のチェックを **外す**（= **false**）。

##### 手順 2 — `Set Simulate Physics`（どれを選ぶか）

**添付メニューの `Set Simulate Physics (Mesh)` / `(CapsuleComponent)` などは選ばないでください。**  
括弧付きの名前は **`BP_SpotRobot` 自身のコンポーネント**向けです。運搬物には使いません。

| メニューに出る名前 | 選ぶ？ | 理由 |
|--------------------|--------|------|
| `Set Simulate Physics (Mesh)` | **×** | ロボットの **Mesh** 用 |
| `Set Simulate Physics (CapsuleComponent)` | **×** | ロボットの **Capsule** 用 |
| `Set Simulate Physics (BodyCollision)` 等 | **×** | 同上（ロボット側） |
| **`Get FoundCarryActor` から出した `Set Simulate Physics`**（Target が運搬物） | **○** | 運搬物用 |

**正しい作り方**

1. **あなた**は **`Get FoundCarryActor` の青ピン**（手順 1 と同じノード）を **ドラッグ**して空白所で **離す**。  
2. **あなた**は 検索欄に **`simulate physics`** と **入力**する。  
3. **あなた**は **次のいずれか**を **選ぶ**（環境により表記が異なります）:

| パターン | ノード | 設定 |
|----------|--------|------|
| **A（推奨）** | **`Set Simulate Physics`**（**Target = FoundCarryActor** の Actor 関数） | **`Simulate` = false** |
| **B** | **`Get Component by Class`**（Class = **`Static Mesh Component`**）→ **`Set Simulate Physics`** | **Target** = 取得した Component、**`Simulate` = false** |
| **C** | **`Get Root Component`** → **`Cast to Primitive Component`** → **`Set Simulate Physics`** | **Target** = Cast 成功参照、**`Simulate` = false** |

4. **パターン A が出ない場合**は **パターン B**（クレート向け）を使う。  
5. **`Get Root Component` だけ**を `Set Simulate Physics` に繋ぐと **型エラー**になります（下記「Root Component が繋がらない」）。  
6. **どれも難しい場合**は **`Set Actor Enable Collision false` だけ**でも可（Python 側でも運搬物の physics は切ります）。**Attach 処理は B-7 へ進んで構いません。**

##### 手順 3 — 実行線を直列接続する

| 接続元（主語） | ピン | 接続先（目的語） | ピン |
|----------------|------|------------------|------|
| **`Branch`（2 つ目）** | **`True`（白）** | **`Set Actor Enable Collision`** | 左 **Exec（白）** |
| **`Set Actor Enable Collision`** | 右 **Exec（白）** | **`Set Simulate Physics`**（またはパターン B の最後のノード） | 左 **Exec（白）** |
| **`Set Simulate Physics`** の右 **Exec（白）** | | **B-7** の **`Attach Actor to Component`** へ **続ける** |

##### 手順 4 — いまのグラフを直す（Target が `self` になっている場合）

スクショのように **`Set Simulate Physics` の Target が `self`** で **オレンジ枠**になっている場合、**ロボット自身**を指しており **誤り**です（`Target is Primitive Component` なのに `self` = この BP のコンポーネント）。

| ノード | いまの状態 | 正しい状態 |
|--------|------------|------------|
| **`Set Actor Enable Collision`** | Target ← `FoundCarryActor`、Collision **OFF** | **このままで OK** |
| **`Set Simulate Physics`** | Target = **`self`**（オレンジ枠） | **削除して作り直す**（下記） |

##### 手順 5 — `Root Component` が `Set Simulate Physics` に繋がらない場合

**エラー例:** `Scene Component Object Reference is not compatible with Primitive Component Object Reference.`

**原因:** **`Get Root Component` の戻り値は `Scene Component` 型**です。  
**`Set Simulate Physics` の Target は `Primitive Component` 型**が必要です。**そのままでは繋げられません。**

**対処（パターン B — 推奨）: `Get Component by Class`**

1. **あなた**は エラーになっている **`Root Component` → `Set Simulate Physics` の青線**を **削除**する（誤った `Set Simulate Physics` ごと削除しても可）。  
2. **あなた**は **`Get FoundCarryActor` の青ピン**を **ドラッグ**して空白で **離す**。  
3. **あなた**は **`Get Component by Class`** を **選択**する。  
4. **あなた**は **`Component Class`** を **`Static Mesh Component`** に **設定**する（`site20_carry` / `BP_Crate_01a` は Static Mesh 系）。  
5. **あなた**は **`Get Component by Class` の `Return Value`（青）** を **ドラッグ**して空白で **離す**。  
6. **あなた**は **`Set Simulate Physics`** を **選択**する。  
7. **あなた**は **`Simulate`** のチェックを **外す**（= **false**）。

**対処（パターン C）: Cast を挟む**

1. **`Get FoundCarryActor`** → **`Get Root Component`**  
2. **`Get Root Component` の Return Value** → **`Cast to Primitive Component` の Object**  
3. **`Cast to Primitive Component` の As Primitive Component`（青）** → **`Set Simulate Physics` の Target**  
4. **`Simulate`** = **false**

**対処（省略）: `Set Simulate Physics` 自体を使わない**

- **`Set Actor Enable Collision false` まで**できていれば **十分**です。  
- **あなた**は **`Set Simulate Physics` ノードを削除**し、**`Set Actor Enable Collision` の右 Exec（白）** から **B-7** へ **白線を続けてください**。

**`Set Simulate Physics` の作り直し（パターン B の完成形）**

**データ線（青）**

```text
[Get FoundCarryActor] ──→ [Set Actor Enable Collision] Target
[Get FoundCarryActor] ──→ [Get Component by Class] Target
                              Component Class = Static Mesh Component
[Get Component by Class] Return Value ──→ [Set Simulate Physics] Target
```

**実行線（白）**

```text
[Branch ②] True ──→ [Set Actor Enable Collision] ──→ [Set Simulate Physics] ──→ B-7 Attach ...
```

#### B-7. Mesh の CarrySocket に Attach する

1. **あなた**は **`FoundCarryActor` の青ピン**から **`Attach Actor to Component`** を **検索して追加**する。  
   - ノード名が **`AttachToComponent`** / **`K2_AttachToComponent`** の場合も同じ操作です。  
   - **Target**（実行対象）は **Carry Actor** 側です（**FoundCarryActor** が Target）。  
2. **あなた**は 各ピンを次のように **設定・接続**する。

| ピン名 | 接続・設定値 |
|--------|----------------|
| **Parent** | B-1 で置いた **`Mesh`** 参照 |
| **Socket Name** | 文字列リテラル **`CarrySocket`**（Phase 1 で付けた名前と **完全一致**） |
| **Location Rule** | **`Snap to Target`** |
| **Rotation Rule** | **`Snap to Target`** |
| **Scale Rule** | **`Keep World`**（または **`Snap to Target`**） |

3. **あなた**は **`Attach Actor to Component` の白い実行出力** → **`Return Node`** を **接続**する。  
4. **あなた**は **成功用 `Return Node` の `Return Value`** に **チェックを ON**（= **`true`**）する。  
   - **失敗用 `Return Node`**（`Branch`② **False** 側）は **チェック OFF**（= **`false`**）のままにする。

#### B-7b. （推奨・任意）関数の先頭で `FoundCarryActor` をクリアする

**これは何か**

- **`FoundCarryActor`** は「ループで見つけた運搬物 Actor」を覚えておく **変数**です。  
- **2 回目以降**に `AttachCarryActor` が呼ばれたとき、**前回の値が残ったまま**だと、今回ループで見つからなくても **`Is Valid` が true** になり、**古い Actor に Attach してしまう**可能性があります。  
- そのため **関数の最初**で一度 **「空（None）」に戻す**のが安全です。

**作業手順**

1. **あなた**は **`Attach Carry Actor` 入口ノードの右 Exec（白）** と、いま **`Get All Actors Of Class` の左 Exec** を **つないでいる白線**を **一度削除**する（または後から差し込む）。  
2. **あなた**は **My Blueprint → Variables** の **`FoundCarryActor`** を **グラフへドラッグ**する。  
3. **あなた**は メニューで **`Set FoundCarryActor`** を **選択**する。  
4. **あなた**は **`Set FoundCarryActor` の値入力（青ピン）** を **右クリック**し、**`Reset to Default`**（または **デフォルトにリセット**）を **選ぶ**。  
   - 表示が **`None`** / 空の Actor 参照になれば OK です。  
   - メニューに無い場合: 値ピンを **クリック**し、ドロップダウンで **未選択（None）** にする。  
5. **あなた**は **白線**で次を **接続**する。

| 接続元（主語） | ピン | 接続先（目的語） | ピン |
|----------------|------|------------------|------|
| **`Attach Carry Actor` 入口** | 右 **Exec（白）** | **`Set FoundCarryActor`** | 左 **Exec（白）** |
| **`Set FoundCarryActor`** | 右 **Exec（白）** | **`Get All Actors Of Class`** | 左 **Exec（白）** |

**完成形（先頭部分だけ）**

```text
[Attach Carry Actor] ──→ [SET FoundCarryActor = None] ──→ [Get All Actors Of Class] ──→ ...
```

> **省略してよいか:** 1 回だけテストする分には **なくても動く**ことが多いです。E2E を何度も回すなら **入れておくことを推奨**します。

#### B-8. Compile と Save

1. **あなた**は ツールバーの **Compile** を **クリック**する。  
2. **UE**は 下部 **Compiler Results** に **`Compile of BP_SpotRobot successful!`** と **表示**すれば成功です。  
3. **あなた**は **File → Save**（または Ctrl+S）で **`BP_SpotRobot`** を **保存**する。

---

### 2-B. `DetachCarryActor` — 新規関数の作成から配線まで

Python は delivery 前に `vbp GridEnv_SpotRobot DetachCarryActor site20_carry` を呼びます。  
**`AttachCarryActor` と同じ「名前で Actor を探す」ループ**を使い、見つかった Actor を **Detach** します。

#### C-0. 関数を新規作成する

1. **あなた**は **`BP_SpotRobot`** の Blueprint エディタを **開く**（`AttachCarryActor` を編集したのと同じ BP）。  
2. **あなた**は 左 **My Blueprint → Functions** の **＋（Function）** を **クリック**する。  
3. **あなた**は 関数名を **`DetachCarryActor`** と **入力**する（Enter で確定）。  
4. **あなた**は **`DetachCarryActor`** を **1 回クリック**して、その関数グラフを **開く**。

#### C-1. Inputs / Outputs を設定する

1. **あなた**は 右 **Details** の検索欄が **空**であることを **確認**する（フィルタがあれば **× でクリア**）。  
2. **あなた**は **Inputs** 行の **＋** を **クリック**する。  
3. **あなた**は **Name** に **`CarryActorName`**、**Type** に **`String`**（または **Name**）を **設定**する。  
4. **あなた**は **Outputs** 行の **＋** を **クリック**する。  
5. **あなた**は **Type** を **`Boolean`** に **設定**する。  
6. **あなた**は **Compile** を **クリック**する。

#### C-2. 関数先頭で `FoundCarryActor` をクリアする

`AttachCarryActor` と **同じ変数 `FoundCarryActor`** を使います（BP 全体で共有）。

1. **あなた**は **Variables** の **`FoundCarryActor`** を **グラフへドラッグ** → **`Set FoundCarryActor`** を **選択**する。  
2. **あなた**は **`Set FoundCarryActor` の青い値入力**を **未接続**（= **None**）のままにする（右クリック → **Reset to Default** でも可）。  
3. **あなた**は **`Detach Carry Actor` 入口の右 Exec（白）** → **`Set FoundCarryActor` の左 Exec** を **接続**する。

#### C-3. Actor 検索ループ（Attach と同一）

**あなた**は **`AttachCarryActor` の B-2 ～ B-4** と **同じノード列**を **再現**する（コピー＆ペーストでも可）。

| 手順 | ノード | 設定・接続 |
|------|--------|------------|
| 1 | **`Get All Actors Of Class`** | **Actor Class = `Actor`**（`None` 禁止） |
| 2 | **`For Each Loop`** | `Out Actors` → `Array` |
| 3 | 実行線 | `Set FoundCarryActor` 右 Exec → `Get All Actors` → `For Each Loop` Exec |
| 4 | ループ内 | `Array Element` → `Get Object Name` → `Equal (String)` ← `Carry Actor Name` |
| 5 | | `Loop Body` → `Branch`①、`Equal` → `Condition` |
| 6 | | `Branch`① **True** → `SET FoundCarryActor` ← `Array Element` |

#### C-4. ループ後の検証（Attach の B-5 と同一）

| 接続元 | ピン | 接続先 | ピン |
|--------|------|--------|------|
| **`For Each Loop`** | **`Completed`（白）** | **`Branch`②** | **`Exec`（白）** |
| **`Get FoundCarryActor`** | 青 | **`Is Valid`** | **`Input Object`（青）** |
| **`Is Valid`** | **`Return Value`（赤）** | **`Branch`②** | **`Condition`（赤）** |

- **`Branch`② False** → **`Return Node`**（**Return Value = false**）  
- **`Branch`② True** → 次の C-5 へ

#### C-5. Detach 実行

1. **あなた**は **`Branch`② の `True`（白）** から **`Detach from Actor`**（または **`K2_DetachFromActor`**）を **追加**する。  
   - **`FoundCarryActor` の青ピン**を **ドラッグ** → 検索 **`detach`** → 選ぶ方法でも可。  
2. **あなた**は **`Detach from Actor` の Target** に **`Get FoundCarryActor`** を **接続**する。  
3. **あなた**は 次の Rule を **設定**する（delivery アニメ用にワールド座標を維持）:

| ピン | 値 |
|------|-----|
| **Location Rule** | **`Keep World`** |
| **Rotation Rule** | **`Keep World`** |
| **Scale Rule** | **`Keep World`** |

4. **あなた**は **`Detach from Actor` の右 Exec（白）** → **成功用 `Return Node` の左 Exec** を **接続**する。  
5. **あなた**は **成功用 `Return Node` の `Return Value`** に **チェック ON**（= **`true`**）を **設定**する。

**完成形（Detach 部分）**

```text
[Branch ②] True → [Detach from Actor] (Target=FoundCarryActor, Keep World) → [Return true]
[Branch ②] False → [Return false]
```

#### C-6. Compile と Save

1. **あなた**は **Compile** を **クリック**する。  
2. **あなた**は **Save**（Ctrl+S）する。

---

### 2-C. （任意）`ProbeCarryAttach` — vbp 可用性チェック用

Python は最初に `vbp GridEnv_SpotRobot ProbeCarryAttach` を試します。無くても **`AttachCarryActor __probe__`** で代替判定しますが、あるとログが分かりやすくなります。

#### D-1. 関数を作成する

1. **あなた**は **Functions → ＋** で **`ProbeCarryAttach`** を **作成**する。  
2. **Inputs** は **追加しない**（0 個のまま）。  
3. **Outputs** の **＋** → **Type** = **`Boolean`** を **設定**する。

#### D-2. グラフ（簡易版）

1. **あなた**は **`Mesh (CharacterMesh0)`** を **グラフへドラッグ**する。  
2. **あなた**は **`Mesh`** から **`Does Socket Exist`**（検索: `Socket Exist`）を **追加**する。  
3. **あなた**は **Socket Name** に **`CarrySocket`** を **入力**する。  
4. **あなた**は 入口の白ピン → **`Does Socket Exist`** → **`Return Node`** を **接続**する。  
5. **あなた**は **`Does Socket Exist` の戻り値（Boolean）** → **`Return Value`** を **接続**する。  
6. **あなた**は **Compile** → **Save** する。

---

### 2-D. Callable 設定の確認（vbp 用）

`Move_Speed` と同じ BP に書いていれば、通常は追加設定不要です。念のため:

1. **あなた**は 各関数（`AttachCarryActor` 等）を **選択**する。  
2. **あなた**は **Details → Graph** セクションで **Access Specifier** が **`Public`** であることを **確認**する（`Private` だと vbp から呼べない場合があります）。  
3. **あなた**は 再度 **Compile** → **Save** する。

---

## Phase 3: 動作確認

### 3-A. エディタで BP を保存したあと PIE を開始する

1. **あなた**は **PIE（Play）** を **停止**していた状態で、上記 **Compile / Save** を **完了**させる。  
2. **あなた**は **Level エディタ**に戻り、**Play** ボタンを **クリック**する。  
3. **UE**は **`GridEnv_SpotRobot`** を含むシーンで **シミュレーション**を **開始**する。

### 3-B. WSL からプローブを実行する

PIE Play 後、WSL から:

```bash
python dev/grid_env_level_nav/scripts/probe_carry_attach_vbp.py
```

**期待される表示:**

```text
[carry-probe] AttachCarryActor vbp: AVAILABLE
```

`site20_carry` がシーンにいる状態なら、続けて trial attach の **OK** も出ます。

### 3-C. E2E で pickup ログを確認する

```bash
PYTHONUNBUFFERED=1 ~/miniforge3/envs/simworld/bin/python \
  dev/grid_env_level_nav/run_site_transport_20m_test.py --max-nav-steps 600
```

leg1 完了・pickup 後の期待ログ:

```text
[Site20Carry] UE bone attach 'site20_carry' → socket 'CarrySocket' (no Python sync during leg2)
```

**フォールバック時**（vbp 未設定のまま）:

```text
[Site20Carry] visual ready 'site20_carry' @ z=... (floor+88cm, Python sync — see CARRY_ATTACH_UE_SETUP.md for bone attach)
```

---

## トラブルシュート

| 症状 | 対処 |
|------|------|
| Details に Inputs が出ない | 検索欄のフィルタ（例: `Skeletal Mesh`）を **× でクリア**する |
| ピン右クリックで `Get Object Name` が出ない | **正常**。ピンを **空白へドラッグして離す**（B-3 方法 A）か、**空白右クリック**でノード追加（方法 B） |
| `Equal (Name)` が検索に無い | **正常**。`Get Object Name` は **String** を返す → **`Equal Exactly (String)`** を使う |
| `Equal (Text)` だけ目立つ | **Text は使わない**。String カテゴリの **Equal (String)** を選ぶ |
| `Is Valid` がたくさん出る | **`Input Object`（青）1 つだけの `Is Valid`** を選ぶ（Index / Timer / Class 系は不可） |
| `Root Component` が `Set Simulate Physics` に繋がらない | **型不一致（正常なエラー）**。`Get Component by Class (Static Mesh Component)` 経由にするか、**Set Simulate Physics を省略** |
| `Get All Actors Of Class` の Actor Class が `None` | ドロップダウンで **`Actor`** を選択（`None` のままでは配列が空） |
| `AttachCarryActor not found` | BP 未 Compile / 別 BP を編集している / Pawn 名不一致 |
| vbp は通るが常に `false` | `Get Object Name` と Python スポーン名 `site20_carry` が一致しているか **Output Log** で確認 |
| attach 成功だが位置がずれる | Phase 1 の **`CarrySocket`** オフセットを Skeleton Editor で再調整 |
| Penetration で stuck | Carry の **Set Actor Enable Collision false** が Attach 直前に実行されているか確認 |
| detach 後 delivery アニメが変 | **Detach** の Rule を **`Keep World`** にする |

---

## Python 側（実装済み）

| ファイル | 役割 |
|----------|------|
| `carry.py` | `attach_carry_to_robot_bone`, `detach_carry_from_robot_bone`, `is_carry_ue_attached` |
| `layered_nav.py` | UE attach 成功時は leg2 の `sync_carry_pose` をスキップ |
| `scripts/probe_carry_attach_vbp.py` | vbp 可用性プローブ |
