# AI Perception Sight — UE Editor セットアップ手順（UE5）

Python 側は `--l2-mode sight`（デフォルト）で `vbp {robot} GetVisibleSightTargetsJson` を呼びます。  
UE 側が未完了の間は `geom_fallback`（registry FOV）にフォールバックします。完了後はログが `L2 sight (ue_sight):` になります。

---

## 用語（UE5 の画面の呼び方）

| 日本語・略称 | 画面での名前 | 何ができるか |
|-------------|-------------|-------------|
| **Content Browser / Content Drawer** | 画面下の **Content Drawer** タブ | `.uasset` 一覧・フォルダ移動・右クリックで新規作成 |
| **Blueprint エディタ** | `BP_xxx` タブを開いた画面 | Viewport・Components・Details・グラフ編集 |
| **Class Settings** | ツールバーの **Class Settings** | 親クラス・Interface など **クラス定義**の設定（**Pawn はここに無い**） |
| **Class Defaults** | ツールバーの **Class Defaults** | **Pawn** カテゴリ（AI Controller Class など）の **デフォルト値** |
| **Details** | 右側 **Details** パネル | 選択中の対象のプロパティ一覧 |

> **よくある間違い**: 「Class Settings」を押しても **Pawn → AI Controller Class** は出ません。**Class Defaults** を押してください。

---

## 事前確認: 親 BP と Child BP

| Blueprint | 役割 |
|-----------|------|
| `BP_SpotRobot` | 親・他シナリオ共用のデフォルト |
| `BP_SpotRobot_Child` | site_transport 用の拡張（ここに AI 設定を足す） |

**注意**: Python の spawn は現在 **親** `BP_SpotRobot` を使っています（`grid_env_hri_simulation.ROBOT_BP`）。  
Child にだけ設定した場合は **効きません**。本手順の最後に **spawn を Child に切り替える**か、**親にも同じ Class Defaults をコピー**してください（末尾 Phase 8 参照）。

---

## Phase 1: `BP_SpotRobot_Child` を開く

1. SimWorld の **Unreal Editor** を起動する
2. 画面 **左下** の **Content Drawer** タブをクリックする  
   （無い場合: メニュー **Window → Content Browser → Content Browser 1**、または **Ctrl + Space**）
3. フォルダツリーで **`Content/Robot_Dog/Blueprint/`** に移動する
4. 一覧から **`BP_SpotRobot_Child`** を **ダブルクリック**する  
   → 中央にロボットの Viewport、左上に **Components**、右に **Details** が出れば成功

---

## Phase 2-A: `BP_SpotDogAIController` を新規作成

`BP_SpotRobot_Child` のタブは **閉じてよい**（Content Drawer に戻る）。

1. **Content Drawer** で `Content/Robot_Dog/Blueprint/` を開いたまま、右側の **空白** を **右クリック**
2. **Blueprint Class** をクリック
3. **Pick Parent Class** ウィンドウが開く  
   - 上の **All Classes** タブを選ぶ  
   - 検索ボックスに `AIController` と入力  
   - **AIController** をクリック → 右下 **Select**
4. 名前を **`BP_SpotDogAIController`** にして **Create**（または Enter）
5. 作成された **`BP_SpotDogAIController`** を **ダブルクリック**して Blueprint エディタを開く

### AI Perception コンポーネントを足す

1. 左上 **Components** パネルで **Add** ボタン（＋）をクリック  
   （または Components 内の空白で右クリック → **Add Component**）
2. 検索欄に `AI Perception` と入力 → **AI Perception** を選択
3. **Components** 一覧に **AI Perception** が増えたことを確認
4. **AI Perception** を **1回クリック**して選択する（青くハイライト）
5. 右側 **Details** を下にスクロール → **AI Perception** セクションを開く
6. **Senses Config** の右にある **＋ Add** をクリック → **AI Sight Config** を選ぶ
7. 追加された **Index [0]**（AI Sight config）を開き、次の値を入れる:

| 項目名（Details 上の表記） | 設定値 |
|--------------------------|--------|
| **Sight Radius** | `650.0` |
| **Lose Sight Radius** | `700.0` |
| **Peripheral Vision Half Angle Degrees** | `45.0`（半角 45°＝全角 90°） |
| **Auto Success Range From Last Seen Location** | `-1.0` |
| **Detect Enemies** | お好みで ON |
| **Detect Neutrals** | **ON（必須に近い）** — prop は多くが Neutral |
| **Detect Friendlies** | 必要なら ON |
| **Starts Enabled** | ON |

8. ツールバー **Compile**（緑のチェックマーク）をクリック
9. **Save**（Ctrl+S）

---

## Phase 2-B: SpotDog に AI Controller を割り当て（重要・手順詳細）

ここが **Class Settings ではなく Class Defaults** です。

### Step 1: `BP_SpotRobot_Child` を開く

1. **Content Drawer** → `BP_SpotRobot_Child` を **ダブルクリック**

### Step 2: Class Defaults モードにする

Blueprint エディタ **上部ツールバー**（Compile / Save の並び）を見る:

```
[ Compile ] [ Save ] [ Browse ] ... [ Class Settings ] [ Class Defaults ] [ Simulation ]
```

1. **Class Defaults** を **1回クリック**する  
   - ボタンが **押された状態**（ハイライト）になる  
   - 右 **Details** の内容が「この Blueprint のデフォルト設定」に切り替わる

> **Class Settings** を押していると、Details には **Class Options / Parent Class / Interfaces** だけが出ます（あなたのスクリーンショットの状態）。**Pawn は出ません。**

### Step 3: Details で Pawn セクションを見つける

1. 右 **Details** パネル上部の **検索ボックス**（虫眼鏡 / "Search"）に  
   `AI Controller` と入力する
2. 次の2項目が絞り込みで出るはずです:

| 検索で見つかる項目 | 設定値 |
|-------------------|--------|
| **AI Controller Class** | ドロップダウン → **BP_SpotDogAIController** を選択 |
| **Auto Possess AI** | ドロップダウン → **Placed in World or Spawned** を選択 |

（検索を消すと、これらは **Pawn** カテゴリの下にまとまって表示されます。）

#### ドロップダウンの選び方

**AI Controller Class**:

1. 右側の **ドロップダウン**（現在 `None` や `AIController` 等）をクリック
2. 一覧から **BP_SpotDogAIController** を探してクリック  
   - 無い場合: 下の **Browse**（虫眼鏡アイコン）→ `Robot_Dog/Blueprint/BP_SpotDogAIController` を選ぶ

**Auto Possess AI**（`EAutoPossessAI` 列挙）:

| 値 | 意味 |
|----|------|
| Disabled | AI Controller を自動生成しない |
| Placed in World | レベルに置いたときだけ |
| Spawned | spawn したときだけ |
| **Placed in World or Spawned** | **どちらでも**（SimWorld は spawn するのでこれ） |

### Step 4: 保存

1. ツールバー **Compile**
2. **Save**（Ctrl+S）

### 設定できたかの確認

1. もう一度 **Class Defaults** が押されていることを確認
2. Details 検索で `AI Controller` → **BP_SpotDogAIController** と表示されている
3. **Auto Possess AI** = **Placed in World or Spawned**

---

## Phase 3: 視点（目の位置）— VisionSensor

SpotDog の **Mesh** は **SkeletalMeshComponent**（`SK_Robot_Dog_...`）です。  
このタイプでは Details の **Sockets** に出てくる **Parent Socket** は「親のソケットに付ける」用で、**新規ソケットの追加ボタン（Add Socket）はありません**。Static Mesh 用の手順とは異なります。

以下 **方法 A（推奨）** か **方法 B** のどちらかで進めてください。

---

### 方法 A（推奨）: Scene Component で視点を置く — Blueprint だけで完結

Skeleton アセットを編集せず、`BP_SpotRobot_Child` 内に空のコンポーネントを足します。

#### Step A-1: コンポーネントを追加

1. `BP_SpotRobot_Child` を開く（**Class Defaults ではない通常モード**）
2. 左上 **Components** パネルで **Mesh (CharacterMesh0)** を **1回クリック**して選択
3. **Add** ボタン（＋）をクリック  
   または **Mesh** の上で **右クリック** → **Add Component**
4. 検索欄に `Scene` と入力 → **Scene Component** を選択
5. 名前を **`VisionSensor`** に変更（Components 一覧で F2 または Details の Variable Name）

#### Step A-2: Mesh の子として付ける

1. **Components** で **VisionSensor** を **ドラッグ**し、**Mesh (CharacterMesh0)** の **上にドロップ**して子にする  
   階層例:
   ```
   BP_SpotRobot_Child
     └ Mesh (CharacterMesh0)
         ├ FusionCamSensor
         ├ ThirdPersonCamera
         ├ BodyCollision
         └ VisionSensor   ← 新規
   ```
2. **VisionSensor** を選択
3. 右 **Details** → **Transform**:
   - **Location X** ≈ `22`（ロボット前方 cm。向きが合わなければ ± を調整）
   - **Location Y** ≈ `0`
   - **Location Z** ≈ `45`（高さ cm）
   - **Rotation** は必要なら Pitch ≈ `-5`（やや下向き）

#### Step A-3: Viewport で位置を確認（任意）

1. 中央 **Viewport** で **VisionSensor** が選択された状態
2. **W** キーで移動ギズモを出し、ロボットの「目」相当の位置に合わせる

#### Step A-4: 視線原点（Get Actor Eyes View Point）— Blueprint では省略可

> **よくある状況**: **My Blueprint → Functions → Override** の一覧に  
> **Get Actor Eyes View Point** が **出てきません**。  
> これは不具合ではなく、標準の `APawn::GetActorEyesViewPoint` が **C++ 専用**で、  
> **Blueprint から Override できない**ためです（`BP_SpotRobot` / `Character` 継承でも同様）。

**推奨（Blueprint のみの場合）: Step A-4 はスキップして Phase 4 へ進む**

- AI Sight は **Actor Location（ルート位置）** と **Actor Rotation（向き）** を視線の原点に使います。
- `VisionSensor` の Transform（例: Y=`50`, Z=`65`）は **目印・将来用**として残して問題ありません。
- 検知距離 650 cm・FOV 90° のサイト搬送では、原点が体中心寄りでも L2 用途では多くの場合十分です。
- PIE 後に **AI Debugger**（`'` キー）で Sight 円錐の向き・範囲を目視確認してください。

**どうしても `VisionSensor` 位置を視線原点にしたい場合**（上級・任意）:

| 手段 | 内容 |
|------|------|
| C++ | 親クラス（`APawn` 派生）で `GetActorEyesViewPoint` を `BlueprintNativeEvent` 化する |
| 簡易代替 | **FusionCamSensor** と同じ付近なら、既定の Actor 原点＋向きの誤差は小さいことが多い |

C++ で Override 可能にした場合の配線例:

1. Override 一覧に **Get Actor Eyes View Point** が現れる
2. **Get World Location**（Target = `VisionSensor`）→ Return **Location**
3. **Get World Rotation**（Target = `VisionSensor`）→ Return **Rotation**
4. **Compile** → **Save**

> **既存の FusionCamSensor** を視点として流用する場合も、Blueprint だけでは AI Sight 原点は変わりません。  
> FusionCam は Python 側カメラ／将来の C++ 実装との位置合わせ用の参考として使えます。

---

### 方法 B: Skeleton アセットに Socket を追加 — 骨に固定したい場合

ソケットは **スケルタルメッシュ／スケルトンアセット** 側で定義します。

#### Step B-1: スケルタルメッシュを開く

1. `BP_SpotRobot_Child` の **Details → Mesh → Skeletal Mesh Asset** に表示されているアセット  
   （例: `SK_Robot_Dog_ColorVariatio...`）の **サムネイル** または **ドロップダウン右の Browse** をクリック
2. **Skeletal Mesh Editor**（または Skeleton Editor）が開く

#### Step B-2: Skeleton Tree で Socket 追加

1. 左側 **Skeleton Tree**（骨の階層）パネルを表示  
   （無い場合: メニュー **Window** から Skeleton Tree を開く）
2. 頭または胸に相当する **ボーン**（例: `head`、`spine_03` 等、モデル依存）を **右クリック**
3. **Add Socket** をクリック
4. 名前を **`VisionSensor`** に変更
5. Viewport でソケット位置を **前方・上方** に調整（目安: 前方 22 cm、上 45 cm 相当）
6. **Save** して Skeletal Mesh Editor を閉じる

#### Step B-3: Blueprint に戻る

1. `BP_SpotRobot_Child` を開き、**Mesh** の **Details → Sockets → Parent Socket** ではなく、  
   他コンポーネントを `VisionSensor` ソケットに **Attach** する使い方も可能
2. Phase 3 の視線原点は **方法 A の A-4 と同様**（Blueprint では Override 不可のため、多くの場合スキップで可）

---

### Phase 3 完了チェック

| 確認項目 | OK の目安 |
|----------|-----------|
| 視点コンポーネント | `VisionSensor` Scene Component または Skeleton の `VisionSensor` Socket がある |
| 位置 | ロボット前方・やや上（Viewport で目視 OK。軸はモデル依存で Y 前方でも可） |
| Get Actor Eyes View Point | **無くても可**（Blueprint 標準では Override 不可） |
| PIE + AI Debugger | Sight 円錐がロボットの向きに沿って出ている（原点は Actor 中心付近で可） |

---

## Phase 4: 知覚対象（Prop / Humanoid）に Stimuli Source

検知される側の Actor に **AI Perception Stimuli Source** が必要です。  
site_transport の静的 prop は **73 個**ありますが、**親 Blueprint に 1 回足す**か、**Execute Python Script** で一括設定できます。

### 推奨: Execute Python Script（Prop 親 BP + Humanoid — 1 回で完了）

既存の CustomDepth 用スクリプトと同じパターンです。**Humanoid もデフォルトで含まれます。**

1. **PIE を停止**（Play 中はアセット保存できません）
2. UE メニュー **Tools → Execute Python Script**
3. 次のファイルを選択:

   `dev/grid_env_level_nav/scripts/enable_ai_sight_stimuli_on_level_props_editor.py`

   （WSL 上の絶対パス例:  
   `~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav/scripts/enable_ai_sight_stimuli_on_level_props_editor.py`）

4. **Output Log** で次を確認:
   - `[SightStimuliProps] mode=base_and_extras targets=2`
   - `[SightStimuliProps]   target: .../BP_LevelProp_Base`
   - `[SightStimuliProps]   target: .../Base_User_Agent`
   - `[SightStimuliProps] OK` が 2 行（または `skip (already configured)`）
   - `updated=2`（初回）または `updated=0 skipped=2`（再実行時）

**動作（2 件）**:

| 対象 | パス | 効果 |
|------|------|------|
| Prop 親 BP | `BP_LevelProp_Base` | 73 子 prop すべてに **継承** |
| Humanoid | `Base_User_Agent` | `site20_humanoid` スポーン元に Sight 刺激を登録 |

Humanoid BP が見つからない場合は **警告を出して skip** します（TrafficSystem pak 未マウント等）。  
そのときは pak をマウントしてからスクリプトを再実行してください。

#### オプション: 環境変数 `SIMWORLD_SIGHT_STIMULI_MODE`

| 値 | 内容 |
|----|------|
| `base_and_extras` | **デフォルト** — 親 prop BP + Humanoid |
| `props_only` | 親 prop BP のみ（Humanoid は触らない） |
| `all_generated` | 73 子 BP を個別更新 + Humanoid（継承が効かないとき） |

#### 73 子 BP それぞれに直接足したい場合（通常は不要）

継承が効かない特殊な BP だけ直すとき:

```text
SIMWORLD_SIGHT_STIMULI_MODE=all_generated
```

で同スクリプトを実行（最大 73 件 + Humanoid）。

---

### 手動（1 Blueprint ずつ）

1. Content Drawer で例: `BP_LevelProp_Base` または任意の `BP_*` prop を開く
2. **Add Component** → **AI Perception Stimuli Source**
3. Details:
   - **Register as Source for Senses** または **Register as Source**: ON
   - **Sight** にチェック
4. **Compile** → **Save**
5. 親に足した場合は子 prop への繰り返しは不要（Humanoid もスクリプト既定で処理）。

### まとめてやる方法（手動・親 1 回）

親の **`BP_LevelProp_Base`**（`/Game/SimWorld/LevelProps/Base/`）に上記コンポーネントを 1 回足せば、  
Construction VOL.1 の **73 子 prop** に継承されます。

> 旧ドキュメントの `BP_InteractableAssetBase` は別パイプライン用です。  
> site_transport_20m の LevelProp は **`BP_LevelProp_Base` 系**を使います。

---

## Phase 5: Python 向け `GetVisibleSightTargetsJson`（vbp）

**重要**: Python は **`vbp GridEnv_SpotRobot GetVisibleSightTargetsJson`** と **ロボット Pawn** に対して呼びます。  
ロジックは **`BP_SpotDogAIController`** に書き、**Pawn 側に薄いラッパー**を足します（Phase 5-B）。

### 返却 JSON 例

```json
{
  "targets": [
    {"actor": "site20_prop_003", "prop_type_id": "dumpster", "is_dynamic": false},
    {"actor": "site20_humanoid", "prop_type_id": "human_worker", "is_dynamic": true}
  ]
}
```

`prop_type_id` / `is_dynamic` は省略可（Python が registry から補完）。**`actor` 名（スポーン名）が最重要**です。

---

### Phase 5-A: `BP_SpotDogAIController` に本体を書く（今の画面）

`GetVisibleSightTargetsJson` 関数グラフが開いている状態から進めます。

#### A-1: 関数の入出力を確認

1. 左 **My Blueprint → Functions** で **`GetVisibleSightTargetsJson`** を選択
2. 右 **Details**:
   - **Inputs**: なし
   - **Outputs**: **Return Value** の型が **String**
   - （Controller 側は Callable でなくてよい）

#### A-2: 知覚 Actor 一覧を取得

> **注意**: **Get Currently Perceived Actors** は **Pure 関数**です。  
> **白い実行ピン（入力・出力）はありません。** 青いデータピンだけで配線します。

1. **AIPerception** コンポーネントを **Components** パネルからグラフへ **ドラッグ**（Get 参照）
2. **AIPerception** の **青いピン**から **Get Currently Perceived Actors** を追加  
   （検索: `Currently Perceived` または `Perceived Actors`）  
   - 実行ピン（白）から検索すると **`Get Actors Perception`** だけ出ることがあります → **選ばない**  
   - **`Get Actors Perception`**: 特定 Actor **1体**の知覚状態  
   - **`Get Currently Perceived Actors`**: 今 Sight で見えている Actor **一覧** ← **こちら**
3. **Target** ← **AIPerception** 参照
4. **Sense to Use** ← **AI Sight**（`AISense_Sight`）
5. **Out Actors**（青）→ **For Each Loop** の **Array**（青）  
   ※ 実行線は **For Each Loop** 側だけに通す（次の A-3）

#### A-3: 実行配線 + ループ（For Each Loop）

**白い実行線**は **関数エントリ → Set Json → For Each Loop → … → Return** だけに通します。

1. 関数エントリ（紫）の **白ピン** → **Set Json**（初期値 `{"targets":[`）→ **For Each Loop** の **Exec**（左の白）
2. **Out Actors** → **For Each Loop** の **Array**（済みならそのまま）
3. **Loop Body**（白）から Append 処理へ（A-4）
4. **Completed**（白）→ Append `]}` → **Return Node**
   - **Array Element** → **Get Object Name**（Target = Element）  
     → スポーン名 `site20_prop_000` 等が取れる
   - （任意）**Cast to BP_LevelProp_Base** → **Get PropTypeId**  
   - （任意）Object Name に `humanoid` が含まれる → `is_dynamic=true`

#### A-4: 文字列を連結して JSON を組み立て（Append）

Blueprint に JSON ノードが無いので **Append（文字列結合）** で組み立てます。

1. ループの前: 変数 **`Json`**（String）を用意し **`{"targets":[`** で初期化
2. **Loop Body** 内:
   - 2件目以降は `,` を Append
   - 各 Actor ごとに次を Append（`Name` = Get Object Name の結果）:

     `{"actor":"` + Name + `"}`

   - 例: `{"actor":"site20_prop_003"}`
3. **Completed** ピンから: **`]}`** を Append → **Return Node** の **Return Value** へ

**最小構成**（`prop_type_id` 省略）でも Python は動作します。

#### A-5: 空のとき

**Out Actors** の長さが 0 のときは **`{"targets":[]}`** を Return。

#### A-6: Compile → Save

`BP_SpotDogAIController` を **Compile** → **Save**。

---

### Phase 5-B: Pawn ラッパー（vbp 用・必須）

spawn されているのは親 **`BP_SpotRobot`**（`GridEnv_SpotRobot`）です。  
**同じ関数名**を Pawn に **Callable** で追加し、Controller を呼びます。

> Phase 8 で Child に切り替えた後は **`BP_SpotRobot_Child`** に同じラッパーを書いてください。

1. **`BP_SpotRobot`**（または Child）を開く
2. **My Blueprint → Functions → + Function**
3. 名前: **`GetVisibleSightTargetsJson`**
4. **Details**:
   - **Callable** = **ON**
   - **Category** = `default` または `SimWorld`
   - **Return Value** = **String**
5. グラフ配線:
   - **Get Controller**
   - **Cast to BP_SpotDogAIController**
   - Cast 成功 → **Get Visible Sight Targets Json**（Controller の関数を呼ぶ）
   - 戻り値 → **Return Node**
   - Cast 失敗 → Return **`{"targets":[]}`**
6. **Compile** → **Save**

---

### Phase 5 完了チェック

| 確認 | OK |
|------|-----|
| Controller に AIPerception + Sight 設定済み | Phase 2-A |
| Pawn Class Defaults → AI Controller = BP_SpotDogAIController | Phase 2-B |
| Pawn に Callable `GetVisibleSightTargetsJson` | Phase 5-B |
| PIE 中 vbp が JSON を返す | Phase 6-B |

---

## Phase 6: 動作確認

### 6-A: AI Debugger（Editor）

1. **Play（▶）** で PIE 開始
2. キー **`'`**（シングルクォート）で **AI Debugger**  
   または **Window → Developer Tools → AI Debugger**
3. SpotDog 付近に Sight 円錐と検知 Actor が出るか確認

### 6-B: vbp（WSL・PIE 中）

```bash
cd /home/winder17wsl_ishizawalab/00_kotaprivate/Program/SimWorld
conda run -n simworld python -c "
import sys
sys.path.insert(0,'dev/grid_env_hri')
import grid_env_hri_simulation as geh
ucv,_=geh.ensure_connection()
print(ucv.client.request('vbp GridEnv_SpotRobot GetVisibleSightTargetsJson'))
"
```

JSON が返れば成功。`error` なら関数名・spawn されている BP を確認。

### 6-C: site_transport 本番

```bash
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld python \
  dev/grid_env_level_nav/run_site_transport_20m_test.py --skip-spawn
```

期待: `[Site20] L2 sight (ue_sight): visible=N ...`

---

## Phase 7: トラブルシュート

| 症状 | 原因 | 対処 |
|------|------|------|
| Class Settings に Pawn が無い | **Class Defaults** 未選択 | ツールバー **Class Defaults** をクリック |
| 何も検知されない | **Detect Neutrals** OFF | AI Sight config で Neutrals を ON |
| 設定したのに効かない | spawn が **親 BP** | Phase 8 を実施 |
| vbp が error | 関数未実装 or 名前不一致 | `GetVisibleSightTargetsJson` のスペル確認 |
| geom_fallback のまま | vbp 失敗 | Phase 5・6-B を再確認 |
| `targets:[]` だが geom は visible>0 | Pawn が **AIController_0** のまま（Cast 失敗） | spawn 後に `ensure_spotdog_sight_controller` が走るか確認。手動: Pawn Details → AI Controller Class = BP_SpotDogAIController、PIE 再起動 |
| Python が途中で接続不能 | UnrealCV **単一クライアント** + destroy/spawn 中の TCP reset | ゾンビ Python を終了、`prepare_ue_connection` 使用。PIE は Play のままでも 10–30s 応答不能になり得る |

---

## Phase 8: spawn を Child に合わせる（Python 側）

`BP_SpotRobot_Child` にだけ設定した場合、次のいずれかが必要です。

**A（推奨）**: `dev/grid_env_hri/grid_env_hri_simulation.py` の `ROBOT_BP` を Child に変更:

```text
/Game/Robot_Dog/Blueprint/BP_SpotRobot_Child.BP_SpotRobot_Child_C
```

**B**: 親 `BP_SpotRobot` の **Class Defaults** にも、Phase 2-B と同じ **AI Controller Class** / **Auto Possess AI** を設定する

---

## Python 側 L2 ポリシー（実装済み）

| 種別 | 例 | FOV 外 |
|------|-----|--------|
| 静的 | props, crate | **最後に見た位置を L2 に保持** |
| 動的 | humanoid, 他ロボット | **L2 から削除** |

---

## 参考リンク

- [AI Perception（Epic 公式）](https://dev.epicgames.com/documentation/en-us/unreal-engine/ai-perception-in-unreal-engine)
- [APawn — AIControllerClass / AutoPossessAI](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Engine/APawn)
