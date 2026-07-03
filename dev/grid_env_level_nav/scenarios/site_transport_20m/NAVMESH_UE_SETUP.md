# Dynamic NavMesh Hybrid — UE セットアップ（Phase 1–4）

Python 側は `--nav-mode navmesh` で NavFindPath ナビを使います。
**以下の UE 作業が完了するまで、ミッションは起動時に FAIL します。**

## 前提

- SimWorld UE プロジェクトに **既存** `NavQueryService`（`NavProjectPoint` が動く）があること  
  - smoke: `conda run -n simworld python dev/grid_env_level_nav/_nav_project_point_smoke_test.py`
- PIE は `/Game/Maps/Level`（site_transport_20m と同じマップ）

## Step 1 — C++ をコピー

| コピー元（本リポジトリ） | コピー先（UE プロジェクト） |
|---|---|
| `dev/grid_env_level_nav/ue_native/NavQueryService.h` | `Source/SimWorld/Public/NavQueryService.h` |
| `dev/grid_env_level_nav/ue_native/NavQueryService.cpp` | `Source/SimWorld/Private/NavQueryService.cpp` |

**上書き**で構いません（`NavRebuild`, `GetActorBoundsJson`, `NavRegisterBoxObstacle` 等が追加されています）。

## Step 2 — Build.cs に依存追加

`Source/SimWorld/SimWorld.Build.cs` の `PrivateDependencyModuleNames` に未追加なら:

```csharp
"NavigationSystem",
"AIModule",
```

## Step 2b — Runtime NavMesh を Dynamic に

`Config/DefaultEngine.ini` の `[/Script/NavigationSystem.RecastNavMesh]` に次を設定:

```ini
RuntimeGeneration=Dynamic
```

UE Editor でも確認: **Project Settings → Navigation Mesh → Runtime Generation → Dynamic**

設定変更後は **Editor 再起動** と **Build → Build Paths** が必要です。
Dynamic Modifiers Only だけだと PIE 開始時に NavMesh が空のままになることがあります。

**Phase 5（UE MoveTo）** は [`NAVMESH_PHASE5_UE_SETUP.md`](NAVMESH_PHASE5_UE_SETUP.md) を参照。

## Step 3 — Visual Studio で Rebuild

1. **UE Editor を完全終了**
2. `SimWorld.sln` を Visual Studio で開く
3. 構成: **Development Editor** / プラットフォーム: **Win64**
4. メニュー **ビルド → ソリューションのリビルド**
5. 成功したら UE Editor を起動

## Step 4 — BP を再コンパイル

1. Content Browser で `BP_NavQueryService`（または `CustomAssets/BP_NavQueryService`）を開く
2. **Compile** → **Save**
3. レベルに配置済みならそのまま。未配置なら site 20m 領域近くに 1 つ Drag&Drop（任意）

## Step 5 — NavMesh をベイク

1. **Window → World Settings** → **Navigation Mesh** が有効
2. 床全体を覆う **Nav Mesh Bounds Volume** があることを確認
3. ツールバー **Build → Build Paths**（または `P` キー）で NavMesh 生成

## Step 6 — WSL から API 確認（PIE Play 中）

```bash
conda activate simworld
cd ~/00_kotaprivate/Program/SimWorld
python dev/grid_env_level_nav/_nav_project_point_smoke_test.py
python dev/grid_env_level_nav/_nav_find_path_smoke_test.py
```

追加 API の手動確認（UnrealCV Python）:

```python
from simworld.communicator.unrealcv import UnrealCV
import json
ucv = UnrealCV()
actor = "BP_NavQueryService_C_0"  # 環境に合わせて変更
print(ucv.client.request(f"vbp {actor} NavRebuild"))
print(ucv.client.request(f"vbp {actor} GetActorBoundsJson site20_prop_001"))
```

`NavRebuild` が `{"ok":true}` を返せば Phase 1 準備完了です。

## Step 7 — ミッション実行

```bash
conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh
```

## 計画クリアランス（Modifier 膨らませ）

**制約**: プロップ表面 ↔ SpotDog **中心** ≥ **100 cm**

| 項目 | 値 | 説明 |
|------|-----|------|
| `GetActorBounds` | プロップ AABB | NavModifier ボックス＝**実形状境界**（追加膨らませなし） |
| `NavFindPath` AgentRadius | **100 cm** | 経路は障害表面から中心 100 cm 以上離れる |
| violation 計測 | AABB 表面距離 | Phase 3（Python `surface_distance.py`） |

Humanoid（Leg2）は `site20_humanoid_nav_obs` として動的ボックスを更新し、同じ 100 cm AgentRadius で再計画します。

## トラブルシュート

| 症状 | 対処 |
|------|------|
| `extended NavQueryService API missing` | Step 1–4 を再実行（古い C++ のまま） |
| `start_not_on_navmesh` | NavMesh Bounds / Build Paths、ロボット開始位置を確認 |
| `actor_not_found` | スポーン後に `site20_prop_*` が存在するか PIE で確認 |
| `no_path` | 障害物が多すぎる／AgentRadius 100 cm で通路が塞がれている → レイアウトまたは通路幅を確認 |
