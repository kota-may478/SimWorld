# SimWorld `dev/` — Development Projects Index

## English

This directory contains **local research and integration prototypes** that extend the SimWorld Python client with Unreal Engine scenarios. Each subdirectory is a self-contained project with its own `README.md`.

### Project map

| Project | Map / environment | Primary focus |
|---------|-------------------|---------------|
| [`grid_env_hri/`](grid_env_hri/README.md) | `/Game/Maps/empty` | Shared 30 m grid floor + 10k transparent cubes, Humanoid, SpotDog helpers |
| [`grid_env_10k/`](grid_env_10k/README.md) | `empty` → `grid_100x100` | 100×100 semi-transparent block grid, patrol, four-rooms layout |
| [`grid_env_10k_semantic/`](grid_env_10k_semantic/README.md) | `grid_100x100` corner | floor / air / wall semantic labeling on the 10k grid |
| [`grid_env_level_semantic/`](grid_env_level_semantic/README.md) | `/Game/Maps/Level` | Semantic block layer on Level map → `Level_semantic` |
| [`grid_env_level_nav/`](grid_env_level_nav/README.md) | `/Game/Maps/Level` | L0/L1/L2 layered navigation, costmaps, material-transport scenarios |
| [`grid_env_depth_perception/`](grid_env_depth_perception/README.md) | `/Game/Maps/Level` | Depth + `object_mask` prop recognition and NavMesh navigation |
| [`llm_material_transport/`](llm_material_transport/README.md) | `/Game/Maps/Level` | LLM-directed Humanoid → SpotDog material transport |
| [`hri_agv/`](hri_agv/README.md) | UE Play mode (generic) | Proxemics-based Humanoid–AGV interaction in 10 m room |
| [`hri_spotdog_follow/`](hri_spotdog_follow/README.md) | UE Play mode (generic) | SpotDog camera/depth follow of walking Humanoid |

### Dependency graph (read-only imports)

```
grid_env_hri  ─────────────────────────────────────────┐
     │                                                    │
     ├── grid_env_10k ── grid_env_10k_semantic           │
     │                                                    │
     ├── grid_env_depth_perception ──┐                    │
     │                               │                    │
     └── grid_env_level_nav ◄────────┴── llm_material_transport
              │
              └── (uses grid_env_10k_pie_patrol helpers, depth_object_perception)

hri_agv ──► hri_spotdog_follow (room layout parity)
```

### Conventions

- **UE connection**: UnrealCV listens on TCP port **9000**; only one Python client at a time. Use `release_ue_connection.py` or restart the Jupyter kernel before CLI scripts. See `grid_env_hri/ue_client_guard.py`.
- **PIE vs Editor**: Most WSL scripts expect **Play-In-Editor (PIE)** on the target map. Editor-only scripts live under `*/scripts/*_editor.py`.
- **Caches**: Run artifacts and registries go under each project's `cache/` (or dot-registries like `.level_semantic_registry.json`). Safe to delete for a clean rerun.
- **Entry points**: Prefer documented `run_*.py` / scenario `run_test.py` over notebooks for reproducible runs. Notebooks are for interactive exploration.
- **Tests**: `python -m unittest discover -s dev/<project> -p 'test_*.py' -v` where tests exist.

### Organizational notes

- `grid_env_level_nav/scenarios/` groups end-to-end scenarios (`compact_nav`, `site_transport_20m`, `construction_site`). Root-level `run_*.py` files are thin backward-compatible wrappers.
- `grid_env_hri` is a **shared library**, not a standalone simulation notebook; other projects import it.
- `grid_env_level_nav/scenarios/construction_site/placement.py` re-exports root `construction_site_placement.py` (removed ~370-line duplicate).

---

## 日本語

このディレクトリは SimWorld Python クライアントを拡張する **ローカル研究・統合プロトタイプ** を格納します。各サブディレクトリは独立プロジェクトで、それぞれ `README.md` を持ちます。

### プロジェクト一覧

| プロジェクト | マップ / 環境 | 主な内容 |
|-------------|--------------|---------|
| [`grid_env_hri/`](grid_env_hri/README.md) | `/Game/Maps/empty` | 30 m グリッド床・1 万透明キューブ・Humanoid/SpotDog 共通ヘルパ |
| [`grid_env_10k/`](grid_env_10k/README.md) | `empty` → `grid_100x100` | 100×100 半透明ブロック格子・パトロール・四部屋レイアウト |
| [`grid_env_10k_semantic/`](grid_env_10k_semantic/README.md) | `grid_100x100` 隅 | 1 万格子への floor/air/wall 意味ラベル付け |
| [`grid_env_level_semantic/`](grid_env_level_semantic/README.md) | `/Game/Maps/Level` | Level 上の意味ブロック層 → `Level_semantic` 保存 |
| [`grid_env_level_nav/`](grid_env_level_nav/README.md) | `/Game/Maps/Level` | L0/L1/L2 層ナビ・コストマップ・資材運搬シナリオ |
| [`grid_env_depth_perception/`](grid_env_depth_perception/README.md) | `/Game/Maps/Level` | 深度 + `object_mask` による認識と NavMesh ナビ |
| [`llm_material_transport/`](llm_material_transport/README.md) | `/Game/Maps/Level` | LLM 指示による Humanoid → SpotDog 資材搬送 |
| [`hri_agv/`](hri_agv/README.md) | UE Play（汎用） | 10 m 部屋での Proxemics ベース HRI |
| [`hri_spotdog_follow/`](hri_spotdog_follow/README.md) | UE Play（汎用） | 歩行 Humanoid を SpotDog がカメラ/深度で追従 |

### 依存関係（インポート方向）

上記「Dependency graph」と同じ構造です。`grid_env_hri` が複数プロジェクトの共通基盤、`grid_env_level_nav` が Level 系シナリオの中核です。

### 規約

- **UE 接続**: UnrealCV は TCP **9000**、同時 1 クライアント。CLI 前に `release_ue_connection.py` または Jupyter カーネル再起動。`grid_env_hri/ue_client_guard.py` 参照。
- **PIE と Editor**: WSL スクリプトは多くが対象マップの **PIE** 前提。Editor 専用は `*/scripts/*_editor.py`。
- **キャッシュ**: 実行成果物は各プロジェクトの `cache/` 等。削除して再実行可能。
- **エントリ**: 再現性には `run_*.py` / シナリオの `run_test.py` を推奨。ノートブックは対話的探索用。
- **テスト**: `python -m unittest discover -s dev/<project> -p 'test_*.py' -v`

### 構成上の注意

- `grid_env_level_nav/scenarios/` に E2E シナリオを集約。ルートの `run_*.py` は後方互換ラッパ。
- `grid_env_hri` は **共有ライブラリ**（単体ノートブックシミュレーションではない）。
- `scenarios/construction_site/placement.py` はルート `construction_site_placement.py` の再エクスポート（重複コード削除済み）。
