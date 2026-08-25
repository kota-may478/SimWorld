# NavQueryService — SimWorld Source への組み込み

Phase 2 の 2-1 で追加する C++ ファイルです。UE プロジェクトの Source へコピーしてリビルドしてください。

## 1. ファイルをコピー

| コピー元（本リポジトリ） | コピー先（UE プロジェクト） |
| --- | --- |
| `dev/grid_env_level_nav/ue_native/NavQueryService.h` | `Source/SimWorld/Public/NavQueryService.h` |
| `dev/grid_env_level_nav/ue_native/NavQueryService.cpp` | `Source/SimWorld/Private/NavQueryService.cpp` |
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.h` | `Source/SimWorld/Public/SpotDogNavController.h` |
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.cpp` | `Source/SimWorld/Private/SpotDogNavController.cpp` |

モジュール名が `SimWorld` でない場合はパスを読み替えてください。

## 2. `SimWorld.Build.cs` に依存を追加

`PrivateDependencyModuleNames` に次を追加（未追加の場合のみ）:

```csharp
"NavigationSystem",
"AIModule",
"Json",          // Phase 5 SpotDogNavController (NavFollowPathJson)
"JsonUtilities",
```

## 3. Visual Studio で Rebuild

1. UE Editor を終了
2. `.sln` を開き **Rebuild**
3. Editor 再起動

## 4. Blueprint 作成

`dev/grid_env_level_nav/create_nav_query_service_editor.py` を UE Editor で実行するか、手動で `BP_NavQueryService` を作成してください（#320 Phase 2 の 2-2）。

## 5. 確認

PIE 開始後、WSL で:

```bash
conda activate simworld
cd ~/01_Private/Program/SimWorld
python dev/grid_env_level_nav/_nav_project_point_smoke_test.py
```
