# ---
# jupyter:
#   jupytext:
#     cell_metadata_filter: -all
#     formats: ipynb,py:percent
#     notebook_metadata_filter: -all,kernelspec,jupytext
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: simworld
#     language: python
#     name: python3
# ---

# %% [markdown]
# # AGV-Human HRI Simulation — 10 m × 10 m Square Room (UE連携版)
#
# **前提**: Unreal Engine でレベルを Play 状態にしてから実行してください。
#
# **処理フロー**
# 1. UE へ接続 (UnrealCV)
# 2. 壁4辺・人間 (Humanoid)・AGV (SpotDog) を明示的にスポーン
# 3. マルチスレッドで人間・AGV を並列制御しながら HRI データを記録
# 4. UE 接続を切断
# 5. 記録データから HRI メトリクスを算出・可視化
#
# **HRI ゾーン定義 (Proxemics 理論)**
# | Zone     | 距離      | AGV の挙動              |
# |----------|-----------|-------------------------|
# | Safety   | < 0.5 m   | 緊急停止                |
# | Personal | 0.5–1.2 m | スロー走行              |
# | Social   | 1.2–3.0 m | 速度を徐々に上げる      |
# | Far      | > 3.0 m   | 最大速度で走行          |

# %%
import math
import sys
import threading
import time
from pathlib import Path
from typing import List, Tuple

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# SimWorld ルートをパスに追加 (dev/hri_agv/ の 2 階層上)
sys.path.append(str(Path().resolve().parent.parent))

from simworld.agent.humanoid import Humanoid
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.utils.vector import Vector

plt.style.use("seaborn-v0_8-whitegrid")

# %%
# --- UE 接続 ---
# Unreal Engine でレベルを Play してから、このセルを実行してください
ucv = UnrealCV()
communicator = Communicator(ucv)


# %%
# ==============================================================
# シミュレーション設定
# ==============================================================

# ---- 部屋 (UE 座標: cm 単位, 10 m = 1000 cm) ----
ROOM_CM        = 1000   # 部屋の一辺 [cm]
WALL_MARGIN_CM = 80     # エージェントが壁から保つ最小距離 [cm]

# ---- 壁アセット設定 ----
# NOTE: WALL_BP_PATH は BP_Interactable_Box を流用。
#       もし環境に存在しない場合は、100×100×100 cm のキューブ BP に変更してください。
WALL_BP_PATH  = "/Game/InteractableAsset/Box/BP_Interactable_Box.BP_Interactable_Box_C"
WALL_THICK_CM = 20    # 壁の厚み [cm]
WALL_H_CM     = 300   # 壁の高さ [cm] (3 m)
WALL_Z_CM     = 0     # 壁のスポーン高さ(中心) [cm]
# 壁アセットのデフォルトサイズが 100×100×100 cm の場合のスケール係数
# 異なる場合はこの値を調整してください
WALL_ASSET_SIZE_CM = 100
# 1 枚を極端に伸ばすと BP 側で伸長が制限されるケースがあるため、
# 壁を複数セグメントに分割して並べる
WALL_SEGMENT_LEN_CM = 100      # 1 セグメントの長さ [cm]
WALL_SEGMENT_OVERLAP_CM = 25   # セグメント重なり [cm]

# ---- エージェントアセットパス ----
# NOTE: humanoid_step_forward / humanoid_rotate を使う場合は Base_User_Agent 系 BP が必要。
HUMAN_BP_PATH = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"
ROBOT_BP_PATH = "/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C"
ROBOT_NAME    = "AGV_SpotRobot"

# ---- スポーン位置 (x, y, z) [cm] ----
HUMAN_SPAWN = (750, 200, 100)   # z=100: Humanoid の標準スポーン高さ
ROBOT_SPAWN = (200, 800, 20)    # z=20:  SpotDog の標準スポーン高さ

# ---- 速度設定 ----
HUMAN_SPEED    = 180   # humanoid の移動速度 (UE 単位)
AGV_SPEED_MAX  = 200   # AGV 最大速度 (far zone)
AGV_SPEED_SLOW = 60    # AGV スロー速度 (social zone)
AGV_CRUISE_SPEED = 140  # 直進巡航速度 [UE 単位]

# ---- HRI ゾーン境界 [cm] ----
SAFETY_R_CM   = 50    # 0.5 m
PERSONAL_R_CM = 120   # 1.2 m
SOCIAL_R_CM   = 300   # 3.0 m

# ---- 制御パラメータ ----
STEP_DUR      = 0.5    # 1 ステップの移動時間 [s]
ROTATE_THR    = 25.0   # この角度差以上でのみ回転 [deg]
WP_REACH_CM   = 80.0   # ウェイポイント到達判定距離 [cm]
COLLISION_MOVE_EPS_CM = 5.0   # この距離未満しか進まなければ衝突とみなす [cm]
TURN_MIN_DEG  = 45.0   # 衝突時の最小回頭角 [deg]
TURN_MAX_DEG  = 150.0  # 衝突時の最大回頭角 [deg]

# ---- シミュレーション時間 ----
SIM_DURATION  = 20.0   # 総記録時間 [s]

rng = np.random.default_rng(42)


# %%
# ==============================================================
# ユーティリティ関数
# ==============================================================

def random_waypoint() -> Tuple[float, float]:
    """壁マージンを考慮した室内ランダムウェイポイントを生成 [cm]。"""
    lo = WALL_MARGIN_CM + 50
    hi = ROOM_CM - WALL_MARGIN_CM - 50
    return (float(rng.uniform(lo, hi)), float(rng.uniform(lo, hi)))


def normalize_angle(deg: float) -> float:
    """角度を [-180, 180] に正規化。"""
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def get_pos2d(actor_name: str) -> Tuple[float, float]:
    """UE アクターの XY 位置 [cm] を取得。"""
    loc = ucv.get_location(actor_name)
    return (float(loc[0]), float(loc[1]))


def get_yaw(actor_name: str) -> float:
    """UE アクターの Yaw 角 [deg] を取得。"""
    ori = ucv.get_orientation(actor_name)
    return float(ori[1])


def yaw_to_target(from_xy: Tuple[float, float], to_xy: Tuple[float, float]) -> float:
    """ある位置からターゲットへの方位角 [deg] を計算。"""
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    return math.degrees(math.atan2(dy, dx))


def hri_zone(dist_cm: float) -> str:
    """距離 [cm] から HRI ゾーンラベルを返す。"""
    if dist_cm < SAFETY_R_CM:
        return "safety"
    elif dist_cm < PERSONAL_R_CM:
        return "personal"
    elif dist_cm < SOCIAL_R_CM:
        return "social"
    return "far"


# %%
# ==============================================================
# ワールドスポーン
# ==============================================================

def spawn_walls():
    """
    部屋の 4 辺の壁をスポーンする。
    ルーム: x, y ∈ [0, ROOM_CM] (cm)
    壁はセグメントを並べて 10m 四方を確実に囲む。
    """
    R = ROOM_CM
    T = WALL_THICK_CM
    H = WALL_H_CM
    S = WALL_ASSET_SIZE_CM
    z_center = WALL_Z_CM

    # 既存の WALL_* を削除して、再実行時の重複・検証ブレを防ぐ
    for obj in ucv.get_objects():
        if str(obj).startswith("WALL_"):
            ucv.destroy(str(obj))

    seg_len = min(float(WALL_SEGMENT_LEN_CM), float(R))
    seg_overlap = min(float(WALL_SEGMENT_OVERLAP_CM), seg_len - 1.0)

    # 角から角までを厳密に埋める中心位置を作る。
    # 端点(0 と R)に対して、最初と最後のセグメント端がぴったり一致するように配置する。
    if R <= seg_len:
        edge_centers = [R / 2]
        step = 0.0
    else:
        advance_target = max(1.0, seg_len - seg_overlap)
        n_segments = int(math.ceil((R - seg_len) / advance_target)) + 1
        step = (R - seg_len) / (n_segments - 1)
        edge_centers = [seg_len / 2 + i * step for i in range(n_segments)]

    walls = []
    for i, c in enumerate(edge_centers):
        walls.append((f"WALL_South_{i:02d}", (c,       -T / 2,     z_center), (seg_len / S, T / S, H / S)))
        walls.append((f"WALL_North_{i:02d}", (c,        R + T / 2, z_center), (seg_len / S, T / S, H / S)))
        walls.append((f"WALL_West_{i:02d}",  (-T / 2,   c,         z_center), (T / S, seg_len / S, H / S)))
        walls.append((f"WALL_East_{i:02d}",  (R + T / 2, c,        z_center), (T / S, seg_len / S, H / S)))

    # 四隅を角壁で閉じて、角の微小隙間を防ぐ
    corner_scale = (T / S, T / S, H / S)
    walls.extend([
        ("WALL_Corner_SW", (-T / 2,    -T / 2,    z_center), corner_scale),
        ("WALL_Corner_SE", (R + T / 2, -T / 2,    z_center), corner_scale),
        ("WALL_Corner_NW", (-T / 2,    R + T / 2, z_center), corner_scale),
        ("WALL_Corner_NE", (R + T / 2, R + T / 2, z_center), corner_scale),
    ])

    print(
        f"  Wall side={R:.1f} cm, seg_len={seg_len:.1f} cm, "
        f"overlap~{(seg_len - step):.1f} cm, n_segments/edge={len(edge_centers)}"
    )

    for name, loc, scale in walls:
        ucv.spawn_bp_asset(WALL_BP_PATH, name)
        ucv.set_location(loc, name)
        ucv.set_orientation((0, 0, 0), name)
        ucv.set_scale(scale, name)
        ucv.set_collision(name, True)
        ucv.set_movable(name, False)   # 壁は動かない
        print(f"  Spawned {name} at {loc}")


def spawn_human() -> Humanoid:
    """人間 (Humanoid) をスポーンして Humanoid オブジェクトを返す。"""
    human = Humanoid(
        position=Vector(HUMAN_SPAWN[0], HUMAN_SPAWN[1]),
        direction=Vector(1, 0),
    )
    communicator.spawn_agent(
        agent=human,
        name=None,
        position=HUMAN_SPAWN,
        model_path=HUMAN_BP_PATH,
        type="humanoid",
    )
    communicator.humanoid_set_speed(human.id, HUMAN_SPEED)
    print(f"  Spawned human [{communicator.get_humanoid_name(human.id)}] at {HUMAN_SPAWN[:2]}")
    return human


def spawn_robot(name: str) -> str:
    """AGV (SpotDog) をスポーンしてアクター名を返す。"""
    ucv.spawn_bp_asset(ROBOT_BP_PATH, name)
    ucv.set_location(ROBOT_SPAWN, name)
    ucv.set_orientation((0, 90, 0), name)   # +Y 方向を向いてスポーン
    ucv.enable_controller(name, True)
    print(f"  Spawned robot [{name}] at {ROBOT_SPAWN[:2]}")
    return name


# %%
print("=== Spawning world ===")

print("[Walls]")
spawn_walls()

print("[Human]")
human = spawn_human()
HUMAN_NAME = communicator.get_humanoid_name(human.id)

print("[Robot]")
spawn_robot(ROBOT_NAME)

time.sleep(3.0)   # スポーン完了 + 物理安定待ち
print(f"\n=== Spawn complete  human={HUMAN_NAME}  robot={ROBOT_NAME} ===")


# %%
# ==============================================================
# シミュレーション制御ループ
# ==============================================================

sim_data:  List[dict]   = []
stop_event = threading.Event()


# ---- 人間制御スレッド ----
def human_control_loop():
    """
    直進し続け、何かに当たって進めなかったらランダム回頭して再び直進する。
    """
    while not stop_event.is_set():
        prev_pos = get_pos2d(HUMAN_NAME)
        if not stop_event.is_set():
            communicator.humanoid_step_forward(human.id, STEP_DUR)

        if stop_event.is_set():
            break

        curr_pos = get_pos2d(HUMAN_NAME)
        moved_cm = math.hypot(curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1])

        # 前進量が極端に小さいときは何かに当たっているとみなしてランダム回頭
        if moved_cm < COLLISION_MOVE_EPS_CM:
            turn_deg = float(rng.uniform(TURN_MIN_DEG, TURN_MAX_DEG))
            direction = 'left' if rng.random() < 0.5 else 'right'
            communicator.humanoid_rotate(human.id, turn_deg, direction)


# ---- AGV 制御スレッド ----
def agv_control_loop():
    """
    直進し続け、何かに当たって進めなかったらランダム回頭して再び直進する。
    """
    while not stop_event.is_set():
        prev_pos = get_pos2d(ROBOT_NAME)

        if stop_event.is_set():
            break

        ucv.dog_move(ROBOT_NAME, [AGV_CRUISE_SPEED, STEP_DUR, 0])

        if stop_event.is_set():
            break

        curr_pos = get_pos2d(ROBOT_NAME)
        moved_cm = math.hypot(curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1])

        # 前進量が極端に小さいときは何かに当たっているとみなしてランダム回頭
        if moved_cm < COLLISION_MOVE_EPS_CM:
            turn_deg = float(rng.uniform(TURN_MIN_DEG, TURN_MAX_DEG))
            clockwise = 1 if rng.random() < 0.5 else -1
            ucv.dog_rotate(ROBOT_NAME, [0.3, turn_deg, clockwise])


# ---- データ記録スレッド ----
def recorder_loop():
    """
    STEP_DUR 秒ごとに両エージェントの位置・距離・ゾーンを記録する。
    """
    t_start = time.time()
    while not stop_event.is_set():
        t         = time.time() - t_start
        robot_pos = get_pos2d(ROBOT_NAME)
        human_pos = get_pos2d(HUMAN_NAME)
        dist_cm   = math.hypot(
            human_pos[0] - robot_pos[0],
            human_pos[1] - robot_pos[1],
        )
        sim_data.append({
            "t":           t,
            "human_x":     human_pos[0],
            "human_y":     human_pos[1],
            "robot_x":     robot_pos[0],
            "robot_y":     robot_pos[1],
            "distance_cm": dist_cm,
            "distance_m":  dist_cm / 100.0,
            "zone":        hri_zone(dist_cm),
        })
        time.sleep(STEP_DUR)


# %%
# ==============================================================
# シミュレーション実行
# ==============================================================
print(f"=== Starting simulation ({SIM_DURATION:.0f} s) ===")

t_human    = threading.Thread(target=human_control_loop, daemon=True)
t_robot    = threading.Thread(target=agv_control_loop,   daemon=True)
t_recorder = threading.Thread(target=recorder_loop,       daemon=True)

t_human.start()
t_robot.start()
t_recorder.start()

# SIM_DURATION 秒後に全スレッドへ停止シグナルを送る
time.sleep(SIM_DURATION)
stop_event.set()

# スレッド終了待ち (最大 5 秒)
for t in [t_human, t_robot, t_recorder]:
    t.join(timeout=5)

print(f"=== Simulation finished — {len(sim_data)} data points recorded ===")


# %%
# ==============================================================
# クリーンアップ
# ==============================================================
communicator.humanoid_stop(human.id)   # 人間を停止

# UE 接続を切断
# NOTE: 切断前に UE ウィンドウを閉じないでください (クラッシュする可能性があります)
communicator.disconnect()
print("Disconnected from UE.")


# %%
# ==============================================================
# データ整形
# ==============================================================
df = pd.DataFrame(sim_data)
ROOM_M = ROOM_CM / 100.0   # 10.0 m

# m 単位の列を追加
df["human_x_m"] = df["human_x"] / 100.0
df["human_y_m"] = df["human_y"] / 100.0
df["robot_x_m"] = df["robot_x"] / 100.0
df["robot_y_m"] = df["robot_y"] / 100.0

df.head(10)


# %%
# ==============================================================
# HRI メトリクスサマリー
# ==============================================================

def hri_metrics(df: pd.DataFrame) -> pd.DataFrame:
    dt = STEP_DUR
    zone_time = df.groupby("zone").size() * dt

    def count_entries(dist_m_series: pd.Series, threshold_m: float) -> int:
        return int(((dist_m_series < threshold_m).astype(int).diff() == 1).sum())

    metrics = {
        "duration_s":                float(df["t"].max()),
        "n_samples":                 len(df),
        "mean_distance_m":           float(df["distance_m"].mean()),
        "min_distance_m":            float(df["distance_m"].min()),
        "max_distance_m":            float(df["distance_m"].max()),
        "time_safety_zone_s":        float(zone_time.get("safety",   0)),
        "time_personal_zone_s":      float(zone_time.get("personal", 0)),
        "time_social_zone_s":        float(zone_time.get("social",   0)),
        "time_far_zone_s":           float(zone_time.get("far",      0)),
        "social_encounters":         count_entries(df["distance_m"], SOCIAL_R_CM / 100),
        "personal_space_violations": count_entries(df["distance_m"], PERSONAL_R_CM / 100),
        "safety_violations":         count_entries(df["distance_m"], SAFETY_R_CM / 100),
    }
    return pd.DataFrame(list(metrics.items()), columns=["metric", "value"])


summary = hri_metrics(df)
summary


# %%
# ==============================================================
# 可視化 (4 パネル)
# ==============================================================

ZONE_COLORS = {
    "far":      "#4caf50",
    "social":   "#2196f3",
    "personal": "#ff9800",
    "safety":   "#f44336",
}
ZONE_ORDER = ["far", "social", "personal", "safety"]

fig = plt.figure(figsize=(15, 10))
gs  = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)

# ---- Panel 1: 軌跡マップ ----
ax1 = fig.add_subplot(gs[0, 0])
room_patch = plt.Polygon(
    [[0, 0], [ROOM_M, 0], [ROOM_M, ROOM_M], [0, ROOM_M]],
    fill=False, edgecolor="dimgray", linewidth=3, zorder=1,
)
ax1.add_patch(room_patch)

ax1.plot(df["human_x_m"], df["human_y_m"],
         color="steelblue", linewidth=0.7, alpha=0.5, label="Human path", zorder=2)

for zone in ZONE_ORDER:
    mask = df["zone"] == zone
    ax1.scatter(df.loc[mask, "robot_x_m"], df.loc[mask, "robot_y_m"],
                c=ZONE_COLORS[zone], s=4, alpha=0.7, label=f"AGV ({zone})", zorder=3)

ax1.scatter(HUMAN_SPAWN[0] / 100, HUMAN_SPAWN[1] / 100,
            marker="o", s=120, c="steelblue", edgecolors="white", linewidth=1.5,
            zorder=6, label="Human spawn")
ax1.scatter(ROBOT_SPAWN[0] / 100, ROBOT_SPAWN[1] / 100,
            marker="s", s=120, c="coral", edgecolors="white", linewidth=1.5,
            zorder=6, label="AGV spawn")

ax1.set_xlim(-0.5, ROOM_M + 0.5)
ax1.set_ylim(-0.5, ROOM_M + 0.5)
ax1.set_aspect("equal")
ax1.set_title("Trajectories (AGV colored by HRI zone)")
ax1.set_xlabel("x [m]")
ax1.set_ylabel("y [m]")
ax1.legend(loc="upper right", fontsize=7, markerscale=2)

# ---- Panel 2: 距離の時系列 ----
ax2 = fig.add_subplot(gs[0, 1])

ax2.fill_between(df["t"], 0, SAFETY_R_CM / 100,
                 alpha=0.20, color=ZONE_COLORS["safety"],   label="Safety zone")
ax2.fill_between(df["t"], SAFETY_R_CM / 100, PERSONAL_R_CM / 100,
                 alpha=0.15, color=ZONE_COLORS["personal"],  label="Personal zone")
ax2.fill_between(df["t"], PERSONAL_R_CM / 100, SOCIAL_R_CM / 100,
                 alpha=0.10, color=ZONE_COLORS["social"],    label="Social zone")

ax2.plot(df["t"], df["distance_m"], color="black", linewidth=1.2, label="Distance")
ax2.axhline(SAFETY_R_CM / 100,   color=ZONE_COLORS["safety"],   linestyle="--", linewidth=1)
ax2.axhline(PERSONAL_R_CM / 100, color=ZONE_COLORS["personal"],  linestyle="--", linewidth=1)
ax2.axhline(SOCIAL_R_CM / 100,   color=ZONE_COLORS["social"],    linestyle="--", linewidth=1)

ax2.set_title("Robot-Human Distance over Time")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("distance [m]")
ax2.legend(fontsize=8)

# ---- Panel 3: ゾーン別滞在時間 (棒グラフ) ----
ax3 = fig.add_subplot(gs[1, 0])
zone_times = {z: float((df["zone"] == z).sum() * STEP_DUR) for z in ZONE_ORDER}
ax3.bar(
    list(zone_times.keys()),
    list(zone_times.values()),
    color=[ZONE_COLORS[z] for z in ZONE_ORDER],
)
ax3.set_title("Time Spent in Each HRI Zone")
ax3.set_xlabel("Zone")
ax3.set_ylabel("time [s]")

# ---- Panel 4: ゾーンタイムライン ----
ax4 = fig.add_subplot(gs[1, 1])
zone_to_y = {"far": 0, "social": 1, "personal": 2, "safety": 3}
ax4.scatter(
    df["t"],
    df["zone"].map(zone_to_y),
    c=[ZONE_COLORS[z] for z in df["zone"]],
    s=3, alpha=0.7,
)
ax4.set_yticks([0, 1, 2, 3])
ax4.set_yticklabels(["Far", "Social", "Personal", "Safety"])
ax4.set_title("HRI Zone Timeline")
ax4.set_xlabel("time [s]")
legend_patches = [
    mpatches.Patch(color=ZONE_COLORS[z], label=z.capitalize()) for z in ZONE_ORDER
]
ax4.legend(handles=legend_patches, fontsize=8)

plt.suptitle(
    f"AGV-Human HRI Simulation (UE) — {ROOM_M:.0f} m × {ROOM_M:.0f} m Room  "
    f"({SIM_DURATION:.0f} s)",
    fontsize=13, fontweight="bold",
)
plt.show()

# %% [markdown]
# ## 次の拡張案
#
# ### アセット設定の調整
# - `WALL_BP_PATH` を実際の環境に存在するキューブ/壁アセットのパスに変更
# - `WALL_ASSET_SIZE_CM` を実際のアセットサイズに合わせてスケールを調整
# - `HUMAN_BP_PATH` / `ROBOT_BP_PATH` を使用しているキャラクターに合わせて変更
#
# ### シミュレーション拡張
# - `SIM_DURATION` を延ばして長時間の HRI を観測
# - 人間の行動を「歩行」「立ち止まる」「方向転換」などのステートマシンに拡張
# - AGV に搬送タスク（PickUp → Delivery）を持たせる
# - 複数人・複数 AGV へのスケールアップ
#
# ### 解析拡張
# - 接近イベントの個別抽出と継続時間分布
# - AGV の速度プロファイル（人間との距離との相関）
# - エゴカメラ画像の取得と可視化
