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
# # AGV-Human HRI Simulation — 10 m × 10 m Square Room
#
# **シナリオ**
# - 10 m × 10 m の正方形の部屋（壁4辺）
# - AGV 1 台がランダムウェイポイントをナビゲーション（タスク駆動）
# - 人間 1 名がランダムウォーク
# - AGV はソーシャルフォースモデルで人間を回避・減速・停止
# - HRI メトリクス（接近回数・ゾーン滞在時間・速度プロファイル・接近イベント等）を計測
#
# **HRI ゾーン定義（Proxemics理論に基づく）**
# | Zone | 距離 | 意味 |
# |------|------|------|
# | Safety | < 0.5 m | AGV緊急停止 |
# | Personal | 0.5–1.2 m | 個人空間侵害 |
# | Social | 1.2–3.0 m | 社会的インタラクション圏 |
# | Far | > 3.0 m | 通常走行域 |

# %%
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.style.use("seaborn-v0_8-whitegrid")
rng = np.random.default_rng(42)


# %%
@dataclass
class SimConfig:
    # ---- Room ----
    room_size: float = 10.0
    wall_margin: float = 0.5      # スポーン・ウェイポイントの壁からの最小距離

    # ---- Time ----
    dt: float = 0.1               # シミュレーションステップ [s]
    duration_s: float = 180.0     # 総シミュレーション時間 [s]

    # ---- 速度 ----
    human_speed_mps: float = 1.2   # 人間歩行速度 [m/s]
    agv_max_speed_mps: float = 1.5  # AGV最大速度 [m/s]
    agv_slow_speed_mps: float = 0.4  # AGVスローゾーン速度 [m/s]

    # ---- HRI ゾーン半径 ----
    safety_radius_m: float = 0.5    # 緊急停止境界
    personal_radius_m: float = 1.2  # 個人空間境界
    social_radius_m: float = 3.0    # 社会的距離境界

    # ---- ソーシャルフォース（AGV） ----
    sf_strength: float = 3.0        # 人間からの斥力強度
    sf_sigma_m: float = 1.5         # 斥力の減衰距離 [m]
    wall_sf_strength: float = 2.0   # 壁からの斥力強度
    wall_sf_sigma_m: float = 0.4    # 壁斥力の減衰距離 [m]

    # ---- ウェイポイント ----
    wp_threshold_m: float = 0.4    # ウェイポイント到達判定距離

    # ---- ノイズ ----
    heading_noise_std_rad: float = 0.06

    # ---- 初期スポーン位置 ----
    human_spawn: Tuple[float, float] = (7.5, 2.0)
    agv_spawn: Tuple[float, float] = (2.0, 8.0)


cfg = SimConfig()
N_STEPS = int(cfg.duration_s / cfg.dt)


# %%
# ---- ユーティリティ ----

def unit(v: np.ndarray) -> np.ndarray:
    """正規化ベクトル。ゼロベクトルの場合は(1,0)を返す。"""
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.array([1.0, 0.0])


def rot2d(angle_rad: float) -> np.ndarray:
    """2D回転行列。"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s], [s, c]])


def clip_to_room(pos: np.ndarray, room_size: float) -> np.ndarray:
    """位置を部屋内にクランプ。"""
    return np.clip(pos, 0.0, room_size)


def reflect_velocity(pos: np.ndarray, vel: np.ndarray, room_size: float) -> np.ndarray:
    """壁に当たった場合、速度の該当成分を反転する。"""
    v = vel.copy()
    if pos[0] <= 0.0 or pos[0] >= room_size:
        v[0] = -v[0]
    if pos[1] <= 0.0 or pos[1] >= room_size:
        v[1] = -v[1]
    return v


def random_waypoint(cfg: SimConfig) -> np.ndarray:
    """壁マージンを考慮したランダムなウェイポイントを生成。"""
    lo = cfg.wall_margin + 0.2
    hi = cfg.room_size - cfg.wall_margin - 0.2
    return rng.uniform(lo, hi, size=2)


# %%
# ---- ソーシャルフォースモデル ----

def agent_repulsion(pos: np.ndarray, other_pos: np.ndarray,
                    strength: float, sigma: float) -> np.ndarray:
    """他エージェントからの斥力ベクトル（指数減衰）。"""
    diff = pos - other_pos
    dist = np.linalg.norm(diff)
    if dist < 1e-6:
        # 同一位置の場合はランダム方向に押す
        angle = rng.uniform(0, 2 * math.pi)
        return strength * np.array([math.cos(angle), math.sin(angle)])
    return (strength * math.exp(-dist / sigma)) * (diff / dist)


def wall_repulsion(pos: np.ndarray, room_size: float,
                   strength: float, sigma: float) -> np.ndarray:
    """4辺の壁からの斥力ベクトルの合計。"""
    force = np.zeros(2)
    # (壁までの距離, 斥力方向) の4辺分
    walls = [
        (pos[0],               np.array([1.0,  0.0])),   # 西壁 (x=0)
        (room_size - pos[0],   np.array([-1.0, 0.0])),   # 東壁 (x=room)
        (pos[1],               np.array([0.0,  1.0])),   # 南壁 (y=0)
        (room_size - pos[1],   np.array([0.0, -1.0])),   # 北壁 (y=room)
    ]
    for d, direction in walls:
        if d < sigma * 4:  # 影響範囲内のみ計算
            force += strength * math.exp(-d / sigma) * direction
    return force


# %%
# ---- エージェント状態 ----

@dataclass
class AgentState:
    name: str
    pos: np.ndarray
    direction: np.ndarray   # 単位方向ベクトル
    speed_mps: float = 0.0


def hri_zone(dist: float, cfg: SimConfig) -> str:
    """距離からHRIゾーンラベルを返す。"""
    if dist < cfg.safety_radius_m:
        return "safety"
    elif dist < cfg.personal_radius_m:
        return "personal"
    elif dist < cfg.social_radius_m:
        return "social"
    return "far"


# %%
# ---- エージェント更新関数 ----

def step_human(human: AgentState, target: np.ndarray,
               cfg: SimConfig) -> Tuple[AgentState, np.ndarray]:
    """
    人間を1ステップ更新する。
    ランダムウェイポイントへの指向性歩行 + 方向ノイズ + 壁反射。
    """
    to_target = target - human.pos
    if np.linalg.norm(to_target) < cfg.wp_threshold_m:
        target = random_waypoint(cfg)
        to_target = target - human.pos

    desired_dir = unit(to_target)
    noise = rng.normal(0.0, cfg.heading_noise_std_rad)
    desired_dir = unit(rot2d(noise) @ desired_dir)

    # 壁への軽い斥力（コーナーへの張り付き防止）
    wf = wall_repulsion(human.pos, cfg.room_size,
                        strength=cfg.wall_sf_strength * 0.3,
                        sigma=cfg.wall_sf_sigma_m)
    move_dir = unit(desired_dir + wf * 0.3)

    new_pos = human.pos + move_dir * cfg.human_speed_mps * cfg.dt
    new_pos = clip_to_room(new_pos, cfg.room_size)

    # 壁に当たった場合は方向を反転
    reflected = reflect_velocity(new_pos, move_dir, cfg.room_size)
    new_dir = unit(reflected)

    return AgentState(name="human", pos=new_pos, direction=new_dir,
                      speed_mps=cfg.human_speed_mps), target


def step_agv(agv: AgentState, human: AgentState, waypoint: np.ndarray,
             cfg: SimConfig) -> Tuple[AgentState, np.ndarray]:
    """
    AGVを1ステップ更新する。
    ソーシャルフォースモデル: ウェイポイント引力 + 人間斥力 + 壁斥力。
    人間との距離に応じた速度スケーリングで安全制御を実現。
    """
    to_wp = waypoint - agv.pos
    if np.linalg.norm(to_wp) < cfg.wp_threshold_m:
        waypoint = random_waypoint(cfg)
        to_wp = waypoint - agv.pos

    desired_dir = unit(to_wp)

    # 人間からの斥力
    sf = agent_repulsion(agv.pos, human.pos, cfg.sf_strength, cfg.sf_sigma_m)

    # 壁からの斥力
    wf = wall_repulsion(agv.pos, cfg.room_size,
                        cfg.wall_sf_strength, cfg.wall_sf_sigma_m)

    # 合力から進行方向を決定
    combined = desired_dir + sf + wf
    new_dir = unit(combined)

    # ノイズ付加
    noise = rng.normal(0.0, cfg.heading_noise_std_rad)
    new_dir = unit(rot2d(noise) @ new_dir)

    # 人間との距離に応じた速度制御
    dist_to_human = np.linalg.norm(agv.pos - human.pos)
    if dist_to_human <= cfg.safety_radius_m:
        speed = 0.0  # 緊急停止
    elif dist_to_human <= cfg.personal_radius_m:
        # 個人空間内: 0 → agv_slow_speed_mps へ線形補間
        t = (dist_to_human - cfg.safety_radius_m) / (cfg.personal_radius_m - cfg.safety_radius_m)
        speed = t * cfg.agv_slow_speed_mps
    elif dist_to_human <= cfg.social_radius_m:
        # 社会的距離内: agv_slow_speed_mps → agv_max_speed_mps へ線形補間
        t = (dist_to_human - cfg.personal_radius_m) / (cfg.social_radius_m - cfg.personal_radius_m)
        speed = cfg.agv_slow_speed_mps + t * (cfg.agv_max_speed_mps - cfg.agv_slow_speed_mps)
    else:
        speed = cfg.agv_max_speed_mps

    new_pos = agv.pos + new_dir * speed * cfg.dt
    new_pos = clip_to_room(new_pos, cfg.room_size)

    reflected = reflect_velocity(new_pos, new_dir, cfg.room_size)
    new_dir = unit(reflected)

    return AgentState(name="agv", pos=new_pos, direction=new_dir, speed_mps=speed), waypoint


# %%
# ---- メインシミュレーションループ ----

def simulate(cfg: SimConfig) -> pd.DataFrame:
    """
    シミュレーションを実行し、全ステップのデータをDataFrameで返す。
    """
    human = AgentState(
        name="human",
        pos=np.array(cfg.human_spawn, dtype=float),
        direction=unit(np.array([1.0, -0.5])),
        speed_mps=cfg.human_speed_mps,
    )
    agv = AgentState(
        name="agv",
        pos=np.array(cfg.agv_spawn, dtype=float),
        direction=unit(np.array([0.5, -1.0])),
        speed_mps=0.0,
    )

    human_target = random_waypoint(cfg)
    agv_waypoint = random_waypoint(cfg)

    rows = []
    prev_dist: Optional[float] = None

    for k in range(N_STEPS):
        t = k * cfg.dt

        human, human_target = step_human(human, human_target, cfg)
        agv, agv_waypoint = step_agv(agv, human, agv_waypoint, cfg)

        dist = float(np.linalg.norm(agv.pos - human.pos))
        zone = hri_zone(dist, cfg)

        # 接近速度: d(dist)/dt の近似 (負 = 接近中)
        approach_speed = ((dist - prev_dist) / cfg.dt) if prev_dist is not None else 0.0
        prev_dist = dist

        rows.append({
            "t":                  t,
            "human_x":            human.pos[0],
            "human_y":            human.pos[1],
            "agv_x":              agv.pos[0],
            "agv_y":              agv.pos[1],
            "distance_m":         dist,
            "zone":               zone,
            "agv_speed_mps":      agv.speed_mps,
            "approach_speed_mps": approach_speed,
        })

    return pd.DataFrame(rows)


# %%
df = simulate(cfg)
df.head(10)


# %%
# ---- HRI メトリクスサマリー ----

def hri_metrics(df: pd.DataFrame, cfg: SimConfig) -> pd.DataFrame:
    dt = cfg.dt

    # ゾーン別滞在時間
    zone_time = df.groupby("zone").size() * dt

    # 接近イベント数（ゾーンへの進入回数をカウント）
    def count_entries(series: pd.Series) -> int:
        binary = (series < 1).astype(int)
        return int((binary.diff() == 1).sum())

    social_encounters   = count_entries(df["distance_m"] / cfg.social_radius_m)
    personal_violations = count_entries(df["distance_m"] / cfg.personal_radius_m)
    safety_violations   = count_entries(df["distance_m"] / cfg.safety_radius_m)

    # 接近速度統計（接近中のステップのみ）
    approaching = df[df["approach_speed_mps"] < 0]["approach_speed_mps"]
    mean_approach = float(approaching.mean()) if len(approaching) > 0 else 0.0
    max_approach  = float(approaching.min())  if len(approaching) > 0 else 0.0  # 最大接近速度（最も負）

    metrics = {
        "duration_s":                cfg.duration_s,
        "mean_distance_m":           float(df["distance_m"].mean()),
        "min_distance_m":            float(df["distance_m"].min()),
        "max_distance_m":            float(df["distance_m"].max()),
        "time_safety_zone_s":        float(zone_time.get("safety",   0)),
        "time_personal_zone_s":      float(zone_time.get("personal", 0)),
        "time_social_zone_s":        float(zone_time.get("social",   0)),
        "time_far_zone_s":           float(zone_time.get("far",      0)),
        "social_encounters":         social_encounters,
        "personal_space_violations": personal_violations,
        "safety_violations":         safety_violations,
        "agv_stopped_time_s":        float((df["agv_speed_mps"] == 0.0).sum() * dt),
        "mean_approach_speed_mps":   mean_approach,
        "max_approach_speed_mps":    max_approach,
    }

    return pd.DataFrame(list(metrics.items()), columns=["metric", "value"])


metrics = hri_metrics(df, cfg)
metrics


# %%
# ---- 接近イベント抽出 ----

def extract_encounters(df: pd.DataFrame, cfg: SimConfig) -> pd.DataFrame:
    """
    ソーシャルゾーン (distance < social_radius) への個別の進入イベントを抽出する。
    各イベントに対して: 開始/終了時刻、継続時間、最小距離、ゾーン侵害フラグを記録。
    """
    in_social = (df["distance_m"] < cfg.social_radius_m).to_numpy()
    events = []
    i = 0
    n = len(df)

    while i < n:
        if in_social[i]:
            start_t = df["t"].iloc[i]
            min_dist = df["distance_m"].iloc[i]
            j = i
            while j < n and in_social[j]:
                min_dist = min(min_dist, df["distance_m"].iloc[j])
                j += 1
            end_t = df["t"].iloc[j - 1]
            events.append({
                "start_t":          start_t,
                "end_t":            end_t,
                "duration_s":       end_t - start_t,
                "min_distance_m":   min_dist,
                "personal_invaded": min_dist < cfg.personal_radius_m,
                "safety_invaded":   min_dist < cfg.safety_radius_m,
            })
            i = j
        else:
            i += 1

    return pd.DataFrame(events)


encounters = extract_encounters(df, cfg)
print(f"ソーシャルゾーン接近イベント: {len(encounters)} 件")
encounters


# %%
# ---- 可視化 (4パネル) ----

ZONE_COLORS = {
    "far":      "#4caf50",   # 緑
    "social":   "#2196f3",   # 青
    "personal": "#ff9800",   # オレンジ
    "safety":   "#f44336",   # 赤
}
ZONE_ORDER = ["far", "social", "personal", "safety"]

fig = plt.figure(figsize=(16, 11))
gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)

# ---- Panel 1: 軌跡マップ ----
ax1 = fig.add_subplot(gs[0, 0])

room_rect = plt.Polygon(
    [[0, 0], [cfg.room_size, 0], [cfg.room_size, cfg.room_size], [0, cfg.room_size]],
    fill=False, edgecolor="dimgray", linewidth=3, zorder=1,
)
ax1.add_patch(room_rect)

# 人間の軌跡（薄い線）
ax1.plot(df["human_x"], df["human_y"],
         color="steelblue", linewidth=0.6, alpha=0.5, label="Human path", zorder=2)

# AGVの軌跡をゾーン別に色分け散布図
for zone in ZONE_ORDER:
    mask = df["zone"] == zone
    ax1.scatter(df.loc[mask, "agv_x"], df.loc[mask, "agv_y"],
                c=ZONE_COLORS[zone], s=3, alpha=0.6,
                label=f"AGV ({zone})", zorder=3)

# スポーン位置
ax1.scatter(*cfg.human_spawn, marker="o", s=120, c="steelblue",
            edgecolors="white", linewidth=1.5, zorder=6, label="Human spawn")
ax1.scatter(*cfg.agv_spawn, marker="s", s=120, c="coral",
            edgecolors="white", linewidth=1.5, zorder=6, label="AGV spawn")

ax1.set_xlim(-0.4, cfg.room_size + 0.4)
ax1.set_ylim(-0.4, cfg.room_size + 0.4)
ax1.set_aspect("equal")
ax1.set_title("Trajectories (AGV colored by HRI zone)")
ax1.set_xlabel("x [m]")
ax1.set_ylabel("y [m]")
ax1.legend(loc="upper right", fontsize=7, markerscale=2)

# ---- Panel 2: 距離の時系列 ----
ax2 = fig.add_subplot(gs[0, 1])

ax2.fill_between(df["t"], 0, cfg.safety_radius_m,
                 alpha=0.20, color=ZONE_COLORS["safety"], label="Safety zone")
ax2.fill_between(df["t"], cfg.safety_radius_m, cfg.personal_radius_m,
                 alpha=0.15, color=ZONE_COLORS["personal"], label="Personal zone")
ax2.fill_between(df["t"], cfg.personal_radius_m, cfg.social_radius_m,
                 alpha=0.10, color=ZONE_COLORS["social"], label="Social zone")

ax2.plot(df["t"], df["distance_m"], color="black", linewidth=1.2, label="Distance")
ax2.axhline(cfg.safety_radius_m,  color=ZONE_COLORS["safety"],   linestyle="--", linewidth=1)
ax2.axhline(cfg.personal_radius_m, color=ZONE_COLORS["personal"], linestyle="--", linewidth=1)
ax2.axhline(cfg.social_radius_m,   color=ZONE_COLORS["social"],   linestyle="--", linewidth=1)

ax2.set_title("Robot-Human Distance over Time")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("distance [m]")
ax2.legend(loc="upper right", fontsize=8)

# ---- Panel 3: AGV速度の時系列 ----
ax3 = fig.add_subplot(gs[1, 0])

ax3.plot(df["t"], df["agv_speed_mps"], color="coral", linewidth=1.0, label="AGV speed")
ax3.axhline(cfg.agv_max_speed_mps,  color="coral",  linestyle=":",  linewidth=0.8, alpha=0.6)
ax3.axhline(cfg.agv_slow_speed_mps, color="orange", linestyle=":",  linewidth=0.8, alpha=0.6)
ax3.axhline(0, color="red", linestyle="--", linewidth=1.0, label="Stop (safety)")

# 距離を薄い線で重ねて相関を可視化
ax3_twin = ax3.twinx()
ax3_twin.plot(df["t"], df["distance_m"], color="gray", linewidth=0.6, alpha=0.4, label="Distance [m]")
ax3_twin.set_ylabel("distance [m]", color="gray", fontsize=8)
ax3_twin.tick_params(axis="y", labelcolor="gray", labelsize=7)

ax3.set_title("AGV Speed over Time (with distance overlay)")
ax3.set_xlabel("time [s]")
ax3.set_ylabel("speed [m/s]")
ax3.legend(loc="upper left", fontsize=8)
ax3_twin.legend(loc="upper right", fontsize=7)

# ---- Panel 4: HRI ゾーンタイムライン ----
ax4 = fig.add_subplot(gs[1, 1])

zone_to_y = {"far": 0, "social": 1, "personal": 2, "safety": 3}
point_colors = [ZONE_COLORS[z] for z in df["zone"]]
ax4.scatter(df["t"], df["zone"].map(zone_to_y),
            c=point_colors, s=2, alpha=0.7, zorder=2)

# 接近イベントの開始をマーク
for _, ev in encounters.iterrows():
    ax4.axvline(ev["start_t"], color="black", linewidth=0.5, alpha=0.3, zorder=1)

ax4.set_yticks([0, 1, 2, 3])
ax4.set_yticklabels(["Far", "Social", "Personal", "Safety"])
ax4.set_title("HRI Zone Timeline (| = encounter start)")
ax4.set_xlabel("time [s]")

legend_patches = [
    mpatches.Patch(color=ZONE_COLORS[z], label=z.capitalize()) for z in ZONE_ORDER
]
ax4.legend(handles=legend_patches, loc="upper right", fontsize=8)

plt.suptitle(
    f"AGV-Human HRI Simulation — {cfg.room_size:.0f} m × {cfg.room_size:.0f} m Room  "
    f"({cfg.duration_s:.0f} s)",
    fontsize=13, fontweight="bold",
)
plt.show()

# %% [markdown]
# ## 次の拡張案
#
# ### シミュレーション拡張
# - **複数エージェント**: 人間複数名・AGV複数台へのスケールアップ
# - **人間の行動モード**: 目的地優先、回避行動、立ち止まり等のステートマシン
# - **AGVタスク**: 搬送タスク（PickUp → 目的地 → Delivery）の実装
# - **障害物**: 部屋内のシェルフや柱などの静的障害物追加
#
# ### 解析拡張
# - **相対方位ヒストグラム**: 接近時の相対角度分布
# - **接近イベントの速度プロファイル**: 接近→停止→再加速のパターン分析
# - **フィールドポテンシャル可視化**: ある時刻のソーシャルフォース場の可視化
#
# ### SimWorld / UE 連携
# - スポーンログから Unreal Engine のアクタースポーンコマンドを生成
# - `Communicator` を通じて実際の UE 環境でシミュレーションを実行
# - `Humanoid` クラスの `position` / `direction` を本シミュレーション結果でフィードフォワード
