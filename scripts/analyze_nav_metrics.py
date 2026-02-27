#!/usr/bin/env python3
"""
Navigation Metrics Analysis & Visualization Script
===================================================
12개 시나리오 (암석 개수 × 외곽지형 유무 × 태양 고도) 분석

시나리오 구성:
  - 암석 개수: 150, 75, 0
  - 외곽 지형: 있음 / 없음
  - 태양 고도: 고고도(High) / 저고도(Low)

시나리오 순서 (1~6: 외곽지형 O, 7~12: 외곽지형 X):
  1: 외곽O, 암석150, 고고도    7:  외곽X, 암석150, 고고도
  2: 외곽O, 암석150, 저고도    8:  외곽X, 암석150, 저고도
  3: 외곽O, 암석 75, 고고도    9:  외곽X, 암석 75, 고고도
  4: 외곽O, 암석 75, 저고도    10: 외곽X, 암석 75, 저고도
  5: 외곽O, 암석  0, 고고도    11: 외곽X, 암석  0, 고고도
  6: 외곽O, 암석  0, 저고도    12: 외곽X, 암석  0, 저고도
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Patch
from pathlib import Path
import warnings
import sys
import argparse

warnings.filterwarnings("ignore")
plt.rcParams.update({
    "figure.dpi": 120,
    "savefig.dpi": 150,
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "figure.facecolor": "white",
})

# ──────────────────────────────────────────────
# 1. 데이터 로드 및 시나리오 매핑
# ──────────────────────────────────────────────

ALL_WAYPOINTS = [
    (20, -6),
    (27, 12),
    (35, 25),
    (34, 40),
    (14, 45),
]

# 분석 대상 웨이포인트 수 (--max-wp 옵션으로 변경 가능, 기본값 5)
NUM_WP = 5
WAYPOINTS = ALL_WAYPOINTS[:NUM_WP]
WP_LABELS = [f"WP{i+1}\n({x},{y})" for i, (x, y) in enumerate(WAYPOINTS)]


def set_max_wp(n):
    """분석 대상 웨이포인트 수를 설정한다."""
    global NUM_WP, WAYPOINTS, WP_LABELS
    NUM_WP = min(n, len(ALL_WAYPOINTS))
    WAYPOINTS = ALL_WAYPOINTS[:NUM_WP]
    WP_LABELS = [f"WP{i+1}\n({x},{y})" for i, (x, y) in enumerate(WAYPOINTS)]

# 시나리오 메타데이터
SCENARIO_META = {
    1:  {"rocks": 150, "terrain": True,  "sun": "High"},
    2:  {"rocks": 150, "terrain": True,  "sun": "Low"},
    3:  {"rocks": 75,  "terrain": True,  "sun": "High"},
    4:  {"rocks": 75,  "terrain": True,  "sun": "Low"},
    5:  {"rocks": 0,   "terrain": True,  "sun": "High"},
    6:  {"rocks": 0,   "terrain": True,  "sun": "Low"},
    7:  {"rocks": 150, "terrain": False, "sun": "High"},
    8:  {"rocks": 150, "terrain": False, "sun": "Low"},
    9:  {"rocks": 75,  "terrain": False, "sun": "High"},
    10: {"rocks": 75,  "terrain": False, "sun": "Low"},
    11: {"rocks": 0,   "terrain": False, "sun": "High"},
    12: {"rocks": 0,   "terrain": False, "sun": "Low"},
}


def scenario_label(sid):
    m = SCENARIO_META[sid]
    t = "Terrain" if m["terrain"] else "NoTerrain"
    s = "HighSun" if m["sun"] == "High" else "LowSun"
    return f"S{sid}: {t}, Rock{m['rocks']}, {s}"


def scenario_short(sid):
    m = SCENARIO_META[sid]
    t = "T" if m["terrain"] else "F"
    s = "H" if m["sun"] == "High" else "L"
    return f"R{m['rocks']}-{t}-{s}"


def load_and_assign_scenarios(csv_path):
    """CSV 로드 후 시나리오 번호/웨이포인트 번호 할당.

    시나리오는 CSV 행 순서(입력 순서)를 기준으로 구분한다.
    run_id나 timestamp가 아닌 파일에 기록된 순서가 실제 시나리오 순서이다.
    """
    df = pd.read_csv(csv_path)
    # CSV 행 순서를 보존 (run_id/timestamp 정렬하지 않음)
    df = df.reset_index(drop=True)

    # 웨이포인트 매칭: goal_x, goal_y → wp_index (항상 전체 웨이포인트로 매칭)
    def match_wp(row):
        for i, (wx, wy) in enumerate(ALL_WAYPOINTS):
            if abs(row["goal_x"] - wx) < 0.5 and abs(row["goal_y"] - wy) < 0.5:
                return i
        return -1

    df["wp_index"] = df.apply(match_wp, axis=1)

    # 연속된 run들을 시나리오로 그룹핑
    # 규칙: WP 순서가 리셋(현재 wp_index <= 이전 wp_index)되면 새 시나리오
    scenarios = []
    current_scenario = 1
    prev_wp = -1
    for _, row in df.iterrows():
        wp = row["wp_index"]
        if wp <= prev_wp:
            current_scenario += 1
        scenarios.append(current_scenario)
        prev_wp = wp
    df["scenario"] = scenarios

    # 완주 여부 판단: 5개 웨이포인트를 모두 SUCCEEDED 했는지
    df["wp_num"] = df["wp_index"] + 1  # 1-based

    # --max-wp 필터: 지정된 웨이포인트까지만 분석
    df = df[df["wp_num"] <= NUM_WP].reset_index(drop=True)

    return df


def build_scenario_summary(df):
    """시나리오별 요약 테이블 생성"""
    rows = []
    for sid in sorted(df["scenario"].unique()):
        sdf = df[df["scenario"] == sid]
        meta = SCENARIO_META.get(sid, {})
        n_total = len(sdf)
        n_succeeded = (sdf["result"] == "SUCCEEDED").sum()
        n_aborted = (sdf["result"] == "ABORTED").sum()

        # NUM_WP개 웨이포인트 모두 SUCCEEDED인지
        all_succeeded = n_succeeded == NUM_WP and n_total == NUM_WP
        last_wp_reached = sdf[sdf["result"] == "SUCCEEDED"]["wp_num"].max() if n_succeeded > 0 else 0

        # SUCCEEDED runs만으로 통계
        suc = sdf[sdf["result"] == "SUCCEEDED"]
        rows.append({
            "scenario": sid,
            "label": scenario_label(sid),
            "short": scenario_short(sid),
            "rocks": meta.get("rocks", "?"),
            "terrain": meta.get("terrain", "?"),
            "sun": meta.get("sun", "?"),
            "n_waypoints_total": n_total,
            "n_succeeded": n_succeeded,
            "n_aborted": n_aborted,
            "completed": all_succeeded,
            "last_wp_reached": int(last_wp_reached) if not pd.isna(last_wp_reached) else 0,
            "total_duration_sec": suc["duration_sec"].sum(),
            "total_distance_m": suc["distance_traveled_m"].sum(),
            "mean_loc_rmse_3d_m": suc["loc_rmse_3d_m"].mean(),
            "max_loc_rmse_3d_m": suc["loc_rmse_3d_m"].max(),
            "mean_depth_abs_rel": suc["depth_mean_abs_rel"].mean(),
            "mean_depth_rmse_m": suc["depth_mean_rmse_m"].mean(),
            "mean_depth_a1": suc["depth_mean_a1"].mean(),
        })
    return pd.DataFrame(rows)


# ──────────────────────────────────────────────
# 2. 시각화 함수들
# ──────────────────────────────────────────────

# 색상 팔레트
COLORS_ROCK = {150: "#e74c3c", 75: "#f39c12", 0: "#2ecc71"}
COLORS_SUN = {"High": "#FFD700", "Low": "#4169E1"}
COLORS_TERRAIN = {True: "#8B4513", False: "#87CEEB"}


def _scenario_colors(summary):
    """시나리오별 색상 (암석 개수 기준)"""
    return [COLORS_ROCK[r] for r in summary["rocks"]]


def plot_completion_status(summary, ax=None):
    """시나리오별 완주 상태 및 도달 웨이포인트"""
    if ax is None:
        fig, ax = plt.subplots(figsize=(14, 4))

    x = np.arange(len(summary))
    colors = _scenario_colors(summary)

    bars = ax.bar(x, summary["last_wp_reached"], color=colors, edgecolor="black", linewidth=0.5, alpha=0.85)

    # 완주 여부 표시
    for i, (_, row) in enumerate(summary.iterrows()):
        marker = "✓" if row["completed"] else "✗"
        color = "green" if row["completed"] else "red"
        ax.text(i, row["last_wp_reached"] + 0.15, marker, ha="center", va="bottom",
                fontsize=14, fontweight="bold", color=color)
        # ABORTED 수 표시
        if row["n_aborted"] > 0:
            ax.text(i, 0.3, f"Ab:{int(row['n_aborted'])}", ha="center", va="bottom",
                    fontsize=7, color="white", fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Waypoints Reached")
    ax.set_title(f"Completion Status per Scenario (WP1~{NUM_WP})")
    ax.set_ylim(0, NUM_WP + 1)
    ax.axhline(NUM_WP, color="green", linestyle="--", alpha=0.3, label=f"Completion ({NUM_WP} WP)")
    ax.legend(loc="upper right")

    # Legend - rock count
    patches = [Patch(color=c, label=f"Rocks {k}") for k, c in COLORS_ROCK.items()]
    ax.legend(handles=patches, loc="upper left", ncol=3)


def plot_localization_accuracy(df, summary, axes=None):
    """SLAM 위치추정 정확도: RMSE 3D 비교"""
    if axes is None:
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    suc = df[df["result"] == "SUCCEEDED"].copy()

    # (a) 시나리오별 평균 RMSE 3D
    ax = axes[0]
    x = np.arange(len(summary))
    colors = _scenario_colors(summary)
    bars = ax.bar(x, summary["mean_loc_rmse_3d_m"] * 100, color=colors, edgecolor="black",
                  linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Mean Localization RMSE 3D (cm)")
    ax.set_title("SLAM Localization RMSE per Scenario (SUCCEEDED only)")
    for i, v in enumerate(summary["mean_loc_rmse_3d_m"] * 100):
        ax.text(i, v + 0.1, f"{v:.1f}", ha="center", va="bottom", fontsize=7)

    # (b) 웨이포인트별 RMSE 3D 박스플롯
    ax = axes[1]
    wp_data = [suc[suc["wp_num"] == wp]["loc_rmse_3d_m"].values * 100 for wp in range(1, NUM_WP + 1)]
    bp = ax.boxplot(wp_data, labels=WP_LABELS, patch_artist=True, widths=0.6)
    cmap = plt.cm.RdYlGn_r
    for i, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(cmap(i / max(NUM_WP - 1, 1)))
        patch.set_alpha(0.7)
    ax.set_ylabel("Localization RMSE 3D (cm)")
    ax.set_title("Localization RMSE Distribution per Waypoint")


def plot_depth_accuracy(df, summary, axes=None):
    """Depth estimation 정확도 분석"""
    if axes is None:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    suc = df[df["result"] == "SUCCEEDED"].copy()
    x = np.arange(len(summary))
    colors = _scenario_colors(summary)

    # (a) 평균 Abs Rel Error
    ax = axes[0]
    ax.bar(x, summary["mean_depth_abs_rel"] * 1000, color=colors, edgecolor="black",
           linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Depth Abs Rel Error (x1e-3)")
    ax.set_title("Mean Depth Abs Rel Error per Scenario")

    # (b) Mean Depth RMSE
    ax = axes[1]
    ax.bar(x, summary["mean_depth_rmse_m"] * 100, color=colors, edgecolor="black",
           linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Depth RMSE (cm)")
    ax.set_title("Depth RMSE per Scenario")

    # (c) delta < 1.25 (a1 accuracy)
    ax = axes[2]
    ax.bar(x, summary["mean_depth_a1"] * 100, color=colors, edgecolor="black",
           linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("delta < 1.25 accuracy (%)")
    ax.set_title("Depth Accuracy (delta<1.25) per Scenario")
    ax.set_ylim(98, 100.5)


def plot_navigation_efficiency(df, summary, axes=None):
    """주행 효율성 분석: 시간 & 거리"""
    if axes is None:
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    suc = df[df["result"] == "SUCCEEDED"].copy()
    x = np.arange(len(summary))
    colors = _scenario_colors(summary)

    # (a) 총 주행 시간
    ax = axes[0]
    ax.bar(x, summary["total_duration_sec"] / 60, color=colors, edgecolor="black",
           linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Total Duration (min)")
    ax.set_title("Total Navigation Duration per Scenario (SUCCEEDED only)")
    for i, v in enumerate(summary["total_duration_sec"] / 60):
        ax.text(i, v + 0.2, f"{v:.1f}", ha="center", va="bottom", fontsize=7)

    # (b) 총 주행 거리
    ax = axes[1]
    ax.bar(x, summary["total_distance_m"], color=colors, edgecolor="black",
           linewidth=0.5, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("Total Distance (m)")
    ax.set_title("Total Distance Traveled per Scenario (SUCCEEDED only)")
    for i, v in enumerate(summary["total_distance_m"]):
        ax.text(i, v + 0.5, f"{v:.1f}", ha="center", va="bottom", fontsize=7)


def plot_factor_analysis(df, summary, axes=None):
    """팩터별 영향 분석: 암석, 외곽지형, 태양고도"""
    if axes is None:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    suc_summary = summary.copy()

    # (a) 암석 개수별 영향
    ax = axes[0]
    rock_groups = suc_summary.groupby("rocks").agg({
        "mean_loc_rmse_3d_m": "mean",
        "mean_depth_abs_rel": "mean",
        "mean_depth_rmse_m": "mean",
    }).reset_index()
    rock_x = np.arange(len(rock_groups))
    w = 0.3
    ax.bar(rock_x - w, rock_groups["mean_loc_rmse_3d_m"] * 100, w, label="Loc RMSE 3D (cm)",
           color="#3498db", alpha=0.8)
    ax.bar(rock_x, rock_groups["mean_depth_rmse_m"] * 100, w, label="Depth RMSE (cm)",
           color="#e74c3c", alpha=0.8)
    ax.bar(rock_x + w, rock_groups["mean_depth_abs_rel"] * 1000, w, label="Depth AbsRel (x1e-3)",
           color="#2ecc71", alpha=0.8)
    ax.set_xticks(rock_x)
    ax.set_xticklabels([f"Rocks {int(r)}" for r in rock_groups["rocks"]])
    ax.set_title("Mean Error by Rock Count")
    ax.legend(fontsize=7)

    # (b) 외곽 지형 유무별 영향
    ax = axes[1]
    terrain_groups = suc_summary.groupby("terrain").agg({
        "mean_loc_rmse_3d_m": "mean",
        "mean_depth_abs_rel": "mean",
        "mean_depth_rmse_m": "mean",
    }).reset_index()
    terrain_x = np.arange(len(terrain_groups))
    ax.bar(terrain_x - w, terrain_groups["mean_loc_rmse_3d_m"] * 100, w, label="Loc RMSE 3D (cm)",
           color="#3498db", alpha=0.8)
    ax.bar(terrain_x, terrain_groups["mean_depth_rmse_m"] * 100, w, label="Depth RMSE (cm)",
           color="#e74c3c", alpha=0.8)
    ax.bar(terrain_x + w, terrain_groups["mean_depth_abs_rel"] * 1000, w, label="Depth AbsRel (x1e-3)",
           color="#2ecc71", alpha=0.8)
    ax.set_xticks(terrain_x)
    ax.set_xticklabels(["No Terrain" if not t else "With Terrain" for t in terrain_groups["terrain"]])
    ax.set_title("Mean Error by Outer Terrain")
    ax.legend(fontsize=7)

    # (c) 태양 고도별 영향
    ax = axes[2]
    sun_groups = suc_summary.groupby("sun").agg({
        "mean_loc_rmse_3d_m": "mean",
        "mean_depth_abs_rel": "mean",
        "mean_depth_rmse_m": "mean",
    }).reset_index()
    sun_x = np.arange(len(sun_groups))
    ax.bar(sun_x - w, sun_groups["mean_loc_rmse_3d_m"] * 100, w, label="Loc RMSE 3D (cm)",
           color="#3498db", alpha=0.8)
    ax.bar(sun_x, sun_groups["mean_depth_rmse_m"] * 100, w, label="Depth RMSE (cm)",
           color="#e74c3c", alpha=0.8)
    ax.bar(sun_x + w, sun_groups["mean_depth_abs_rel"] * 1000, w, label="Depth AbsRel (x1e-3)",
           color="#2ecc71", alpha=0.8)
    ax.set_xticks(sun_x)
    ax.set_xticklabels([f"Sun {s}" for s in sun_groups["sun"]])
    ax.set_title("Mean Error by Sun Elevation")
    ax.legend(fontsize=7)


def plot_per_waypoint_heatmap(df, summary):
    """시나리오 × 웨이포인트 히트맵 (Loc RMSE 3D)"""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    suc = df.copy()
    scenario_ids = sorted(suc["scenario"].unique())

    for ax_idx, (metric, title, fmt, cmap_name) in enumerate([
        ("loc_rmse_3d_m", "Localization RMSE 3D (cm)", ".1f", "YlOrRd"),
        ("depth_mean_abs_rel", "Depth Abs Rel Error (x1e-3)", ".1f", "YlOrRd"),
    ]):
        ax = axes[ax_idx]
        matrix = np.full((len(scenario_ids), NUM_WP), np.nan)

        for i, sid in enumerate(scenario_ids):
            sdf = suc[suc["scenario"] == sid]
            for _, row in sdf.iterrows():
                wp = int(row["wp_num"]) - 1
                val = row[metric]
                if metric == "loc_rmse_3d_m":
                    val *= 100  # cm
                elif metric == "depth_mean_abs_rel":
                    val *= 1000
                # ABORTED는 구분 표시
                if row["result"] == "ABORTED":
                    matrix[i, wp] = np.nan  # ABORTED는 빈칸
                else:
                    matrix[i, wp] = val

        im = ax.imshow(matrix, cmap=cmap_name, aspect="auto", interpolation="nearest")
        plt.colorbar(im, ax=ax, shrink=0.8)

        # 셀 값 표시 & ABORTED 마킹
        for i, sid in enumerate(scenario_ids):
            sdf = suc[suc["scenario"] == sid]
            for _, row in sdf.iterrows():
                wp = int(row["wp_num"]) - 1
                if row["result"] == "ABORTED":
                    ax.text(wp, i, "AB", ha="center", va="center", fontsize=7,
                            color="red", fontweight="bold")
                elif not np.isnan(matrix[i, wp]):
                    ax.text(wp, i, f"{matrix[i, wp]:{fmt}}", ha="center", va="center",
                            fontsize=7, color="black" if matrix[i, wp] < np.nanpercentile(matrix, 75) else "white")

            # WP가 아예 없는 경우 (암시적 ABORT)
            existing_wps = set(sdf["wp_num"].values)
            for wp_num in range(1, NUM_WP + 1):
                if wp_num not in existing_wps:
                    ax.text(wp_num - 1, i, "N/A", ha="center", va="center",
                            fontsize=7, color="gray", fontstyle="italic")

        ax.set_xticks(range(NUM_WP))
        ax.set_xticklabels([f"WP{i+1}" for i in range(NUM_WP)])
        ax.set_yticks(range(len(scenario_ids)))
        ax.set_yticklabels([scenario_short(s) for s in scenario_ids])
        ax.set_title(title)


def plot_loc_xyz_breakdown(df, summary, axes=None):
    """X, Y, Z축별 위치추정 오차 분석"""
    if axes is None:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    suc = df[df["result"] == "SUCCEEDED"].copy()
    x = np.arange(len(summary))
    w = 0.25

    # 시나리오별 평균 X, Y, Z RMSE
    for sid in summary["scenario"]:
        sdf = suc[suc["scenario"] == sid]

    xyz_means = []
    for sid in summary["scenario"]:
        sdf = suc[suc["scenario"] == sid]
        xyz_means.append({
            "x": sdf["loc_rmse_x_m"].mean() * 100,
            "y": sdf["loc_rmse_y_m"].mean() * 100,
            "z": sdf["loc_rmse_z_m"].mean() * 100,
        })

    ax = axes[0]
    ax.bar(x - w, [m["x"] for m in xyz_means], w, label="X RMSE", color="#e74c3c", alpha=0.8)
    ax.bar(x, [m["y"] for m in xyz_means], w, label="Y RMSE", color="#3498db", alpha=0.8)
    ax.bar(x + w, [m["z"] for m in xyz_means], w, label="Z RMSE", color="#2ecc71", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([scenario_short(s) for s in summary["scenario"]], rotation=45, ha="right")
    ax.set_ylabel("RMSE (cm)")
    ax.set_title("X/Y/Z Localization RMSE per Scenario")
    ax.legend()

    # 전체 축별 분포
    ax = axes[1]
    data_xyz = [suc["loc_rmse_x_m"].values * 100,
                suc["loc_rmse_y_m"].values * 100,
                suc["loc_rmse_z_m"].values * 100]
    bp = ax.boxplot(data_xyz, labels=["X", "Y", "Z"], patch_artist=True, widths=0.5)
    for patch, color in zip(bp["boxes"], ["#e74c3c", "#3498db", "#2ecc71"]):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    ax.set_ylabel("RMSE (cm)")
    ax.set_title("Overall X/Y/Z RMSE Distribution")

    # 축별 평균 오차 비율 (파이 차트)
    ax = axes[2]
    avg_x = suc["loc_rmse_x_m"].mean()
    avg_y = suc["loc_rmse_y_m"].mean()
    avg_z = suc["loc_rmse_z_m"].mean()
    total = avg_x + avg_y + avg_z
    sizes = [avg_x / total, avg_y / total, avg_z / total]
    ax.pie(sizes, labels=[f"X\n{avg_x*100:.2f}cm", f"Y\n{avg_y*100:.2f}cm", f"Z\n{avg_z*100:.2f}cm"],
           colors=["#e74c3c", "#3498db", "#2ecc71"], autopct="%1.1f%%", startangle=90)
    ax.set_title("Mean RMSE Proportion by Axis")


def plot_paired_comparison(df, summary):
    """고고도 vs 저고도 페어 비교"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))

    pairs = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10), (11, 12)]
    suc = df[df["result"] == "SUCCEEDED"].copy()

    for idx, (s_high, s_low) in enumerate(pairs):
        ax = axes[idx // 3][idx % 3]
        meta = SCENARIO_META[s_high]
        t_label = "Terrain" if meta["terrain"] else "NoTerrain"

        high_data = suc[suc["scenario"] == s_high]
        low_data = suc[suc["scenario"] == s_low]

        metrics_high = {
            "Loc RMSE\n(cm)": high_data["loc_rmse_3d_m"].mean() * 100,
            "Depth AbsRel\n(x1e-3)": high_data["depth_mean_abs_rel"].mean() * 1000,
            "Depth RMSE\n(cm)": high_data["depth_mean_rmse_m"].mean() * 100,
        }
        metrics_low = {
            "Loc RMSE\n(cm)": low_data["loc_rmse_3d_m"].mean() * 100 if len(low_data) > 0 else 0,
            "Depth AbsRel\n(x1e-3)": low_data["depth_mean_abs_rel"].mean() * 1000 if len(low_data) > 0 else 0,
            "Depth RMSE\n(cm)": low_data["depth_mean_rmse_m"].mean() * 100 if len(low_data) > 0 else 0,
        }

        mx = np.arange(len(metrics_high))
        w = 0.35
        ax.bar(mx - w / 2, list(metrics_high.values()), w, label=f"S{s_high} HighSun",
               color=COLORS_SUN["High"], edgecolor="black", linewidth=0.5, alpha=0.85)
        ax.bar(mx + w / 2, list(metrics_low.values()), w, label=f"S{s_low} LowSun",
               color=COLORS_SUN["Low"], edgecolor="black", linewidth=0.5, alpha=0.85)
        ax.set_xticks(mx)
        ax.set_xticklabels(list(metrics_high.keys()))
        ax.set_title(f"{t_label}, Rocks {meta['rocks']}")
        ax.legend(fontsize=7)

    fig.suptitle("High Sun vs Low Sun Paired Comparison", fontsize=14, fontweight="bold")
    plt.tight_layout()


def plot_trajectory_overview(df):
    """2D 경로 개요 (웨이포인트 위치 + 시나리오 결과)"""
    fig, ax = plt.subplots(figsize=(10, 8))

    # 웨이포인트 위치
    wp_x = [w[0] for w in WAYPOINTS]
    wp_y = [w[1] for w in WAYPOINTS]

    # 경로 그리기 (순서대로 연결)
    ax.plot(wp_x, wp_y, "k--", alpha=0.3, linewidth=1, zorder=1)

    # 시나리오별 도달 표시
    for sid in sorted(df["scenario"].unique()):
        sdf = df[df["scenario"] == sid]
        suc_wps = sdf[sdf["result"] == "SUCCEEDED"]["wp_num"].values
        last = max(suc_wps) if len(suc_wps) > 0 else 0
        meta = SCENARIO_META[sid]
        marker = "o" if meta["terrain"] else "s"
        color = COLORS_ROCK[meta["rocks"]]
        # 마지막 도달 WP만 표시 (살짝 오프셋)
        if last > 0:
            wx, wy = WAYPOINTS[last - 1]
            offset_x = (sid - 6.5) * 0.3
            offset_y = (sid - 6.5) * 0.15
            ax.scatter(wx + offset_x, wy + offset_y, c=color, marker=marker,
                       s=60, edgecolors="black", linewidth=0.5, alpha=0.8, zorder=3)

    # 웨이포인트 마커
    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax.scatter(wx, wy, c="black", marker="*", s=200, zorder=5)
        ax.annotate(f"WP{i+1}\n({wx},{wy})", (wx, wy), textcoords="offset points",
                    xytext=(10, 10), fontsize=8, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

    # 범례
    patches = [Patch(color=c, label=f"Rocks {k}") for k, c in COLORS_ROCK.items()]
    from matplotlib.lines import Line2D
    patches.append(Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=8, label="With Terrain"))
    patches.append(Line2D([0], [0], marker="s", color="w", markerfacecolor="gray", markersize=8, label="No Terrain"))
    ax.legend(handles=patches, loc="upper left")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Waypoint Route & Scenario Reach Status")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")


def plot_loc_factor_paired_comparison(df, summary):
    """각 팩터(태양 고도, 암석 수, 외부 지형)에 따른 Localization RMSE 3D 페어/그룹 비교"""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    suc = df[df["result"] == "SUCCEEDED"].copy()

    # 1. Sun Elevation (High vs Low)
    ax = axes[0]
    pairs_sun = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10), (11, 12)]
    labels_sun = ["T, R150", "T, R75", "T, R0", "F, R150", "F, R75", "F, R0"]
    x = np.arange(len(pairs_sun))
    w = 0.35
    high_vals = [suc[suc["scenario"] == p[0]]["loc_rmse_3d_m"].mean() * 100 for p in pairs_sun]
    low_vals = [suc[suc["scenario"] == p[1]]["loc_rmse_3d_m"].mean() * 100 for p in pairs_sun]

    ax.bar(x - w/2, high_vals, w, label="High Sun", color=COLORS_SUN["High"], edgecolor="black", alpha=0.85)
    ax.bar(x + w/2, low_vals, w, label="Low Sun", color=COLORS_SUN["Low"], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_sun, rotation=45, ha="right")
    ax.set_ylabel("Loc RMSE 3D (cm)")
    ax.set_title("Sun Elevation Effect (High vs Low)")
    ax.legend()

    # 2. Rock Count (150 vs 75 vs 0)
    ax = axes[1]
    groups_rock = [(1, 3, 5), (2, 4, 6), (7, 9, 11), (8, 10, 12)]
    labels_rock = ["T, High", "T, Low", "F, High", "F, Low"]
    x = np.arange(len(groups_rock))
    w = 0.25
    r150_vals = [suc[suc["scenario"] == g[0]]["loc_rmse_3d_m"].mean() * 100 for g in groups_rock]
    r75_vals = [suc[suc["scenario"] == g[1]]["loc_rmse_3d_m"].mean() * 100 for g in groups_rock]
    r0_vals = [suc[suc["scenario"] == g[2]]["loc_rmse_3d_m"].mean() * 100 for g in groups_rock]

    ax.bar(x - w, r150_vals, w, label="150 Rocks", color=COLORS_ROCK[150], edgecolor="black", alpha=0.85)
    ax.bar(x, r75_vals, w, label="75 Rocks", color=COLORS_ROCK[75], edgecolor="black", alpha=0.85)
    ax.bar(x + w, r0_vals, w, label="0 Rocks", color=COLORS_ROCK[0], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_rock, rotation=45, ha="right")
    ax.set_ylabel("Loc RMSE 3D (cm)")
    ax.set_title("Rock Count Effect (150 vs 75 vs 0)")
    ax.legend()

    # 3. Terrain (True vs False)
    ax = axes[2]
    pairs_terrain = [(1, 7), (2, 8), (3, 9), (4, 10), (5, 11), (6, 12)]
    labels_terrain = ["R150, High", "R150, Low", "R75, High", "R75, Low", "R0, High", "R0, Low"]
    x = np.arange(len(pairs_terrain))
    w = 0.35
    t_vals = [suc[suc["scenario"] == p[0]]["loc_rmse_3d_m"].mean() * 100 for p in pairs_terrain]
    f_vals = [suc[suc["scenario"] == p[1]]["loc_rmse_3d_m"].mean() * 100 for p in pairs_terrain]

    ax.bar(x - w/2, t_vals, w, label="With Terrain", color=COLORS_TERRAIN[True], edgecolor="black", alpha=0.85)
    ax.bar(x + w/2, f_vals, w, label="No Terrain", color=COLORS_TERRAIN[False], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_terrain, rotation=45, ha="right")
    ax.set_ylabel("Loc RMSE 3D (cm)")
    ax.set_title("Terrain Effect (With vs No Terrain)")
    ax.legend()

    fig.suptitle("Localization Error Breakdown by Factors", fontsize=14, fontweight="bold")
    plt.tight_layout()


def plot_depth_factor_paired_comparison(df, summary):
    """각 팩터(태양 고도, 암석 수, 외부 지형)에 따른 Depth RMSE 페어/그룹 비교"""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    suc = df[df["result"] == "SUCCEEDED"].copy()

    # 1. Sun Elevation (High vs Low)
    ax = axes[0]
    pairs_sun = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10), (11, 12)]
    labels_sun = ["T, R150", "T, R75", "T, R0", "F, R150", "F, R75", "F, R0"]
    x = np.arange(len(pairs_sun))
    w = 0.35
    high_vals = [suc[suc["scenario"] == p[0]]["depth_mean_rmse_m"].mean() * 100 for p in pairs_sun]
    low_vals = [suc[suc["scenario"] == p[1]]["depth_mean_rmse_m"].mean() * 100 for p in pairs_sun]

    ax.bar(x - w/2, high_vals, w, label="High Sun", color=COLORS_SUN["High"], edgecolor="black", alpha=0.85)
    ax.bar(x + w/2, low_vals, w, label="Low Sun", color=COLORS_SUN["Low"], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_sun, rotation=45, ha="right")
    ax.set_ylabel("Depth RMSE (cm)")
    ax.set_title("Sun Elevation Effect (High vs Low)")
    ax.legend()

    # 2. Rock Count (150 vs 75 vs 0)
    ax = axes[1]
    groups_rock = [(1, 3, 5), (2, 4, 6), (7, 9, 11), (8, 10, 12)]
    labels_rock = ["T, High", "T, Low", "F, High", "F, Low"]
    x = np.arange(len(groups_rock))
    w = 0.25
    r150_vals = [suc[suc["scenario"] == g[0]]["depth_mean_rmse_m"].mean() * 100 for g in groups_rock]
    r75_vals = [suc[suc["scenario"] == g[1]]["depth_mean_rmse_m"].mean() * 100 for g in groups_rock]
    r0_vals = [suc[suc["scenario"] == g[2]]["depth_mean_rmse_m"].mean() * 100 for g in groups_rock]

    ax.bar(x - w, r150_vals, w, label="150 Rocks", color=COLORS_ROCK[150], edgecolor="black", alpha=0.85)
    ax.bar(x, r75_vals, w, label="75 Rocks", color=COLORS_ROCK[75], edgecolor="black", alpha=0.85)
    ax.bar(x + w, r0_vals, w, label="0 Rocks", color=COLORS_ROCK[0], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_rock, rotation=45, ha="right")
    ax.set_ylabel("Depth RMSE (cm)")
    ax.set_title("Rock Count Effect (150 vs 75 vs 0)")
    ax.legend()

    # 3. Terrain (True vs False)
    ax = axes[2]
    pairs_terrain = [(1, 7), (2, 8), (3, 9), (4, 10), (5, 11), (6, 12)]
    labels_terrain = ["R150, High", "R150, Low", "R75, High", "R75, Low", "R0, High", "R0, Low"]
    x = np.arange(len(pairs_terrain))
    w = 0.35
    t_vals = [suc[suc["scenario"] == p[0]]["depth_mean_rmse_m"].mean() * 100 for p in pairs_terrain]
    f_vals = [suc[suc["scenario"] == p[1]]["depth_mean_rmse_m"].mean() * 100 for p in pairs_terrain]

    ax.bar(x - w/2, t_vals, w, label="With Terrain", color=COLORS_TERRAIN[True], edgecolor="black", alpha=0.85)
    ax.bar(x + w/2, f_vals, w, label="No Terrain", color=COLORS_TERRAIN[False], edgecolor="black", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels_terrain, rotation=45, ha="right")
    ax.set_ylabel("Depth RMSE (cm)")
    ax.set_title("Terrain Effect (With vs No Terrain)")
    ax.legend()

    fig.suptitle("Depth Error Breakdown by Factors", fontsize=14, fontweight="bold")
    plt.tight_layout()


def print_summary_table(summary):
    """콘솔에 요약 테이블 출력"""
    print("\n" + "=" * 120)
    print("  Scenario Summary")
    print("=" * 120)
    header = (f"{'Scenario':<35} {'Done':>4} {'WP':>6} {'Ab':>3} "
              f"{'Dur(min)':>8} {'Dist(m)':>8} "
              f"{'LocRMSE(cm)':>11} {'DepAbsRel':>10} {'DepRMSE(cm)':>11} {'Dep_a1(%)':>10}")
    print(header)
    print("-" * 120)
    for _, row in summary.iterrows():
        comp = "  Y" if row["completed"] else "  N"
        print(f"{row['label']:<35} {comp:>4} {int(row['last_wp_reached']):>5}/{NUM_WP} {int(row['n_aborted']):>3} "
              f"{row['total_duration_sec']/60:>8.1f} {row['total_distance_m']:>8.1f} "
              f"{row['mean_loc_rmse_3d_m']*100:>11.2f} {row['mean_depth_abs_rel']*1000:>10.2f} "
              f"{row['mean_depth_rmse_m']*100:>11.2f} {row['mean_depth_a1']*100:>10.3f}")
    print("=" * 120)

    # Factor summary
    print("\n  Factor-wise Mean Comparison (SUCCEEDED runs only)")
    print("-" * 80)

    for factor, label in [("rocks", "Rock Count"), ("terrain", "Outer Terrain"), ("sun", "Sun Elevation")]:
        print(f"\n  > {label}:")
        grp = summary.groupby(factor).agg({
            "mean_loc_rmse_3d_m": "mean",
            "mean_depth_abs_rel": "mean",
            "mean_depth_rmse_m": "mean",
            "mean_depth_a1": "mean",
            "completed": "sum",
        })
        for val, row in grp.iterrows():
            if factor == "terrain":
                val_str = "Yes" if val else "No"
            elif factor == "sun":
                val_str = "High" if val == "High" else "Low"
            else:
                val_str = f"{int(val)}"
            print(f"    {val_str:>8}: LocRMSE={row['mean_loc_rmse_3d_m']*100:.2f}cm  "
                  f"DepAbsRel={row['mean_depth_abs_rel']*1000:.2f}  "
                  f"DepRMSE={row['mean_depth_rmse_m']*100:.2f}cm  "
                  f"Dep_a1={row['mean_depth_a1']*100:.3f}%  "
                  f"Completed={int(row['completed'])}")


# ──────────────────────────────────────────────
# 3. 메인 실행
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Navigation Metrics Analysis")
    parser.add_argument("--max-wp", type=int, default=5, choices=range(1, 6),
                        help="분석할 최대 웨이포인트 번호 (1~5, 기본값: 5)")
    parser.add_argument("--csv", type=str, default=None,
                        help="입력 CSV 파일 경로 (기본: navigation_metrics_0212.csv)")
    args = parser.parse_args()

    # 웨이포인트 수 설정
    set_max_wp(args.max_wp)
    print(f"Analyzing up to WP{NUM_WP} ({NUM_WP} waypoints)")

    csv_path = Path(args.csv) if args.csv else Path(__file__).parent.parent / "data" / "navigation_metrics_0212.csv"
    if not csv_path.exists():
        print(f"Error: {csv_path} not found")
        sys.exit(1)

    output_dir = Path(__file__).parent.parent / "data" / "analysis_output"
    output_dir.mkdir(exist_ok=True)

    print(f"Loading data: {csv_path}")
    df = load_and_assign_scenarios(csv_path)
    summary = build_scenario_summary(df)

    # Scenario assignment check
    print(f"\nTotal {len(df)} runs, {df['scenario'].nunique()} scenarios detected")
    for sid in sorted(df["scenario"].unique()):
        sdf = df[df["scenario"] == sid]
        wps = sdf["wp_num"].tolist()
        results = sdf["result"].tolist()
        status = " ".join([f"WP{w}({'OK' if r == 'SUCCEEDED' else 'AB'})" for w, r in zip(wps, results)])
        print(f"  Scenario {sid:>2} [{scenario_short(sid)}]: {status}")

    # Console summary
    print_summary_table(summary)

    # -- Visualization --
    print(f"\nGenerating charts...")

    # Fig 1: 완주 상태
    fig1, ax1 = plt.subplots(figsize=(14, 4))
    plot_completion_status(summary, ax1)
    fig1.tight_layout()
    fig1.savefig(output_dir / "01_completion_status.png")
    print(f"  ✅ 01_completion_status.png")

    # Fig 2: 위치추정 정확도
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    plot_localization_accuracy(df, summary, axes2)
    fig2.tight_layout()
    fig2.savefig(output_dir / "02_localization_accuracy.png")
    print(f"  ✅ 02_localization_accuracy.png")

    # Fig 3: Depth 정확도
    fig3, axes3 = plt.subplots(1, 3, figsize=(16, 5))
    plot_depth_accuracy(df, summary, axes3)
    fig3.tight_layout()
    fig3.savefig(output_dir / "03_depth_accuracy.png")
    print(f"  ✅ 03_depth_accuracy.png")

    # Fig 4: 주행 효율성
    fig4, axes4 = plt.subplots(1, 2, figsize=(14, 5))
    plot_navigation_efficiency(df, summary, axes4)
    fig4.tight_layout()
    fig4.savefig(output_dir / "04_navigation_efficiency.png")
    print(f"  ✅ 04_navigation_efficiency.png")

    # Fig 5: 팩터 분석
    fig5, axes5 = plt.subplots(1, 3, figsize=(16, 5))
    plot_factor_analysis(df, summary, axes5)
    fig5.tight_layout()
    fig5.savefig(output_dir / "05_factor_analysis.png")
    print(f"  ✅ 05_factor_analysis.png")

    # Fig 6: 히트맵
    plot_per_waypoint_heatmap(df, summary)
    plt.tight_layout()
    plt.savefig(output_dir / "06_heatmap.png")
    print(f"  ✅ 06_heatmap.png")

    # Fig 7: X/Y/Z축 분석
    fig7, axes7 = plt.subplots(1, 3, figsize=(16, 5))
    plot_loc_xyz_breakdown(df, summary, axes7)
    fig7.tight_layout()
    fig7.savefig(output_dir / "07_xyz_breakdown.png")
    print(f"  ✅ 07_xyz_breakdown.png")

    # Fig 8: 페어 비교
    plot_paired_comparison(df, summary)
    plt.savefig(output_dir / "08_paired_comparison.png")
    print(f"  ✅ 08_paired_comparison.png")

    # Fig 9: 경로 개요
    plot_trajectory_overview(df)
    plt.tight_layout()
    plt.savefig(output_dir / "09_trajectory_overview.png")
    print(f"  ✅ 09_trajectory_overview.png")

    # Fig 10: 팩터별 페어/그룹 비교 (Loc RMSE)
    plot_loc_factor_paired_comparison(df, summary)
    plt.savefig(output_dir / "10_loc_factor_paired_comparison.png")
    print(f"  ✅ 10_loc_factor_paired_comparison.png")

    # Fig 11: 팩터별 페어/그룹 비교 (Depth RMSE)
    plot_depth_factor_paired_comparison(df, summary)
    plt.savefig(output_dir / "11_depth_factor_paired_comparison.png")
    print(f"  ✅ 11_depth_factor_paired_comparison.png")

    plt.close("all")

    # Save summary CSV
    summary.to_csv(output_dir / "scenario_summary.csv", index=False)
    print(f"\nSummary CSV saved: {output_dir / 'scenario_summary.csv'}")
    print(f"All charts saved to: {output_dir}/")
    print("\nAnalysis complete!")


if __name__ == "__main__":
    main()
