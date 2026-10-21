#!/usr/bin/env python3
"""Top-down map of a mono localization run: where the drone was tracked and lost.

Usage: plot_localization_map.py <stats.csv> [--output map.png] [--no-show]

Reads the files the `mono` node writes on shutdown:
  <stats>.csv              per-frame features, tracking state and T_world_camera
  <stats>_keyframes.csv    keyframes of every map in the atlas
  <stats>_map_points.csv   map points of every map in the atlas
"""

import argparse
import csv
import os
from collections import Counter

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from scipy.spatial.transform import Rotation

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
MAP_POINTS = "#c3c2b7"
KEYFRAME_PATH = "#52514e"
GOOD = "#0ca30c"
CRITICAL = "#d03b3b"
# Sequential blue ramp (steps 250 -> 700) for the matched feature count.
MATCHED_CMAP = LinearSegmentedColormap.from_list(
    "matched", ["#86b6ef", "#3987e5", "#256abf", "#184f95", "#0d366b"])

# Points farther than this above/below the camera heights are floor/ceiling
# clutter in a top-down view.
HEIGHT_BAND_M = 1.0


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pose_arrays(rows):
    """Positions (N, 3) and rotations R_world_camera (N, 3, 3) from x..qw columns."""
    t = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rows])
    q_xyzw = np.array([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in rows])
    return t, Rotation.from_quat(q_xyzw).as_matrix()


def ground_basis(R_world_camera):
    """Rows (e1, e2, up) of a world->top-down rotation.

    ORB_SLAM3's world frame is the first keyframe's optical frame (y down), so
    the vertical is estimated as the mean camera "down" axis (column 1 of
    R_world_camera) of the mapping keyframes, assuming the Kinect was held
    roughly level. e1 is the mean viewing direction projected on the ground.
    """
    up = -R_world_camera[:, :, 1].mean(axis=0)
    up /= np.linalg.norm(up)
    forward = R_world_camera[:, :, 2].mean(axis=0)
    e1 = forward - forward.dot(up) * up
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    return np.vstack([e1, e2, up])


def tracking_events(times, tracked):
    """Yield (kind, frame index, time): 'lost' at the last tracked frame, 'relocalized' at the first."""
    for i in range(1, len(tracked)):
        if tracked[i - 1] and not tracked[i]:
            yield "lost", i - 1, times[i]
        elif not tracked[i - 1] and tracked[i]:
            yield "relocalized", i, times[i]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", help="feature stats CSV written by the mono node")
    parser.add_argument("--output", help="PNG path (default: <stats>_map.png)")
    parser.add_argument("--no-show", action="store_true", help="only save the PNG")
    args = parser.parse_args()

    stem = os.path.splitext(args.csv)[0]
    frames = read_rows(args.csv)
    keyframes = read_rows(stem + "_keyframes.csv")
    map_points = read_rows(stem + "_map_points.csv")
    if not frames or "map_id" not in frames[0]:
        raise SystemExit(f"{args.csv} has no poses; re-run localization with the current mono node")

    if args.no_show or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")

    t0 = float(frames[0]["stamp"])
    times = [float(r["stamp"]) - t0 for r in frames]
    tracked = [r["map_id"] != "" for r in frames]
    if not any(tracked):
        raise SystemExit("The drone was never tracked; nothing to place on the map")

    # Plot the map the drone was localized in for most frames.
    map_counts = Counter(r["map_id"] for r in frames if r["map_id"])
    map_id = map_counts.most_common(1)[0][0]
    if len(map_counts) > 1:
        print(f"Drone was tracked in maps {dict(map_counts)}; plotting map {map_id}")

    kf_rows = sorted((r for r in keyframes if r["map_id"] == map_id), key=lambda r: int(r["id"]))
    kf_t, kf_R = pose_arrays(kf_rows)
    basis = ground_basis(kf_R)

    def to_ground(p):
        return p @ basis.T  # columns: e1, e2, height

    kf_g = to_ground(kf_t)
    points_g = to_ground(np.array([[float(r[k]) for k in ("x", "y", "z")]
                                   for r in map_points if r["map_id"] == map_id]))

    # Drone frames tracked in other maps have poses in another frame: skip them.
    on_map = [i for i, r in enumerate(frames) if r["map_id"] == map_id]
    drone_t, _ = pose_arrays([frames[i] for i in on_map])
    drone_g = to_ground(drone_t)
    pose_of = {i: drone_g[k] for k, i in enumerate(on_map)}
    matched = np.array([int(frames[i]["matched"]) for i in on_map])

    heights = np.concatenate([kf_g[:, 2], drone_g[:, 2]])
    in_band = ((points_g[:, 2] > heights.min() - HEIGHT_BAND_M) &
               (points_g[:, 2] < heights.max() + HEIGHT_BAND_M))
    points_g = points_g[in_band]

    fig, ax = plt.subplots(figsize=(10, 9), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    ax.scatter(points_g[:, 0], points_g[:, 1], s=1, color=MAP_POINTS, lw=0,
               label=f"Map points ({len(points_g)}, ±{HEIGHT_BAND_M:.0f} m of camera height)")
    ax.plot(kf_g[:, 0], kf_g[:, 1], color=KEYFRAME_PATH, lw=1, ls=":",
            label=f"Mapping keyframes, Kinect ({len(kf_g)})")

    sc = ax.scatter(drone_g[:, 0], drone_g[:, 1], c=matched, cmap=MATCHED_CMAP, s=10, lw=0,
                    vmin=0, vmax=np.percentile(matched, 99), zorder=3,
                    label="Drone, tracked (color = matched features)")
    cbar = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.02)
    cbar.set_label("Features matched to map points", color=TEXT_SECONDARY)
    cbar.outline.set_visible(False)

    start = drone_g[0]
    ax.scatter(*start[:2], marker="o", s=90, facecolor=SURFACE, edgecolor=TEXT_PRIMARY,
               lw=2, zorder=5, label="Start")

    events = [(kind, pose_of[i], t) for kind, i, t in tracking_events(times, tracked) if i in pose_of]
    lost_label = reloc_label = True
    for k, (kind, p, t) in enumerate(events):
        # The other end of this loss (its relocalization, or the loss before a
        # relocalization); labels point away from it so the two never overlap.
        j = k + 1 if kind == "lost" else k - 1
        partner = events[j] if 0 <= j < len(events) else None
        partner = partner if partner and partner[0] != kind else None
        if kind == "lost":
            ax.scatter(*p[:2], marker="X", s=140, color=CRITICAL, edgecolor=SURFACE, lw=1.5,
                       zorder=6, label="Lost (last tracked pose)" if lost_label else None)
            lost_label = False
            if partner:
                q = partner[1]
                ax.plot([p[0], q[0]], [p[1], q[1]], color=TEXT_SECONDARY, lw=1, ls="--", zorder=4)
        else:
            ax.scatter(*p[:2], marker="^", s=120, color=GOOD, edgecolor=SURFACE, lw=1.5,
                       zorder=6, label="Relocalized (first pose)" if reloc_label else None)
            reloc_label = False

        direction = p[:2] - partner[1][:2] if partner else np.array([1.0, 1.0])
        if np.linalg.norm(direction) < 1e-6:
            direction = np.array([-1.0, 1.0]) if kind == "lost" else np.array([1.0, -1.0])
        offset = 45 * direction / np.linalg.norm(direction)
        ax.annotate(f"{kind} {t:.1f} s", p[:2], xytext=offset, textcoords="offset points",
                    ha="right" if offset[0] < 0 else "left", va="center",
                    fontsize=9, color=TEXT_PRIMARY, zorder=7,
                    bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec="none", alpha=0.9),
                    arrowprops=dict(arrowstyle="-", color=TEXT_SECONDARY, lw=0.8,
                                    shrinkA=0, shrinkB=6))

    n_tracked = sum(tracked)
    ax.set_title(f"Where the drone was localized  —  map {map_id}, "
                 f"{100 * n_tracked / len(frames):.0f}% of {len(frames)} frames tracked, "
                 f"{sum(1 for e in events if e[0] == 'lost')} losses",
                 loc="left", color=TEXT_PRIMARY, fontsize=11)
    ax.set_xlabel("Ground plane, along mean mapping view direction (m)", color=TEXT_SECONDARY)
    ax.set_ylabel("Ground plane, lateral (m)", color=TEXT_SECONDARY)
    ax.set_aspect("equal")

    # Frame the drone path and the mapped area without far outliers.
    xy = np.vstack([points_g[:, :2], kf_g[:, :2], drone_g[:, :2]])
    lo, hi = np.percentile(xy, 1, axis=0), np.percentile(xy, 99, axis=0)
    lo = np.minimum(lo, drone_g[:, :2].min(axis=0)) - 0.5
    hi = np.maximum(hi, drone_g[:, :2].max(axis=0)) + 0.5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])

    ax.grid(color="#e6e5e0", lw=1)
    ax.tick_params(colors=TEXT_SECONDARY)
    for side in ax.spines.values():
        side.set_visible(False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, frameon=False,
              labelcolor=TEXT_PRIMARY, markerscale=1.5)
    fig.tight_layout()

    output = args.output or stem + "_map.png"
    fig.savefig(output, dpi=150, facecolor=SURFACE, bbox_inches="tight")
    print(f"Map plot saved to {output}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()
