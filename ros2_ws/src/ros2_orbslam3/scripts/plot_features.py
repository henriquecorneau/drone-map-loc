#!/usr/bin/env python3
"""Plot detected vs. matched ORB features per frame from a mono localization run.

Usage: plot_features.py <stats.csv> [--output plot.png] [--no-show]

The CSV is written by the `mono` node on shutdown (see --features-csv).
"""

import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt

# ORB_SLAM3::Tracking::eTrackingState values that mean the frame was tracked.
TRACKING_OK_STATES = {2, 5}  # OK, OK_KLT

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e6e5e0"
DETECTED_COLOR = "#2a78d6"
MATCHED_COLOR = "#eb6834"
LOST_COLOR = "#ecebe6"


def load_stats(path):
    stamps, detected, matched, states = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            stamps.append(float(row["stamp"]))
            detected.append(int(row["detected"]))
            matched.append(int(row["matched"]))
            states.append(int(row["state"]))
    t0 = stamps[0] if stamps else 0.0
    return [s - t0 for s in stamps], detected, matched, states


def lost_spans(times, states):
    """Yield (start, end) time spans where tracking was not OK."""
    start = None
    for t, state in zip(times, states):
        if state not in TRACKING_OK_STATES and start is None:
            start = t
        elif state in TRACKING_OK_STATES and start is not None:
            yield start, t
            start = None
    if start is not None:
        yield start, times[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", help="feature stats CSV written by the mono node")
    parser.add_argument("--output", help="PNG path (default: next to the CSV)")
    parser.add_argument("--no-show", action="store_true", help="only save the PNG")
    args = parser.parse_args()

    times, detected, matched, states = load_stats(args.csv)
    if not times:
        raise SystemExit(f"No frames in {args.csv}")

    if args.no_show or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")

    fig, ax = plt.subplots(figsize=(11, 5), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    for i, (start, end) in enumerate(lost_spans(times, states)):
        ax.axvspan(start, end, color=LOST_COLOR, lw=0,
                   label="Tracking lost" if i == 0 else None)

    ax.plot(times, detected, color=DETECTED_COLOR, lw=2, label="Detected features")
    ax.plot(times, matched, color=MATCHED_COLOR, lw=2,
            label="Features matched to map points")

    ok = [i for i, s in enumerate(states) if s in TRACKING_OK_STATES]
    mean_det = sum(detected) / len(detected)
    mean_match = sum(matched[i] for i in ok) / len(ok) if ok else 0.0
    ax.set_title(
        f"Localization: features per frame  —  {len(times)} frames, "
        f"{100 * len(ok) / len(times):.0f}% tracked, "
        f"mean {mean_det:.0f} detected / {mean_match:.0f} matched (tracked frames)",
        loc="left", color=TEXT_PRIMARY, fontsize=11)

    ax.set_xlabel("Time since first frame (s)", color=TEXT_SECONDARY)
    ax.set_ylabel("Number of features", color=TEXT_SECONDARY)
    ax.set_xlim(times[0], times[-1])
    # The first frames use ORB_SLAM3's initializer extractor (5x nFeatures);
    # clip that spike so the rest of the run stays readable.
    p99 = sorted(detected + matched)[int(0.99 * (2 * len(times) - 1))]
    ax.set_ylim(0, max(p99, 1) * 1.15)
    ax.grid(axis="y", color=GRID, lw=1)
    ax.tick_params(colors=TEXT_SECONDARY)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(TEXT_SECONDARY)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3,
              frameon=False, labelcolor=TEXT_PRIMARY)
    fig.tight_layout()

    output = args.output or os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(output, dpi=150, facecolor=SURFACE)
    print(f"Plot saved to {output}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()
