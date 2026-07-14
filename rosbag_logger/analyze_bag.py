#!/usr/bin/env python3
"""
analyze_bag.py — stability + odometry-drift report for a Livox/FAST_LIO rosbag.

For every topic it prints message count, duration, average rate and the MAX GAP
between consecutive messages (a large gap is the "data stopped sending" symptom
that best-effort/QoS problems produce). For /Odometry it also reconstructs the
trajectory and reports path length, net displacement and max distance from the
start — the numbers that reveal drift — and can save an XY trajectory plot.

Run it AFTER sourcing ROS (needs the message definitions):
    source /opt/ros/humble/setup.bash
    source ~/fast_lio_ws/install/setup.bash
    python3 analyze_bag.py ~/ros2_bags/fastlio_XXXX --plot traj.png

If the sensor was stationary, net/max-dist should stay near 0; growing values
(and which axis grows) tell you the drift magnitude and direction.
"""
import argparse
import atexit
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict


def _prepare_bag(bag_dir: str) -> str:
    """rosbag2_py's SequentialReader cannot read zstd FILE-compressed bags, so if
    this bag is compressed, decompress the splits into a temp dir (auto-removed on
    exit) and return that dir. Otherwise return bag_dir unchanged."""
    comp = glob.glob(os.path.join(bag_dir, "*.zstd"))
    if not comp:
        return bag_dir
    if shutil.which("zstd") is None:
        sys.exit("Bag is zstd-compressed but the 'zstd' CLI is missing "
                 "(sudo apt install zstd) — cannot decompress for analysis.")
    tmp = tempfile.mkdtemp(prefix="bag_analyze_")
    atexit.register(shutil.rmtree, tmp, ignore_errors=True)
    for f in comp:
        out = os.path.join(tmp, os.path.basename(f)[:-len(".zstd")])
        subprocess.run(["zstd", "-d", "-f", "-q", "-o", out, f], check=True)
    meta = open(os.path.join(bag_dir, "metadata.yaml")).read()
    meta = (meta.replace(".db3.zstd", ".db3")
                .replace("compression_format: zstd", "compression_format: ''")
                .replace("compression_mode: FILE", "compression_mode: ''"))
    with open(os.path.join(tmp, "metadata.yaml"), "w") as fh:
        fh.write(meta)
    print(f"(decompressed {len(comp)} zstd split(s) to a temp dir for reading)")
    return tmp


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="path to the bag directory (the one with metadata.yaml)")
    ap.add_argument("--storage", default="sqlite3", help="storage id (default sqlite3)")
    ap.add_argument("--odom-topic", default="/Odometry")
    ap.add_argument("--plot", metavar="PNG", help="save an XY trajectory plot to this path")
    ap.add_argument("--stall", type=float, default=1.0,
                    help="flag topics whose max gap exceeds this many seconds (default 1.0)")
    args = ap.parse_args()

    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        print(f"ERROR: ROS not sourced ({e}). Run: source /opt/ros/humble/setup.bash",
              file=sys.stderr)
        return 2

    bag_dir = _prepare_bag(args.bag)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=args.storage),
                ConverterOptions("cdr", "cdr"))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    counts = defaultdict(int)
    first_t, last_t, prev_t = {}, {}, {}
    max_gap = defaultdict(float)
    odom = []  # (t, x, y, z)
    odom_cls = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        ts = t_ns * 1e-9
        counts[topic] += 1
        first_t.setdefault(topic, ts)
        last_t[topic] = ts
        if topic in prev_t:
            gap = ts - prev_t[topic]
            if gap > max_gap[topic]:
                max_gap[topic] = gap
        prev_t[topic] = ts
        if topic == args.odom_topic:
            if odom_cls is None:
                odom_cls = get_message(topic_types[topic])
            m = deserialize_message(data, odom_cls)
            p = m.pose.pose.position
            odom.append((ts, p.x, p.y, p.z))

    if not counts:
        print("Bag is empty (no messages recorded). If topics existed, this is almost"
              " certainly a QoS mismatch — check rosbag_record_qos.yaml.", file=sys.stderr)
        return 1

    print("=" * 72)
    print(f"Bag: {args.bag}")
    print("=" * 72)
    print(f"{'topic':<28}{'count':>8}{'dur(s)':>10}{'rate(Hz)':>10}{'maxgap(s)':>12}")
    print("-" * 72)
    for topic in sorted(counts):
        dur = last_t[topic] - first_t[topic]
        rate = counts[topic] / dur if dur > 0 else 0.0
        flag = "  <-- STALL" if max_gap[topic] > args.stall else ""
        print(f"{topic:<28}{counts[topic]:>8}{dur:>10.1f}{rate:>10.2f}"
              f"{max_gap[topic]:>12.3f}{flag}")

    if odom:
        xs = [p[1] for p in odom]
        ys = [p[2] for p in odom]
        zs = [p[3] for p in odom]
        p0 = odom[0][1:]
        path_len = sum(math.dist(odom[i][1:], odom[i - 1][1:]) for i in range(1, len(odom)))
        net = math.dist(odom[-1][1:], p0)
        maxdist = max(math.dist(p[1:], p0) for p in odom)
        print("-" * 72)
        print(f"Odometry drift summary ({args.odom_topic}, frame camera_init):")
        print(f"  samples            : {len(odom)}")
        print(f"  path length        : {path_len:.3f} m")
        print(f"  net displacement   : {net:.3f} m   (start -> end)")
        print(f"  max dist from start: {maxdist:.3f} m")
        print(f"  X span             : {max(xs) - min(xs):.3f} m   [{min(xs):.3f}, {max(xs):.3f}]")
        print(f"  Y span             : {max(ys) - min(ys):.3f} m   [{min(ys):.3f}, {max(ys):.3f}]")
        print(f"  Z span             : {max(zs) - min(zs):.3f} m   [{min(zs):.3f}, {max(zs):.3f}]")
        print("  If STATIONARY: net/max-dist ~0 = stable; growing = drift (see which axis).")

        if args.plot:
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                t0 = odom[0][0]
                fig, ax = plt.subplots(figsize=(7, 7))
                ax.plot(xs, ys, lw=0.6, alpha=0.4, color="gray", zorder=1)
                sc = ax.scatter(xs, ys, c=[p[0] - t0 for p in odom], s=5,
                                cmap="viridis", zorder=2)
                ax.scatter([xs[0]], [ys[0]], c="green", s=70, marker="o",
                           label="start", zorder=3)
                ax.scatter([xs[-1]], [ys[-1]], c="red", s=70, marker="X",
                           label="end", zorder=3)
                ax.set_aspect("equal", "datalim")
                ax.set_xlabel("x [m]")
                ax.set_ylabel("y [m]")
                ax.set_title("FAST_LIO odometry XY trajectory")
                ax.legend(loc="best")
                fig.colorbar(sc, ax=ax, label="t [s]")
                fig.savefig(args.plot, dpi=130, bbox_inches="tight")
                print(f"  trajectory plot    -> {args.plot}")
            except Exception as e:  # noqa: BLE001
                print(f"  (plot skipped: {e})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
