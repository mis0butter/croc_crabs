"""Extract an RSS hardware run (.mcap) into one .npz of time series for system ID.

Reads the ROS 2 messages straight from the .mcap (the CSV export truncates
/han/joint_states to 2 joints and drops /chewie/joint_states), and writes:

  {arm}_t, {arm}_q, {arm}_v, {arm}_effort      joint states, URDF joint order (J1..J6)
  {arm}_wrench_t, {arm}_wrench                 /rss/slide_{arm}_dyn_input_wrench [Fx Fy Fz Tx Ty Tz]
  slide_wrench_t, slide_wrench                 /rss/slide_dyn_input_wrench
  platform_t, platform                         /rss/slide_platform_state [x y z roll pitch yaw aux]
  tf/<parent>/<child>_t, tf/<parent>/<child>   dynamic /tf as [x y z qx qy qz qw]
  tf_static/<parent>/<child>                   /tf_static as [x y z qx qy qz qw]

Every *_t is in seconds on the logger clock (mcap log_time). Header stamps are
kept as *_stamp where the message has one; see the clock report printed at the end.

Usage:
  python sysid/extract_mcap.py rss_platform/mcap_data/junette_0_3000kg_0/junette_0_3000kg_0.mcap
"""
import argparse
import collections
import os

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

ARMS = ("han", "chewie")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def extract(path):
    js = {arm: [] for arm in ARMS}
    wrench = collections.defaultdict(list)
    platform = []
    tf = collections.defaultdict(list)
    tf_static = {}
    frames = {}

    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, channel, message, msg in reader.iter_decoded_messages():
            topic, t = channel.topic, message.log_time * 1e-9
            if topic in ("/han/joint_states", "/chewie/joint_states"):
                arm = topic.split("/")[1]
                names = [f"{arm}_{j}_joint" for j in JOINTS]
                if not set(names) <= set(msg.name):
                    continue  # gripper-only messages
                idx = [list(msg.name).index(n) for n in names]
                js[arm].append((t, stamp(msg.header),
                                [msg.position[i] for i in idx],
                                [msg.velocity[i] for i in idx],
                                [msg.effort[i] for i in idx]))
            elif topic.endswith("dyn_input_wrench"):
                key = topic.split("/")[-1].replace("_dyn_input_wrench", "")  # slide, slide_han, slide_chewie
                w = msg.wrench
                wrench[key].append((t, stamp(msg.header), [w.force.x, w.force.y, w.force.z,
                                                           w.torque.x, w.torque.y, w.torque.z]))
                frames[key] = msg.header.frame_id
            elif topic == "/rss/slide_platform_state":
                platform.append((t, [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.aux]))
            elif topic in ("/tf", "/tf_static"):
                for tr in msg.transforms:
                    p, r = tr.transform.translation, tr.transform.rotation
                    row = [p.x, p.y, p.z, r.x, r.y, r.z, r.w]
                    key = f"{tr.header.frame_id}/{tr.child_frame_id}"
                    if topic == "/tf":
                        tf[key].append((t, row))
                    else:
                        tf_static[key] = row

    out = {}
    for arm in ARMS:
        t, st, q, v, e = zip(*js[arm])
        out[f"{arm}_t"], out[f"{arm}_stamp"] = np.array(t), np.array(st)
        out[f"{arm}_q"], out[f"{arm}_v"], out[f"{arm}_effort"] = np.array(q), np.array(v), np.array(e)
    for key, rows in wrench.items():
        name = {"slide": "slide", "slide_han": "han", "slide_chewie": "chewie"}[key]
        t, st, w = zip(*rows)
        out[f"{name}_wrench_t"], out[f"{name}_wrench_stamp"], out[f"{name}_wrench"] = map(np.array, (t, st, w))
        out[f"{name}_wrench_frame"] = np.array(frames[key])
    t, p = zip(*platform)
    out["platform_t"], out["platform"] = np.array(t), np.array(p)
    for key, rows in tf.items():
        t, x = zip(*rows)
        out[f"tf/{key}_t"], out[f"tf/{key}"] = np.array(t), np.array(x)
    for key, row in tf_static.items():
        out[f"tf_static/{key}"] = np.array(row)
    t0 = min(v[0] for k, v in out.items() if k.endswith("_t"))
    out["t0"] = np.array(t0)   # all *_t / *_stamp arrays keep absolute time; subtract t0 to plot
    return out


def report(out):
    print(f"{'series':28s} {'samples':>8s} {'rate [Hz]':>10s} {'span [s]':>9s} {'stamp - log [ms]':>18s}")
    for k in sorted(k for k in out if k.endswith("_t") and not k.startswith("tf")):
        t = out[k]
        base = k[:-2]
        lag = ""
        if f"{base}_stamp" in out:
            d = (out[f"{base}_stamp"] - t) * 1e3
            lag = f"{np.median(d):+8.1f} ± {np.std(d):5.1f}"
        print(f"{base:28s} {len(t):8d} {1 / np.median(np.diff(t)):10.1f} {t[-1] - t[0]:9.2f} {lag:>18s}")
    n_tf = sum(1 for k in out if k.startswith("tf/") and k.endswith("_t"))
    print(f"/tf: {n_tf} dynamic transforms, /tf_static: {sum(1 for k in out if k.startswith('tf_static/'))} static")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("-o", "--out", help="output .npz (default: next to the .mcap)")
    args = ap.parse_args()
    out = extract(args.mcap)
    dst = args.out or os.path.splitext(args.mcap)[0] + ".npz"
    np.savez_compressed(dst, **out)
    report(out)
    print(f"wrote {dst}")
