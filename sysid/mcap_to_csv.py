#!/usr/bin/env python3
"""
Dump ROS 2 topics from an .mcap file to one CSV per topic.

    pip install mcap mcap-ros2-support

    # list what's inside
    python mcap_to_csv.py junette_0_3000kg_0.mcap --list

    # everything
    python mcap_to_csv.py junette_0_3000kg_0.mcap -o csv/

    # just the topics you care about
    python mcap_to_csv.py junette_0_3000kg_0.mcap -o csv/ \
        -t /rss/slide_dyn_input_wrench /rss/slide_platform_state

Nested fields become dotted columns (wrench.force.x), fixed/variable arrays
become indexed columns (position.0, position.1, ...).
"""

import argparse
import csv
import re
import sys
from collections import OrderedDict
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

MAX_ARRAY = 64  # arrays longer than this get dumped as a single string cell


def sub_fields(obj):
    """Return {name: value} if obj is a decoded ROS message, else None."""
    slots = getattr(obj, "__slots__", None)
    if slots:
        return OrderedDict((k, getattr(obj, k, None)) for k in slots)
    d = getattr(obj, "__dict__", None)
    if isinstance(d, dict) and d and not isinstance(obj, (str, bytes)):
        return OrderedDict((k, v) for k, v in d.items() if not k.startswith("_"))
    return None


def flatten(obj, prefix="", out=None):
    if out is None:
        out = OrderedDict()

    fields = sub_fields(obj)
    if fields is not None:
        for name, value in fields.items():
            flatten(value, f"{prefix}{name}.", out)
        return out

    key = prefix[:-1] if prefix.endswith(".") else prefix

    if isinstance(obj, (bytes, bytearray)):
        out[key] = obj.hex()
    elif isinstance(obj, str) or not hasattr(obj, "__len__"):
        out[key] = obj
    else:  # list / tuple / numpy array
        items = list(obj)
        if len(items) > MAX_ARRAY:
            out[key] = " ".join(str(v) for v in items)
        else:
            for i, item in enumerate(items):
                flatten(item, f"{key}.{i}.", out)
    return out


def safe_name(topic):
    return re.sub(r"[^A-Za-z0-9]+", "_", topic).strip("_") or "topic"


def list_topics(path):
    with open(path, "rb") as f:
        summary = make_reader(f).get_summary()
        for ch in summary.channels.values():
            schema = summary.schemas.get(ch.schema_id)
            count = summary.statistics.channel_message_counts.get(ch.id, "?")
            print(f"{ch.topic:55s} {schema.name if schema else '?':40s} {count} msgs")


def export(path, out_dir, topics=None):
    out_dir.mkdir(parents=True, exist_ok=True)
    writers, files, counts = {}, {}, {}

    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=topics, log_time_order=True
        ):
            row = OrderedDict(
                log_time_ns=message.log_time,
                publish_time_ns=message.publish_time,
            )
            row.update(flatten(ros_msg))

            w = writers.get(channel.topic)
            if w is None:
                fp = (out_dir / f"{safe_name(channel.topic)}.csv").open(
                    "w", newline="", encoding="utf-8"
                )
                files[channel.topic] = fp
                w = writers[channel.topic] = csv.DictWriter(
                    fp, fieldnames=list(row), extrasaction="ignore", restval=""
                )
                w.writeheader()
                counts[channel.topic] = 0
            w.writerow(row)
            counts[channel.topic] += 1

    for fp in files.values():
        fp.close()

    if not counts:
        print("No messages matched. Check topic names with --list.", file=sys.stderr)
    for topic, n in sorted(counts.items()):
        print(f"{out_dir / (safe_name(topic) + '.csv')}  ({n} rows)")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mcap")
    p.add_argument("-o", "--out", default="csv_out", type=Path)
    p.add_argument("-t", "--topics", nargs="*", default=None)
    p.add_argument("--list", action="store_true", help="list topics and exit")
    a = p.parse_args()

    if a.list:
        list_topics(a.mcap)
    else:
        export(a.mcap, a.out, a.topics)