#!/usr/bin/env python3
"""
Batch or single reorder ROS 2 bag directories by each message's header.stamp timestamp,
preserving original directory names and *.db3 filenames.

Single bag mode:
    If --input-dir is a bag directory containing .db3 files, --output-dir is treated as a
    parent directory in which a subdirectory named after the input bag's basename will be
    created. The new bag (metadata.yaml + <basename>_0.db3) will live there.

Batch mode:
    If --input-dir contains multiple bag subdirectories, each subdir is treated as a bag.
    Matching subdirectories will be created under --output-dir, preserving original names.

Usage examples:
  Single:
    python3 reorder_bags_by_header.py \
      --input-dir /data/bags/session42 \
      --output-dir /data/sorted_bags

  Batch:
    python3 reorder_bags_by_header.py \
      --input-dir /data/bags \
      --output-dir /data/sorted_bags
"""
import os
import sys
import argparse
import importlib

import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)


def is_rosbag_directory(path: str) -> bool:
    """
    Determine if a directory is a ROS 2 bag by checking for any .db3 file inside.
    """
    try:
        return any(entry.endswith('.db3') for entry in os.listdir(path))
    except OSError:
        return False


def msg_class_from_type(type_name: str):
    """
    Convert ROS type string (e.g. 'sensor_msgs/msg/Image') to the Python message class.
    """
    try:
        parts = type_name.split('/')
        pkg = parts[0]
        msg_name = parts[-1]
        # Import the ROS message module, e.g. sensor_msgs.msg
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg_name)
    except Exception as e:
        raise RuntimeError(f"Could not import message class for type '{type_name}': {e}")


def reorder_bag(input_dir: str, output_dir: str, storage_id: str = 'sqlite3') -> None:
    """
    Read all messages from a bag folder, sort by header.stamp (or storage ts),
    and write to a new bag folder, preserving topic names and types.
    """
    print(f"\nProcessing bag: {input_dir}")
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=input_dir, storage_id=storage_id)
    conv_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_opts, conv_opts)

    # Map topics to type strings
    topics_and_types = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics_and_types}

    # Buffer all messages
    buffer = []  # list of (stamp_ns, topic, msg)
    while reader.has_next():
        topic, serialized, storage_ts = reader.read_next()
        MsgClass = msg_class_from_type(topic_type_map[topic])
        msg = deserialize_message(serialized, MsgClass)
        # Use header.stamp if available, else storage timestamp
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            h = msg.header.stamp
            stamp_ns = h.sec * 1_000_000_000 + h.nanosec
        else:
            stamp_ns = storage_ts
        buffer.append((stamp_ns, topic, msg))

    # Sort by timestamp across entire bag
    buffer.sort(key=lambda x: x[0])

    # Prepare writer
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_dir, storage_id=storage_id),
        conv_opts
    )

    # Register topics using TopicMetadata
    for topic, type_name in topic_type_map.items():
        meta = TopicMetadata(
            name=topic,
            type=type_name,
            serialization_format='cdr'
        )
        writer.create_topic(meta)

    # Write sorted messages
    for stamp_ns, topic, msg in buffer:
        ser = serialize_message(msg)
        writer.write(topic, ser, stamp_ns)

    print(f"Re-saved sorted bag into '{output_dir}'")


def main():
    parser = argparse.ArgumentParser(
        description='Reorder one or more ROS 2 bags by header timestamp, preserving original names.'
    )
    parser.add_argument(
        '--input-dir', '-i', required=True,
        help='Path to a bag directory or parent directory of multiple bags.'
    )
    parser.add_argument(
        '--output-dir', '-o', required=True,
        help='Directory where reordered bag(s) will be written.'
    )
    parser.add_argument(
        '--storage-id', default='sqlite3',
        help='Storage backend (e.g., sqlite3 or mcap).'
    )
    args = parser.parse_args()

    # Initialize ROS context
    rclpy.init()

    inp = args.input_dir
    out_base = args.output_dir

    # Single bag mode
    if is_rosbag_directory(inp) and not any(
        os.path.isdir(os.path.join(inp, d)) for d in os.listdir(inp)
    ):
        base = os.path.basename(os.path.normpath(inp))
        dst = os.path.join(out_base, base)
        # os.makedirs(dst, exist_ok=True)
        reorder_bag(inp, dst, args.storage_id)
    else:
        # Batch mode: scan subdirectories
        os.makedirs(out_base, exist_ok=True)
        for name in sorted(os.listdir(inp)):
            src = os.path.join(inp, name)
            if not os.path.isdir(src) or not is_rosbag_directory(src):
                continue
            dst = os.path.join(out_base, name)
            # os.makedirs(dst, exist_ok=True)
            reorder_bag(src, dst, args.storage_id)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
