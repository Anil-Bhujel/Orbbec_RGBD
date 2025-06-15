#!/usr/bin/env python3
"""
Scan a base directory for ROS 2 bag folders and generate missing metadata.yaml files
using `ros2 bag reindex` if needed.

Usage:
    python3 reindex_bags.py /path/to/base_directory
"""
import os
import subprocess
import argparse


def is_rosbag_directory(path: str) -> bool:
    """
    Determine if a directory is a ROS 2 bag by checking for .db3 files inside.
    """
    try:
        for entry in os.listdir(path):
            if entry.endswith('.db3'):
                return True
    except FileNotFoundError:
        return False
    return False


def process_bags(base_dir: str) -> None:
    """
    Iterate through immediate subdirectories of base_dir, check for metadata.yaml,
    and run `ros2 bag reindex` on those needing it.
    """
    if not os.path.isdir(base_dir):
        print(f"Error: '{base_dir}' is not a directory.")
        return

    for name in sorted(os.listdir(base_dir)):
        bag_dir = os.path.join(base_dir, name)
        if not os.path.isdir(bag_dir):
            continue

        # Only consider directories that look like ROS 2 bags
        if not is_rosbag_directory(bag_dir):
            print(f"Skipping '{bag_dir}': no .db3 files found.")
            continue

        meta_file = os.path.join(bag_dir, 'metadata.yaml')
        if os.path.exists(meta_file):
            print(f"Metadata already exists for '{bag_dir}', skipping.")
        else:
            print(f"Generating metadata for '{bag_dir}'...")
            try:
                subprocess.run(
                    ['ros2', 'bag', 'reindex', bag_dir],
                    check=True
                )
                print(f"Successfully reindexed '{bag_dir}'.")
            except subprocess.CalledProcessError as e:
                print(f"Error reindexing '{bag_dir}': {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Re-generate missing metadata.yaml for ROS 2 bags in a directory.'
    )
    parser.add_argument(
        'base_dir',
        help='Path to the directory containing ROS 2 bag subdirectories.'
    )
    args = parser.parse_args()

    process_bags(args.base_dir)


if __name__ == '__main__':
    main()
