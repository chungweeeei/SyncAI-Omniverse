#!/usr/bin/env python3
"""
Generate warehouse USD locally and open in usdview for preview.
Does NOT require Isaac Sim — only needs `pip install usd-core pyyaml`.

Usage:
    pip install usd-core pyyaml
    python scripts/preview_scene.py          # generate + open usdview
    python scripts/preview_scene.py --no-view  # generate only
"""

import os
import sys
import argparse
import subprocess

# Add src/ to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from syncai_omniverse.warehouse_builder import WarehouseBuilder


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCENES_DIR = os.path.join(ROOT_DIR, "scenes")
SCENE_PATH = os.path.join(SCENES_DIR, "warehouse.usda")

# Default config (same as startup_scene.py)
DEFAULT_CONFIG = {
    "ground_size": [8.0, 8.0],
    "wall_height": 2.0,
    "wall_thickness": 0.1,
    "num_shelf_rows": 1,
    "num_shelf_cols": 2,
    "shelf_spacing": 2.5,
    "shelf_height": 1.0,
    "shelf_width": 0.8,
    "shelf_depth": 0.3,
    "num_obstacle_boxes": 2,
    "box_size_range": [0.2, 0.3],
}


def main():
    parser = argparse.ArgumentParser(description="Preview USD scene locally")
    parser.add_argument("--no-view", action="store_true", help="Generate only, don't open usdview")
    args = parser.parse_args()

    os.makedirs(SCENES_DIR, exist_ok=True)

    # Try loading YAML config
    config = DEFAULT_CONFIG
    try:
        import yaml
        config_path = os.path.join(ROOT_DIR, "config", "sim_config.yaml")
        if os.path.exists(config_path):
            with open(config_path) as f:
                config = yaml.safe_load(f).get("warehouse", DEFAULT_CONFIG)
            print(f"Loaded config from {config_path}")
    except ImportError:
        print("pyyaml not installed — using default config")

    # Build warehouse
    print(f"Building warehouse scene -> {SCENE_PATH}")
    builder = WarehouseBuilder(config, SCENE_PATH)
    builder.build()
    print(f"  Done: {SCENE_PATH}")

    if not args.no_view:
        print(f"\nOpening usdview: {SCENE_PATH}")
        try:
            subprocess.run(["usdview", SCENE_PATH])
        except FileNotFoundError:
            print("usdview not found. Install with: pip install usd-core")
            print(f"You can also open the file manually: {SCENE_PATH}")


if __name__ == "__main__":
    main()
