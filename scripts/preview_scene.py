"""Generate a warehouse USD scene locally and optionally open in usdview.

Only requires: pip install usd-core pyyaml
No Isaac Sim dependency.
"""

import argparse
import os
import sys

# Ensure src/ is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import yaml
from syncai_omniverse.usd.stl_to_usd import stl_to_usd
from syncai_omniverse.usd.scene import build_combined_scene


def _resolve_stl_path(raw_path: str) -> str:
    """Resolve an STL path; rewrite container-style /workspace/... to project-relative."""
    if raw_path.startswith("/workspace/"):
        return os.path.join(
            os.path.dirname(__file__), "..",
            raw_path.replace("/workspace/", ""),
        )
    return raw_path


def main():
    parser = argparse.ArgumentParser(description="Preview warehouse scene")
    parser.add_argument("--no-view", action="store_true", help="Generate only, don't open usdview")
    parser.add_argument("--config", default="config/sim_config.yaml", help="Path to config YAML")
    args = parser.parse_args()

    # Load config
    config_path = os.path.join(os.path.dirname(__file__), "..", args.config)
    with open(config_path) as f:
        config = yaml.safe_load(f)

    output_path = os.path.join(
        os.path.dirname(__file__), "..",
        config.get("output", {}).get("scene_path", "scenes/preview.usda"),
    )

    scene_mode = config.get("scene_mode", "stl")

    if scene_mode == "stl":
        stl_cfg = config["stl"]
        stl_path = _resolve_stl_path(stl_cfg["file_path"])
        print(f"Building STL scene from {stl_path}")
        stl_to_usd(stl_path, output_path, stl_cfg)
    elif scene_mode == "warehouse_with_robot":
        stl_path = _resolve_stl_path(config["stl"]["file_path"])
        print(f"Building warehouse + robot scene from {stl_path}")
        build_combined_scene(output_path, stl_path, config)
    else:
        print(f"Scene mode '{scene_mode}' not yet implemented")
        sys.exit(1)

    print(f"Scene saved: {output_path}")

    if not args.no_view:
        import subprocess
        subprocess.run(["usdview", output_path])


if __name__ == "__main__":
    main()
