# CLAUDE.md

## Project Overview

Warehouse simulation with a TurtleBot3 AMR on NVIDIA Isaac Sim 5.1. Currently in early build-up phase — only STL→USD scene conversion is implemented.

**Isaac Sim version**: 5.1.0 (Docker image: `nvcr.io/nvidia/isaac-sim:5.1.0`)

## Running

```bash
# Local preview (no Isaac Sim required):
pip install usd-core pyyaml
python scripts/preview_scene.py --no-view   # Generate USD only
python scripts/preview_scene.py             # Generate + open in usdview
```

## Architecture

### Directory layout

```
assets/models/             -- 3D model files (STL)
config/                    -- sim_config.yaml, cyclonedds.xml
src/syncai_omniverse/
  usd/                     -- Pure pxr modules (no Isaac Sim dependency)
    stl_to_usd.py          -- STL -> USD with physics, collision, materials
  sim/                     -- (planned) Isaac Sim runtime modules
  ros2/                    -- (planned) ROS2 bridge OmniGraph setup
scripts/                   -- CLI entry points
docker/                    -- (planned) Docker entrypoint
scenes/                    -- Generated USD output (gitignored)
```

### Dependency boundaries

- **`usd/`** depends only on `pxr` (`pip install usd-core`). Can develop and test on any machine.
- **`sim/`** (planned) will require Isaac Sim runtime (`isaacsim`, `omni`, `carb`).
- **`ros2/`** (planned) will require Isaac Sim + OmniGraph + ROS2 bridge extensions.

## Key modules

- **`stl_to_usd.py`** — Parses binary/ASCII STL files and builds a complete USD stage with physics, collision, ground plane, materials, and lighting. No Isaac Sim dependency (pure `pxr`).

## Important Patterns

- **USD-only modules** (`src/syncai_omniverse/usd/`) depend only on `pxr`, not on Isaac Sim runtime. They can run with `pip install usd-core`.
- **Import order in scripts** (future): `SimulationApp` must be instantiated *before* importing any `omni`, `pxr`, or `isaacsim` modules.
- **Kit event subscriptions must be stored globally** to prevent garbage collection from silently dropping them.
