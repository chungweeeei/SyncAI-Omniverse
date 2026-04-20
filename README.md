# SyncAI-Omniverse

Warehouse simulation with a TurtleBot3 AMR on NVIDIA Isaac Sim 5.1.

## Project Structure

```
assets/models/dp1f/       -- Real floor plan STL
config/                   -- Simulation config (sim_config.yaml)
src/syncai_omniverse/usd/ -- USD scene builders (pure pxr, no Isaac Sim)
scripts/                  -- CLI entry points
scenes/                   -- Generated USD output (gitignored)
```

## Quick Start

Generate a USD scene from the floor plan STL (no Isaac Sim required):

```bash
pip install usd-core pyyaml
python scripts/preview_scene.py --no-view
```

## Configuration

Edit `config/sim_config.yaml` to switch scene modes and adjust parameters.
