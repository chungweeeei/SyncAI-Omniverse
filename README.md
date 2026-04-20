# SyncAI-Omniverse

Warehouse simulation with a Jetbot AMR on NVIDIA Isaac Sim 5.1.

## Prerequisites

- NVIDIA Isaac Sim 5.1
- GPU with RTX support
- Nucleus server access (for built-in Jetbot asset)

## Project Structure

```
config/sim_config.yaml        — Tunable parameters (scene size, robot speed, etc.)
scripts/create_warehouse_scene.py  — Generate warehouse USD (headless)
scripts/run_simulation.py     — Launch simulation with keyboard-controlled Jetbot
src/syncai_omniverse/          — Reusable library modules
```

## Usage

Use Isaac Sim's bundled Python to run the scripts:

```bash
# Generate the warehouse scene
python.sh scripts/create_warehouse_scene.py

# Run the simulation
python.sh scripts/run_simulation.py
```

`run_simulation.py` will auto-generate the scene if it doesn't exist yet.

## Controls

| Key | Action |
|-----|--------|
| W / Up Arrow | Forward |
| S / Down Arrow | Backward |
| A / Left Arrow | Turn left |
| D / Right Arrow | Turn right |

## Configuration

Edit `config/sim_config.yaml` to adjust:

- `warehouse` — Ground size, wall height, shelf layout, obstacle count
- `jetbot` — Spawn position, max speeds
- `simulation` — Physics and rendering timestep
