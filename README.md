# SyncAI-Omniverse

Warehouse simulation with a differential-drive AMR (SlotCar) on **NVIDIA Isaac Sim 5.1**. The scene is built from a real-floor-plan STL; the robot exposes a standard ROS2 interface (`/cmd_vel`, `/odom`, `/joint_states`, `/scan`, `/tf`, `/clock`) for nav2 / rviz2 integration.

## Status

- STL → USD warehouse + `SlotCar` robot assembly (floating-base articulation, diff-drive, single caster, RTX lidar).
- ROS2 bridge graphs: clock, TF, odom + joint states, `cmd_vel` subscriber, RTX lidar publisher.
- Runs in the official `nvcr.io/nvidia/isaac-sim:5.1.0` container, networked via an external bridge (`syncai-network`) so it can sit alongside other SyncAI stacks.

## Project Structure

```
assets/models/
  dp1f/                          -- Warehouse floor-plan STL
  slotcar/                       -- (reserved) slot-car model assets
config/
  sim_config.yaml                -- scene_mode, STL, robot spawn, output path
  cyclonedds.xml                 -- DDS config (auto-interface, multicast on)
docker-compose.yml               -- Isaac Sim container on external syncai-network
src/syncai_omniverse/
  usd/                           -- Pure pxr (no Isaac Sim runtime)
    stl_to_usd.py                -- STL parse + warehouse stage build
    slot_car.py                  -- SlotCar articulation authoring
    scene.py                     -- Combined / robot-only stage composers
  ros2/                          -- OmniGraph builders (Isaac Sim runtime required)
    clock_publisher.py           -- /clock
    tf_publisher.py              -- /tf for base_link -> child links
    odom_publisher.py            -- /odom, /joint_states, odom->base_link TF
    cmd_vel_subscriber.py        -- /cmd_vel -> diff controller -> articulation
    lidar_publisher.py           -- RTX lidar -> /scan (LaserScan or PointCloud2)
    _ns.py                       -- namespace helpers
  sim/                           -- (planned) runtime-only utilities
  helpers/
    stl_helper.py                -- STL I/O helpers
scripts/
  preview_scene.py               -- Build USD locally, optionally open in usdview
  run_sim.py                     -- Launch Isaac Sim + attach ROS2 graphs
  diagnose_cmdvel.py             -- Diagnostic
  probe_cmdvel.py                -- Diagnostic
scenes/                          -- Generated USD output (gitignored)
tests/test_usd/                  -- USD-layer tests
```

## Quick Start

### 1. Local preview (no Isaac Sim)

Generate and inspect the USD stage with only `pxr`:

```bash
pip install usd-core pyyaml
python scripts/preview_scene.py --no-view     # generate only
python scripts/preview_scene.py               # generate + open usdview
```

Scene mode is selected in `config/sim_config.yaml`:
- `warehouse_with_robot` (default) — STL warehouse + SlotCar
- `robot_only` — flat ground + SlotCar (diagnostic, no warehouse mesh)
- `stl` — warehouse only

### 2. Full simulation (Isaac Sim container)

One-time setup — create the shared bridge network (shared across SyncAI compose stacks):

```bash
docker network create syncai-network          # skip if it already exists
```

Bring the container up:

```bash
docker compose up -d
docker exec -it syncai-simulation bash
```

Inside the container:

```bash
# Generate the scene USD (first time only, or after config/asset changes)
/isaac-sim/python.sh scripts/preview_scene.py --no-view

# Launch Isaac Sim with all ROS2 bridges
/isaac-sim/python.sh scripts/run_sim.py
```

Useful `run_sim.py` flags:

| Flag | Purpose |
|---|---|
| `--scene <path>` | Override stage (default `scenes/dp1f_slotcar.usda`) |
| `--headless` | No window (use with livestream or CI) |
| `--no-ros2` / `--no-clock` / `--no-odom` / `--no-cmdvel` / `--no-lidar` | Skip individual graphs |
| `--lidar-config <stem>` | Pick a config from `SUPPORTED_LIDAR_CONFIGS` (e.g. `SICK_picoScan150`, `SICK_multiScan165`, `Velodyne_VLS128`) |
| `--lidar-publish-type {auto,laser_scan,point_cloud}` | `auto` picks LaserScan only for true-2D configs |
| `--cmd-vel-topic <topic>` | Default `/cmd_vel_smoothed` (nav2 velocity_smoother output). Use `/cmd_vel` for raw teleop |
| `--ros-namespace <ns>` | Prefix data topics; `/tf`, `/tf_static`, `/clock` stay global |
| `--debug-cmdvel` / `--debug-pose` | Verbose diagnostics |

## ROS2 Topics

Published by the sim:
- `/clock` — sim time (enable `use_sim_time:=true` in rviz2 / nav2)
- `/tf`, `/tf_static` — `odom -> base_link`, `base_link -> lidar_link`, wheels
- `/odom` — `nav_msgs/Odometry`
- `/joint_states` — wheel encoder states
- `/scan` — `sensor_msgs/LaserScan` (2D configs) or `sensor_msgs/PointCloud2`

Subscribed by the sim:
- `/cmd_vel_smoothed` by default (override with `--cmd-vel-topic /cmd_vel`)

## Networking

`docker-compose.yml` attaches the container to the external bridge `syncai-network`. Other SyncAI compose stacks joining the same network reach Isaac Sim by service name and share DDS discovery (user-defined bridges allow multicast by default).

Host-side ROS2 tools (`ros2 topic list` on the host) will **not** see container topics — `docker exec` into `syncai-simulation`, or attach another ROS2 container to `syncai-network`.

If Kit UI windows don't appear after `docker compose up`, run once on the host:

```bash
xhost +local:docker
```

## Configuration

`config/sim_config.yaml`:

```yaml
scene_mode: warehouse_with_robot       # or stl / robot_only
stl:
  file_path: /workspace/assets/models/dp1f/dp1f.stl
  scale: 0.1
  center_xy: true
  ground_plane_size: [20.0, 30.0]
robot:
  enabled: true
  robot_name: SlotCar
  spawn_position: [0.0, 0.0, 0.15]
output:
  scene_path: scenes/dp1f_slotcar.usda
```

`config/cyclonedds.xml` — auto-detect interface, multicast on. Unchanged between `network_mode: host` and the user-defined bridge setup.

## Architecture Notes

- **USD-only vs runtime modules.** Code under `src/syncai_omniverse/usd/` depends only on `pxr` (`pip install usd-core`) and can be edited/tested without Isaac Sim. Code under `src/syncai_omniverse/ros2/` imports `omni.graph.core` and `isaacsim.*` and can only run inside the container.
- **Graph-stage ordering.** `cmd_vel_subscriber` runs on `OnPhysicsStep` in an **on-demand** graph (required, or the node silently never fires). All other graphs tick on `OnPlaybackTick`.
- **Articulation root on `base_link`.** Must live on a `RigidBody`; applying it to the parent `Xform` produces a fixed-base articulation (wheels spin, chassis stays pinned).
- **Single caster + frictionless material.** Diff-drive needs exactly one caster; 2 fixed-joint casters overconstrain PhysX static friction and stall the chassis. Caster material uses `friction_combine_mode=min` so μ=0 is actually honored against the warehouse floor.
- **RTX lidar publishes full scans.** `fullScan=true` is set on `ROS2RtxLidarHelper` so each publish contains one complete rotation instead of a partial accumulation at render tick rate.

## Development

```bash
# Edit USD authoring locally, verify in usdview, iterate without bringing up the container
python scripts/preview_scene.py

# Full ROS2 stack lives inside the container
docker exec -it syncai-simulation bash
```

USD unit tests live in `tests/test_usd/`.
