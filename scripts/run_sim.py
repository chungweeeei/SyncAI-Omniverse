"""Launch Isaac Sim, open a scene USD, and attach the ROS2 TF publisher.

Run inside the isaac-sim container:
    /isaac-sim/python.sh scripts/run_sim.py
    /isaac-sim/python.sh scripts/run_sim.py --scene /workspace/scenes/warehouse.usda
    /isaac-sim/python.sh scripts/run_sim.py --headless
    /isaac-sim/python.sh scripts/run_sim.py --no-ros2       # skip TF setup
"""
import argparse
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--scene",
    default="/workspace/scenes/warehouse.usda",
    help="Path to the USD stage to open.",
)
parser.add_argument(
    "--robot",
    default="/World/SlotCar",
    help="Prim path of the articulation root whose TF should be published.",
)
parser.add_argument("--headless", action="store_true", help="Run without a window.")
parser.add_argument("--no-ros2", action="store_true", help="Skip ROS2 TF publisher setup.")
parser.add_argument(
    "--tf-targets",
    default="lidar_link",
    help="Comma-separated link names under --robot to publish (relative to --robot). "
         "Use 'auto' to publish every rigid-body child link.",
)
parser.add_argument(
    "--no-odom",
    action="store_true",
    help="Skip /odom + /joint_states + odom->base_link TF publisher.",
)
parser.add_argument(
    "--no-cmdvel",
    action="store_true",
    help="Skip /cmd_vel -> DifferentialController -> ArticulationController graph.",
)
parser.add_argument(
    "--debug-cmdvel",
    action="store_true",
    help="Verbose cmd_vel graph build log + periodic runtime snapshot of live values.",
)
parser.add_argument(
    "--no-lidar",
    action="store_true",
    help="Skip RTX lidar + /scan publisher.",
)
parser.add_argument(
    "--lidar-config",
    default="SICK_picoScan150",
    help="Lidar config stem from SUPPORTED_LIDAR_CONFIGS (e.g. SICK_picoScan150, "
         "SICK_TIM781, RPLIDAR_S2E). Vendor folder is NOT included.",
)
args = parser.parse_args()

scene_path = Path(args.scene).resolve()
if not scene_path.exists():
    raise SystemExit(f"Scene not found: {scene_path}")

# SimulationApp MUST be instantiated before importing any omni/pxr/isaacsim modules.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

# Make our src/ importable now that the app is alive.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import omni.usd
import omni.timeline
from isaacsim.core.utils.stage import open_stage, is_stage_loading

print(f"[run_sim] Opening stage: {scene_path}")
open_stage(str(scene_path))
# Wait for the stage (and its references) to finish loading.
while is_stage_loading():
    simulation_app.update()

stage = omni.usd.get_context().get_stage()
if stage is None:
    simulation_app.close()
    raise SystemExit(f"Failed to open stage: {scene_path}")

if not args.no_ros2:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    # Allow the extension to finish loading before authoring its nodes.
    for _ in range(20):
        simulation_app.update()

    from syncai_omniverse.ros2.tf_publisher import attach_tf_publisher

    if args.tf_targets.strip().lower() == "auto":
        target_links = None
    else:
        target_links = [s.strip() for s in args.tf_targets.split(",") if s.strip()]
    attach_tf_publisher(stage, robot_path=args.robot, target_links=target_links)
    targets_desc = ",".join(target_links) if target_links else "<auto: all rigid-body links>"
    print(f"[run_sim] TF publisher attached: {args.robot}/base_link -> /tf  targets={targets_desc}")

    if not args.no_odom:
        from syncai_omniverse.ros2.odom_publisher import attach_odom_publisher

        attach_odom_publisher(stage, robot_path=args.robot)
        print(f"[run_sim] Odom publisher attached: /odom, /joint_states, odom->base_link on /tf")

    if not args.no_cmdvel:
        from syncai_omniverse.ros2.cmd_vel_subscriber import (
            attach_cmd_vel_subscriber,
            print_cmd_vel_snapshot,
        )

        attach_cmd_vel_subscriber(stage, robot_path=args.robot, debug=args.debug_cmdvel)
        print(f"[run_sim] cmd_vel subscriber attached: /cmd_vel -> drivewhl_l/r_joint")

    if not args.no_lidar:
        from isaacsim.core.utils.extensions import enable_extension as _enable_ext

        _enable_ext("isaacsim.sensors.rtx")
        for _ in range(20):
            simulation_app.update()

        from syncai_omniverse.ros2.lidar_publisher import attach_lidar_publisher

        attach_lidar_publisher(stage, robot_path=args.robot, config=args.lidar_config)
        print(f"[run_sim] lidar publisher attached: /scan  config={args.lidar_config}")

omni.timeline.get_timeline_interface().play()

try:
    import time
    next_debug = time.time() + 2.0
    while simulation_app.is_running():
        simulation_app.update()
        if args.debug_cmdvel and not args.no_cmdvel and time.time() >= next_debug:
            print_cmd_vel_snapshot()
            next_debug = time.time() + 2.0
finally:
    simulation_app.close()
