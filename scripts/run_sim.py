"""Launch Isaac Sim, open a scene USD, and attach ROS2 bridges per robot.

Run inside the isaac-sim container:
    /isaac-sim/python.sh scripts/run_sim.py
    /isaac-sim/python.sh scripts/run_sim.py --scene /workspace/scenes/warehouse.usda
    /isaac-sim/python.sh scripts/run_sim.py --headless
    /isaac-sim/python.sh scripts/run_sim.py --no-ros2

Multi-robot: this script reads `config/sim_config.yaml` at startup and loops
the per-robot attach (tf, odom, cmd_vel, lidar) over every entry under
`robots:`. Each robot gets a namespaced topic tree (e.g. /robot01/cmd_vel)
and namespaced OmniGraph paths (e.g. /robot01/CmdVelActionGraph) so two or
more robots can coexist in the same stage without prim collisions. Clock
and doors stay as stage-wide singletons.
"""
import argparse
import sys
from pathlib import Path

import yaml

# Per-model defaults. wheel_radius / wheel_distance are auto-derived from
# each robot's `scale` (see _resolve_robot_runtime below) so they don't need
# to be re-stated here when scale changes.
_MODEL_DEFAULTS = {
    "mir250": {
        "scene": "/workspace/scenes/dp1f_mir.usda",
        # MiR250 drive gains / accel limits tuned for the ~100 kg chassis:
        # enough torque to track cmd_vel without startup stall, but capped
        # so pitch transient on accel steps stays inside the caster-engage
        # envelope. Override per robot via `wheel_drive_*` keys in YAML if
        # a non-default scale needs different gains.
        "wheel_drive_damping": 800.0,
        "wheel_drive_max_force": 40.0,
        "max_linear_accel": 1.0,
        "max_linear_decel": 1.5,
        "max_angular_accel": 1.5,
        "lidar_layout": "dual_diagonal",
        "tf_targets": "lidar_link_front:scan_front,lidar_link_rear:scan_rear",
        # Real MiR250 unscaled geometry; the actual values used at runtime
        # are these * the per-robot `scale`.
        "wheel_radius_unscaled": 0.10,
        "wheel_distance_unscaled": 0.445,
    },
}


def _load_sim_config(scene_path: Path) -> dict:
    """Locate and load sim_config.yaml.

    Looks next to the scene (../config relative to scenes/) and falls back
    to the project's config dir derived from this script's location.
    """
    candidates = [
        scene_path.resolve().parent.parent / "config" / "sim_config.yaml",
        Path(__file__).resolve().parent.parent / "config" / "sim_config.yaml",
    ]
    for p in candidates:
        if p.exists():
            with open(p) as f:
                return yaml.safe_load(f) or {}
    raise SystemExit(f"sim_config.yaml not found; searched: {candidates}")


def _resolve_robot_runtime(robot_cfg: dict, model: str) -> dict:
    """Merge YAML per-robot keys with model defaults; derive wheel geometry.

    `wheel_radius` and `wheel_distance` are auto-computed from the robot's
    `scale` so the YAML stays a single source of truth (mismatches between
    scale and these values cause cmd_vel to track at the wrong velocity --
    a bug we hit while iterating on per-robot scale).
    """
    if model not in _MODEL_DEFAULTS:
        raise SystemExit(f"Unknown model={model!r}; expected {sorted(_MODEL_DEFAULTS)}")
    md = _MODEL_DEFAULTS[model]
    s = float(robot_cfg.get("scale", 1.0))
    namespace = robot_cfg.get("namespace", "") or ""
    robot_name = robot_cfg["robot_name"]
    return {
        "robot_name": robot_name,
        "namespace": namespace,
        "robot_path": f"/World/{robot_name}",
        "scale": s,
        "wheel_radius": md["wheel_radius_unscaled"] * s,
        "wheel_distance": md["wheel_distance_unscaled"] * s,
        "wheel_drive_damping": float(robot_cfg.get(
            "wheel_drive_damping", md["wheel_drive_damping"])),
        "wheel_drive_max_force": float(robot_cfg.get(
            "wheel_drive_max_force", md["wheel_drive_max_force"])),
        "max_linear_accel": md["max_linear_accel"],
        "max_linear_decel": md["max_linear_decel"],
        "max_angular_accel": md["max_angular_accel"],
        "lidar_layout": robot_cfg.get("lidar_layout", md["lidar_layout"]),
        "tf_targets": robot_cfg.get("tf_targets", md["tf_targets"]),
        "model": model,
    }


def _graph_path(namespace: str, suffix: str) -> str:
    """Build a namespaced OmniGraph path. Suffix is e.g. 'CmdVelActionGraph'.

    Hardcoded defaults inside attach_* functions would collide if called
    twice; namespacing the graph path is what lets two robots share a stage.
    """
    return f"/{namespace}/{suffix}" if namespace else f"/{suffix}"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--scene",
    default=None,
    help="Path to the USD stage to open. Defaults to the mir250 scene.",
)
parser.add_argument("--headless", action="store_true", help="Run without a window.")
parser.add_argument("--no-ros2", action="store_true", help="Skip ROS2 TF publisher setup.")
parser.add_argument(
    "--tf-targets",
    default=None,
    help="Comma-separated link names under --robot to publish (relative to --robot). "
         "Each entry may use `prim:frame_id` to decouple the USD prim from the "
         "published TF frame id. Defaults to mir250's "
         "`lidar_link_front:scan_front,lidar_link_rear:scan_rear`. Use 'auto' "
         "for every rigid-body child link (prim name == frame id).",
)
parser.add_argument(
    "--no-clock",
    action="store_true",
    help="Skip the /clock publisher. Without /clock, rviz2/nav2 must run with "
         "use_sim_time=false (wall clock) which causes TF_OLD_DATA warnings "
         "because TF msgs are stamped with sim time starting at 0.",
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
    "--no-lidar",
    action="store_true",
    help="Skip RTX lidar + /scan publisher.",
)
parser.add_argument(
    "--no-doors",
    action="store_true",
    help="Skip attaching ROS2 controllers for /World/Doors/* articulations.",
)
parser.add_argument(
    "--no-conveyors",
    action="store_true",
    help="Skip attaching ROS2 controllers + cargo box for /World/Conveyors/*.",
)
parser.add_argument(
    "--lidar-debug-draw",
    action="store_true",
    help="Draw each RTX lidar's returns in the viewport as red points "
         "(RtxLidarDebugDrawPointCloud writer, non-buffer). Useful for "
         "visually confirming the scan coverage / mount pose.",
)
parser.add_argument(
    "--lidar-config",
    default="SICK_picoScan150",
    help="Lidar config stem from SUPPORTED_LIDAR_CONFIGS. True 2D (publish as "
         "laser_scan): SICK_picoScan150 (2761 pts), SICK_tim781 (811 pts). 3D "
         "(publish as point_cloud): SICK_multiScan165 (11520 pts, elev -7..+34), "
         "SICK_multiScan136 (10800 pts), Velodyne_VLS128 (128ch rotary). Vendor "
         "folder is NOT included.",
)
parser.add_argument(
    "--lidar-publish-type",
    default="auto",
    choices=["auto", "laser_scan", "point_cloud"],
    help="ROS2 message type. 'auto' picks laser_scan for true 2D configs "
         "(elevation=[0,0]) and point_cloud otherwise.",
)
parser.add_argument(
    "--lidar-layout",
    default=None,
    choices=["single_center", "dual_diagonal"],
    help="Lidar mounting pattern. single_center: one RTX lidar on one mount "
         "point, publishing to a single /scan. "
         "dual_diagonal: two RTX lidars at front-left + rear-right, publishing "
         "/scan_front + /scan_rear on separate graphs (matches real MiR250 "
         "safety-lidar layout). Defaults to dual_diagonal.",
)
parser.add_argument(
    "--lidar-parent",
    default=None,
    help="For single_center layouts, link name under --robot to parent the "
         "RTX lidar to. Default: `lidar_link`. Ignored for dual_diagonal "
         "(both mount points are hard-coded to `lidar_link_front` + "
         "`lidar_link_rear`).",
)
parser.add_argument(
    "--lidar-frame",
    default="scan",
    help="For single_center layouts, ROS frame_id used in the /scan message "
         "header. Ignored for dual_diagonal (frames are `scan_front` + `scan_rear`).",
)
args = parser.parse_args()

# Resolve scene path + load YAML to discover the per-robot list.
_DEFAULT_MODEL = "mir250"
_model_cfg = _MODEL_DEFAULTS[_DEFAULT_MODEL]
if args.scene is None:
    args.scene = _model_cfg["scene"]
if args.lidar_parent is None:
    # Only used for single_center; dual_diagonal hard-codes both mount points.
    args.lidar_parent = "lidar_link"

scene_path = Path(args.scene).resolve()
if not scene_path.exists():
    raise SystemExit(f"Scene not found: {scene_path}")

_sim_cfg = _load_sim_config(scene_path)
_robot_entries = _sim_cfg.get("robots") or (
    [_sim_cfg["robot"]] if _sim_cfg.get("robot") else []
)
_robot_entries = [r for r in _robot_entries if r and r.get("enabled", True)]
if not _robot_entries:
    raise SystemExit(
        "No enabled robots in sim_config.yaml (looked for `robots:` list "
        "or legacy `robot:` dict)."
    )
ROBOTS = [
    _resolve_robot_runtime(r, r.get("model", _DEFAULT_MODEL))
    for r in _robot_entries
]
# Per-instance lidar layout override via CLI applies to ALL robots if set.
if args.lidar_layout is not None:
    for r in ROBOTS:
        r["lidar_layout"] = args.lidar_layout
# tf_targets override likewise.
if args.tf_targets is not None:
    for r in ROBOTS:
        r["tf_targets"] = args.tf_targets

# SimulationApp MUST be instantiated before importing any omni/pxr/isaacsim modules.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

# Make our src/ importable now that the app is alive.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import omni.usd
import omni.timeline
from isaacsim.core.utils.stage import open_stage, is_stage_loading

print(f"[run_sim] Opening stage: {scene_path}")
print(f"[run_sim] robots="
      + ", ".join(f"{r['robot_name']}(ns={r['namespace']!r}, scale={r['scale']:.2f}, "
                  f"lidar={r['lidar_layout']})" for r in ROBOTS))
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

    if not args.no_clock:
        from syncai_omniverse.ros2.clock_publisher import attach_clock_publisher

        # Single shared /clock for the whole stage (regardless of robot count).
        attach_clock_publisher(stage)
        print(f"[run_sim] Clock publisher attached: /clock (use rviz2/nav2 with use_sim_time:=true)")

    # Imports lifted out of the per-robot loop so each is loaded once.
    from syncai_omniverse.ros2.tf_publisher import attach_tf_publisher
    if not args.no_odom:
        from syncai_omniverse.ros2.odom_publisher import attach_odom_publisher
    if not args.no_cmdvel:
        from syncai_omniverse.ros2.cmd_vel_subscriber import attach_cmd_vel_subscriber
    if not args.no_lidar:
        from isaacsim.core.utils.extensions import enable_extension as _enable_ext

        _enable_ext("isaacsim.sensors.rtx")
        for _ in range(20):
            simulation_app.update()

        from syncai_omniverse.ros2.lidar_publisher import (
            attach_lidar_debug_draw,
            attach_lidar_publisher,
        )

    all_lidar_prims: list[str] = []

    for r in ROBOTS:
        ns = r["namespace"]
        rp = r["robot_path"]
        gprefix = f"/{ns}" if ns else ""
        ns_log = f"/{ns}" if ns else ""

        # -- TF: per-robot graph + namespaced topics --
        tf_targets_str = r["tf_targets"]
        if tf_targets_str.strip().lower() == "auto":
            target_links = None
        else:
            target_links = [s.strip() for s in tf_targets_str.split(",") if s.strip()]
        attach_tf_publisher(
            stage,
            robot_path=rp,
            target_links=target_links,
            namespace=ns,
            graph_path=_graph_path(ns, "TFActionGraph"),
        )
        targets_desc = ",".join(target_links) if target_links else "<auto: all rigid-body links>"
        print(f"[run_sim] [{r['robot_name']}] TF: {ns_log}/tf, targets={targets_desc}")

        # -- Odom + joint_states --
        if not args.no_odom:
            attach_odom_publisher(
                stage, robot_path=rp, namespace=ns,
                graph_path=_graph_path(ns, "OdomActionGraph"),
            )
            print(f"[run_sim] [{r['robot_name']}] Odom: {ns_log}/odom, "
                  f"{ns_log}/joint_states")

        # -- cmd_vel subscriber --
        if not args.no_cmdvel:
            _cmd_vel_topic = "/cmd_vel_smoothed"
            attach_cmd_vel_subscriber(
                stage,
                robot_path=rp,
                namespace=ns,
                topic=_cmd_vel_topic,
                wheel_radius=r["wheel_radius"],
                wheel_distance=r["wheel_distance"],
                wheel_drive_damping=r["wheel_drive_damping"],
                wheel_drive_max_force=r["wheel_drive_max_force"],
                max_linear_accel=r["max_linear_accel"],
                max_linear_decel=r["max_linear_decel"],
                max_angular_accel=r["max_angular_accel"],
                graph_path=_graph_path(ns, "CmdVelActionGraph"),
            )
            print(f"[run_sim] [{r['robot_name']}] cmd_vel: {ns_log}{_cmd_vel_topic} "
                  f"(r={r['wheel_radius']:.3f}, d={r['wheel_distance']:.3f})")

        # -- Lidar (per-robot, layout-dependent) --
        if not args.no_lidar:
            if r["lidar_layout"] == "dual_diagonal":
                # Two RTX lidars on diagonal corners -> two namespaced /scan_*
                # topics. Anti-ghost geometry: front 0°/250° + rear 180°/250°
                # so each lidar's 110° blind wedge covers the bearing to the
                # opposite one. Union coverage stays 360° (70° side overlap).
                front = attach_lidar_publisher(
                    stage,
                    robot_path=rp,
                    lidar_link="lidar_link_front",
                    lidar_name="LidarFront",
                    topic="/scan_front",
                    frame_id="scan_front",
                    namespace=ns,
                    config=args.lidar_config,
                    publish_type=args.lidar_publish_type,
                    graph_path=_graph_path(ns, "LidarActionGraphFront"),
                    rotation_z_deg=0.0,
                    horizontal_fov_deg=250.0,
                )
                rear = attach_lidar_publisher(
                    stage,
                    robot_path=rp,
                    lidar_link="lidar_link_rear",
                    lidar_name="LidarRear",
                    topic="/scan_rear",
                    frame_id="scan_rear",
                    namespace=ns,
                    config=args.lidar_config,
                    publish_type=args.lidar_publish_type,
                    graph_path=_graph_path(ns, "LidarActionGraphRear"),
                    rotation_z_deg=180.0,
                    horizontal_fov_deg=250.0,
                )
                all_lidar_prims += [front, rear]
                print(f"[run_sim] [{r['robot_name']}] dual lidar: "
                      f"{ns_log}/scan_front + {ns_log}/scan_rear  "
                      f"config={args.lidar_config}")
            else:
                p = attach_lidar_publisher(
                    stage,
                    robot_path=rp,
                    lidar_link=args.lidar_parent,
                    frame_id=args.lidar_frame,
                    topic="/scan",
                    namespace=ns,
                    config=args.lidar_config,
                    publish_type=args.lidar_publish_type,
                    graph_path=_graph_path(ns, "LidarActionGraph"),
                )
                all_lidar_prims.append(p)
                print(f"[run_sim] [{r['robot_name']}] single lidar: "
                      f"{ns_log}/scan  config={args.lidar_config}  "
                      f"parent={args.lidar_parent}")

    if not args.no_lidar and args.lidar_debug_draw:
        for prim_path in all_lidar_prims:
            attach_lidar_debug_draw(prim_path)

    if not args.no_doors:
        from syncai_omniverse.ros2.door_controller import attach_door_controller

        # Walk /World/Doors and attach one controller graph per door. Topic
        # and open-target are read from each door's USD customData (authored
        # by `auto_door.build_auto_door`), so run_sim.py needs no per-door
        # CLI plumbing and the scene stays self-describing.
        doors_root = stage.GetPrimAtPath("/World/Doors")
        if doors_root and doors_root.IsValid():
            for door in doors_root.GetChildren():
                name = door.GetName()
                custom = door.GetCustomData() or {}
                # Skip prims without the ros2_cmd_topic marker -- scene
                # authors set it from build_auto_door; anything else under
                # /World/Doors is a bystander.
                cmd_topic = custom.get("ros2_cmd_topic")
                state_topic = custom.get("ros2_state_topic")
                if cmd_topic is None or state_topic is None:
                    continue
                # Doors are shared warehouse infrastructure, not robot-specific.
                # Keep their topics global (no ROS namespace prefix) so every
                # AMR in the scene targets the same /door/<id>/cmd_topic.
                # Per-robot data topics (cmd_vel, odom, scan) still get
                # namespaced above.
                attach_door_controller(
                    stage,
                    door_path=str(door.GetPath()),
                    cmd_topic=cmd_topic,
                    state_topic=state_topic,
                    namespace="",
                    graph_path=f"/DoorGraph_{name}",
                )

    if not args.no_conveyors:
        # IsaacConveyor lives in isaacsim.asset.gen.conveyor; enable it BEFORE
        # creating the OG nodes that reference its node type. Same 20-tick
        # update wait as the ROS2 bridge / RTX sensor enables above.
        from isaacsim.core.utils.extensions import enable_extension as _enable_ext
        _enable_ext("isaacsim.asset.gen.conveyor")
        for _ in range(20):
            simulation_app.update()

        from syncai_omniverse.ros2.conveyor_controller import (
            attach_conveyor_controller,
        )

        # Walk /World/Conveyors and attach one IsaacConveyor + ScriptNode +
        # ROS2Subscriber graph per conveyor. CustomData (authored by
        # syncai_omniverse.usd.conveyor.build_conveyor) carries the speed/
        # status topics, belt surface prim, optional cargo box config, and
        # optional limit-switch threshold -- the scene stays self-describing.
        # Drop-zone registry (authored by build_drop_zones into
        # /World/DropZones/<id>). Pure metadata; we collect (id, x, y, r)
        # tuples and pass the whole list into every conveyor's controller so
        # the ScriptNode can validate /cargo/drop_cmd payloads. The drop
        # topic itself is a single global one (parallels /conveyor/<id>/...
        # and /door/<id>/... -- shared infrastructure, no robot ns).
        # Each tuple: (id, center_x, center_y, radius, drop_x, drop_y, drop_z).
        # drop_pose was added so the box snaps to a deterministic world point
        # at the moment of release instead of falling from wherever it was
        # being carried. Defaults to (cx, cy, 0.5) when YAML omits drop_pose.
        drop_zones_list: list[tuple[str, float, float, float, float, float, float]] = []
        dz_root = stage.GetPrimAtPath("/World/DropZones")
        if dz_root and dz_root.IsValid():
            for dz in dz_root.GetChildren():
                cd_z = dz.GetCustomData() or {}
                zid = cd_z.get("drop_zone_id")
                pxy = cd_z.get("position_xy")
                radius = float(cd_z.get("radius", 1.0))
                if zid and pxy is not None:
                    cx, cy = float(pxy[0]), float(pxy[1])
                    dpose = cd_z.get("drop_pose")
                    if dpose is not None:
                        dx, dy, dz_ = (
                            float(dpose[0]), float(dpose[1]), float(dpose[2])
                        )
                    else:
                        dx, dy, dz_ = cx, cy, 0.5
                    drop_zones_list.append(
                        (str(zid), cx, cy, radius, dx, dy, dz_)
                    )
        print(f"[run_sim] drop_zones={drop_zones_list}")

        # Collected during the conveyor loop and consumed by
        # attach_pending_publisher() afterwards. Only conveyors with a
        # pickup_target enter the list -- /cargo/pending is a fleet-dispatch
        # signal, not a generic conveyor monitor.
        pending_conveyors: list[tuple[str, str]] = []

        conv_root = stage.GetPrimAtPath("/World/Conveyors")
        if conv_root and conv_root.IsValid():
            for conv in conv_root.GetChildren():
                cd = conv.GetCustomData() or {}
                speed_topic = cd.get("ros2_speed_topic")
                status_topic = cd.get("ros2_status_topic")
                belt_prim = cd.get("belt_surface_prim")
                if not (speed_topic and status_topic and belt_prim):
                    continue
                # Direction stored as Gf.Vec3f; cast to plain tuple for the
                # controller signature.
                dir_v = cd.get("direction")
                if dir_v is None:
                    direction = (1.0, 0.0, 0.0)
                else:
                    direction = (float(dir_v[0]), float(dir_v[1]), float(dir_v[2]))

                # No initial box is spawned at startup -- boxes arrive
                # exclusively via /cargo/<conv>/spawn_cmd. We still parse
                # test_box_position/size/mass from customData because the
                # ScriptNode uses those values when it spawns each new box
                # at runtime (one per spawn_cmd payload).
                box_prim_path = None
                initial_box_id = "box01"
                spawn_pos = (0.0, 0.0, 0.0)
                spawn_size = 0.3
                spawn_mass = 5.0
                if cd.get("test_box_enabled"):
                    bp = cd.get("test_box_position")
                    if bp is None:
                        print(f"[run_sim] [conveyor {conv.GetName()}] "
                              f"test_box_enabled but test_box_position missing; "
                              f"using default spawn pose (0,0,0)")
                    else:
                        spawn_pos = (
                            float(bp[0]), float(bp[1]), float(bp[2])
                        )
                        spawn_size = float(cd.get("test_box_size", 0.3))
                        spawn_mass = float(cd.get("test_box_mass", 5.0))

                limit_axis = (
                    cd.get("limit_switch_axis")
                    if cd.get("limit_switch_enabled")
                    else None
                )
                limit_thr = (
                    float(cd.get("limit_switch_threshold", 0.0))
                    if limit_axis
                    else None
                )
                limit_cmp = (
                    cd.get("limit_switch_comparator", "ge")
                    if limit_axis
                    else None
                )

                # Pickup target wiring (box -> AMR top handoff). When the
                # limit switch fires AND a candidate robot's base_link XY is
                # within dock_radius of dock_pose, the ScriptNode flips the
                # box to kinematic and pose-locks it to that robot.
                pickup_enabled = bool(cd.get("pickup_enabled", False))
                pickup_candidates: list[str] = []
                pickup_dock_xy = (0.0, 0.0)
                pickup_dock_radius = 0.6
                pickup_attach_offset = (0.0, 0.0, 0.30)
                pickup_follow_yaw = True
                if pickup_enabled:
                    cands_csv = str(cd.get("pickup_candidates", ""))
                    pickup_candidates = [c for c in cands_csv.split(",") if c]
                    dxy = cd.get("pickup_dock_xy")
                    if dxy is not None:
                        pickup_dock_xy = (float(dxy[0]), float(dxy[1]))
                    pickup_dock_radius = float(
                        cd.get("pickup_dock_radius", 0.6)
                    )
                    aoff = cd.get("pickup_attach_offset")
                    if aoff is not None:
                        pickup_attach_offset = (
                            float(aoff[0]),
                            float(aoff[1]),
                            float(aoff[2]),
                        )
                    pickup_follow_yaw = bool(cd.get("pickup_follow_yaw", True))

                # IsaacConveyor drives surface velocity onto the belt mesh
                # via PhysxSurfaceVelocityAPI, and PhysX only treats the mesh
                # as a "moving surface that pushes other bodies" if it's also
                # a kinematic RigidBody. The Isaac asset ships the belt mesh
                # with CollisionAPI only, so apply the missing pieces here
                # (mirrors isaacsim.asset.gen.conveyor's own commands.py).
                from pxr import UsdPhysics
                from pxr import PhysxSchema
                belt = stage.GetPrimAtPath(belt_prim)
                if belt and belt.IsValid():
                    if not belt.HasAPI(UsdPhysics.RigidBodyAPI):
                        rb = UsdPhysics.RigidBodyAPI.Apply(belt)
                    else:
                        rb = UsdPhysics.RigidBodyAPI(belt)
                    rb.CreateKinematicEnabledAttr(True)
                    if not belt.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(belt)
                    if not belt.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
                        PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt)
                    print(f"[run_sim] [conveyor {conv.GetName()}] "
                          f"applied RigidBody(kinematic)+SurfaceVelocity to "
                          f"{belt_prim}")
                else:
                    print(f"[run_sim] [conveyor {conv.GetName()}] "
                          f"belt prim not found: {belt_prim} -- skipping "
                          f"physics API apply (IsaacConveyor will no-op)")

                # The A08 belt mesh is the curved looped strip and is not
                # watertight from above -- a small DynamicCuboid spawned on
                # top can tunnel through and land on the kinematic Rollers
                # body inside the conveyor frame. Rollers ship kinematic but
                # without surface velocity, so cargo touching them stays put
                # even when the belt is "running". Apply SurfaceVelocityAPI
                # to Rollers so the ScriptNode can mirror the belt's surface
                # velocity onto whatever the cargo actually contacts.
                rollers_prim_path = f"{conv.GetPath()}/Rollers"
                rollers = stage.GetPrimAtPath(rollers_prim_path)
                if rollers and rollers.IsValid():
                    if not rollers.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
                        PhysxSchema.PhysxSurfaceVelocityAPI.Apply(rollers)
                    print(f"[run_sim] [conveyor {conv.GetName()}] "
                          f"applied SurfaceVelocity to {rollers_prim_path}")
                else:
                    rollers_prim_path = ""

                # default_speed: belt auto-runs at this speed on startup.
                # Once /conveyor/<id>/speed_cmd publishes any non-zero value
                # the ScriptNode switches to honouring sub.get() literally
                # (so a subsequent 0.0 publish stops the belt).
                default_speed = float(cd.get("default_speed", 0.0))

                # Conveyors, like doors, are shared infrastructure; keep the
                # ROS2 namespace empty so any robot in the scene can drive the
                # same /conveyor/<id>/speed_cmd topic.
                attach_conveyor_controller(
                    stage,
                    conveyor_path=str(conv.GetPath()),
                    speed_topic=speed_topic,
                    status_topic=status_topic,
                    belt_surface_prim=belt_prim,
                    direction=direction,
                    box_prim=box_prim_path,
                    rollers_prim=rollers_prim_path,
                    limit_axis=limit_axis,
                    limit_threshold=limit_thr,
                    limit_comparator=limit_cmp,
                    default_speed=default_speed,
                    pickup_enabled=pickup_enabled,
                    pickup_candidates=pickup_candidates,
                    pickup_dock_xy=pickup_dock_xy,
                    pickup_dock_radius=pickup_dock_radius,
                    pickup_attach_offset=pickup_attach_offset,
                    pickup_follow_yaw=pickup_follow_yaw,
                    drop_cmd_topic="/cargo/drop_cmd",
                    drop_zones=drop_zones_list,
                    spawn_cmd_topic=f"/cargo/{conv.GetName()}/spawn_cmd",
                    pickup_cmd_topic=f"/cargo/{conv.GetName()}/pickup_cmd",
                    initial_box_id=initial_box_id,
                    box_spawn_position=spawn_pos,
                    box_spawn_size=spawn_size,
                    box_spawn_mass=spawn_mass,
                    namespace="",
                    graph_path=f"/ConveyorGraph_{conv.GetName()}",
                    debug=True,
                )

                if pickup_enabled:
                    pending_conveyors.append(
                        (conv.GetName(), status_topic)
                    )

        if pending_conveyors:
            from syncai_omniverse.ros2.pending_publisher import (
                attach_pending_publisher,
            )
            attach_pending_publisher(
                stage,
                conveyors_info=pending_conveyors,
                topic="/cargo/pending",
            )

omni.timeline.get_timeline_interface().play()

try:
    while simulation_app.is_running():
        simulation_app.update()
finally:
    simulation_app.close()
