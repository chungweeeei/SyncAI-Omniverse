"""Launch Isaac Sim, open a scene USD, and attach the ROS2 TF publisher.

Run inside the isaac-sim container:
    /isaac-sim/python.sh scripts/run_sim.py
    /isaac-sim/python.sh scripts/run_sim.py --scene /workspace/scenes/warehouse.usda
    /isaac-sim/python.sh scripts/run_sim.py --headless
    /isaac-sim/python.sh scripts/run_sim.py --no-ros2       # skip TF setup

Robot models:
    --robot-model=mir250    -> /World/MirAMR (MiR250-style chassis, dual lidar)
"""
import argparse
import sys
from pathlib import Path

# Per-model defaults. Keys selected by --robot-model; CLI flags still override.
_MODEL_DEFAULTS = {
    "mir250": {
        "robot_prim": "/World/MirAMR",
        "scene": "/workspace/scenes/dp1f_mir.usda",
        # MiR250 drive gains / accel limits tuned for the ~100 kg chassis:
        # enough torque to track cmd_vel without startup stall, but capped
        # so pitch transient on accel steps stays inside the caster-engage
        # envelope.
        "wheel_distance": 0.445,
        "wheel_drive_damping": 800.0,
        "wheel_drive_max_force": 40.0,
        "max_linear_accel": 1.0,
        "max_linear_decel": 1.5,
        "max_angular_accel": 1.5,
        "lidar_layout": "dual_diagonal",
        "tf_targets": "lidar_link_front:scan_front,lidar_link_rear:scan_rear",
    },
}

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--robot-model",
    default="mir250",
    choices=sorted(_MODEL_DEFAULTS),
    help="Which AMR model the scene contains. Selects per-model defaults for "
         "--robot, --scene, wheel distance, drive gains, and lidar layout. "
         "Individual CLI flags still override these.",
)
parser.add_argument(
    "--scene",
    default=None,
    help="Path to the USD stage to open. Defaults per --robot-model.",
)
parser.add_argument(
    "--robot",
    default=None,
    help="Prim path of the articulation root. Defaults per --robot-model "
         "(/World/MirAMR).",
)
parser.add_argument("--headless", action="store_true", help="Run without a window.")
parser.add_argument("--no-ros2", action="store_true", help="Skip ROS2 TF publisher setup.")
parser.add_argument(
    "--tf-targets",
    default=None,
    help="Comma-separated link names under --robot to publish (relative to --robot). "
         "Each entry may use `prim:frame_id` to decouple the USD prim from the "
         "published TF frame id. Default per --robot-model (mir250: "
         "`lidar_link_front:scan_front,lidar_link_rear:scan_rear`). Use 'auto' "
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
    "--debug-cmdvel",
    action="store_true",
    help="Verbose cmd_vel graph build log + periodic runtime snapshot of live values.",
)
parser.add_argument(
    "--debug-pose",
    action="store_true",
    help="Every 0.5s, print the robot base_link world pose (xyz + roll/pitch/yaw "
         "in degrees) and linear/angular velocity. Use to diagnose stuck/flip/pivot "
         "problems -- e.g. pitch ~180 means the chassis flipped onto its back.",
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
         "point, publishing to --cmd-vel-topic-style single /scan. "
         "dual_diagonal: two RTX lidars at front-left + rear-right, publishing "
         "/scan_front + /scan_rear on separate graphs (matches real MiR250 "
         "safety-lidar layout). Defaults per --robot-model.",
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
parser.add_argument(
    "--ros-namespace",
    default="",
    help="ROS2 namespace prepended to data topics (cmd_vel, odom, joint_states, "
         "scan). /tf and /tf_static stay global so nav2/rviz can discover the "
         "TF tree without extra remapping. Empty string = no namespace.",
)
parser.add_argument(
    "--cmd-vel-topic",
    default="/cmd_vel_smoothed",
    help="Topic the sim subscribes to for Twist commands. Defaults to the nav2 "
         "velocity_smoother output. Several nav2 publishers (controller_server + "
         "each BT recovery action) race on raw /cmd_vel, producing a 0/v sawtooth "
         "that our wheel drive can't integrate. Override to /cmd_vel when running "
         "without nav2 (e.g. raw teleop_twist_keyboard).",
)
args = parser.parse_args()

# Resolve per-model defaults. CLI flags override; unset flags inherit from
# _MODEL_DEFAULTS[args.robot_model].
_model_cfg = _MODEL_DEFAULTS[args.robot_model]
if args.scene is None:
    args.scene = _model_cfg["scene"]
if args.robot is None:
    args.robot = _model_cfg["robot_prim"]
if args.tf_targets is None:
    args.tf_targets = _model_cfg["tf_targets"]
if args.lidar_layout is None:
    args.lidar_layout = _model_cfg["lidar_layout"]
if args.lidar_parent is None:
    # Only used for single_center; dual_diagonal hard-codes both mount points.
    args.lidar_parent = "lidar_link"

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
print(f"[run_sim] robot-model={args.robot_model}  robot={args.robot}  "
      f"lidar-layout={args.lidar_layout}")
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

        attach_clock_publisher(stage)
        print(f"[run_sim] Clock publisher attached: /clock (use rviz2/nav2 with use_sim_time:=true)")

    from syncai_omniverse.ros2.tf_publisher import attach_tf_publisher

    if args.tf_targets.strip().lower() == "auto":
        target_links = None
    else:
        target_links = [s.strip() for s in args.tf_targets.split(",") if s.strip()]
    attach_tf_publisher(
        stage,
        robot_path=args.robot,
        target_links=target_links,
        namespace=args.ros_namespace,
    )
    targets_desc = ",".join(target_links) if target_links else "<auto: all rigid-body links>"
    ns_prefix = f"{args.ros_namespace}/" if args.ros_namespace else ""
    print(f"[run_sim] TF publisher attached: {ns_prefix}base_link -> /tf  targets={targets_desc}")

    if not args.no_odom:
        from syncai_omniverse.ros2.odom_publisher import attach_odom_publisher

        attach_odom_publisher(stage, robot_path=args.robot, namespace=args.ros_namespace)
        ns_prefix = f"/{args.ros_namespace}" if args.ros_namespace else ""
        print(f"[run_sim] Odom publisher attached: {ns_prefix}/odom, "
              f"{ns_prefix}/joint_states, odom->base_link on /tf")

    if not args.no_cmdvel:
        from syncai_omniverse.ros2.cmd_vel_subscriber import (
            attach_cmd_vel_subscriber,
            print_cmd_vel_snapshot,
        )

        attach_cmd_vel_subscriber(
            stage,
            robot_path=args.robot,
            namespace=args.ros_namespace,
            debug=args.debug_cmdvel,
            topic=args.cmd_vel_topic,
            wheel_distance=_model_cfg["wheel_distance"],
            wheel_drive_damping=_model_cfg["wheel_drive_damping"],
            wheel_drive_max_force=_model_cfg["wheel_drive_max_force"],
            max_linear_accel=_model_cfg["max_linear_accel"],
            max_linear_decel=_model_cfg["max_linear_decel"],
            max_angular_accel=_model_cfg["max_angular_accel"],
        )
        ns_prefix = f"/{args.ros_namespace}" if args.ros_namespace else ""
        print(f"[run_sim] cmd_vel subscriber attached: {ns_prefix}{args.cmd_vel_topic} -> "
              f"drivewhl_l/r_joint")

    if not args.no_lidar:
        from isaacsim.core.utils.extensions import enable_extension as _enable_ext

        _enable_ext("isaacsim.sensors.rtx")
        for _ in range(20):
            simulation_app.update()

        from syncai_omniverse.ros2.lidar_publisher import (
            attach_lidar_debug_draw,
            attach_lidar_publisher,
        )

        lidar_prims: list[str] = []
        if args.lidar_layout == "dual_diagonal":
            # Two RTX lidars on diagonal corners -> two separate /scan_* topics.
            # nav2 side must merge them (laser_scan_multi_merger) or consume
            # both in the obstacle layer.
            # Dual-diagonal anti-ghost geometry: front sensor rotated 0°
            # (open sector faces +X), rear rotated 180° (open sector faces -X).
            # With 250° FOV each, the 110° blind wedge on each lidar covers
            # the bearing to the opposite lidar, so they never cross-scan.
            # Union coverage is still 360° because the two 250° sectors
            # overlap 70° on each side.
            lidar_prims.append(attach_lidar_publisher(
                stage,
                robot_path=args.robot,
                lidar_link="lidar_link_front",
                lidar_name="LidarFront",
                topic="/scan_front",
                frame_id="scan_front",
                namespace=args.ros_namespace,
                config=args.lidar_config,
                publish_type=args.lidar_publish_type,
                graph_path="/LidarActionGraphFront",
                rotation_z_deg=0.0,
                horizontal_fov_deg=250.0,
            ))
            lidar_prims.append(attach_lidar_publisher(
                stage,
                robot_path=args.robot,
                lidar_link="lidar_link_rear",
                lidar_name="LidarRear",
                topic="/scan_rear",
                frame_id="scan_rear",
                namespace=args.ros_namespace,
                config=args.lidar_config,
                publish_type=args.lidar_publish_type,
                graph_path="/LidarActionGraphRear",
                rotation_z_deg=180.0,
                horizontal_fov_deg=250.0,
            ))
            ns_prefix = f"/{args.ros_namespace}" if args.ros_namespace else ""
            print(f"[run_sim] dual lidar attached: {ns_prefix}/scan_front + "
                  f"{ns_prefix}/scan_rear  config={args.lidar_config}")
        else:
            lidar_prims.append(attach_lidar_publisher(
                stage,
                robot_path=args.robot,
                lidar_link=args.lidar_parent,
                frame_id=args.lidar_frame,
                topic="/scan",
                namespace=args.ros_namespace,
                config=args.lidar_config,
                publish_type=args.lidar_publish_type,
            ))
            ns_prefix = f"/{args.ros_namespace}" if args.ros_namespace else ""
            print(f"[run_sim] lidar publisher attached: {ns_prefix}/scan  "
                  f"config={args.lidar_config}  parent={args.lidar_parent}  "
                  f"publish_type={args.lidar_publish_type}")

        if args.lidar_debug_draw:
            for prim_path in lidar_prims:
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
                open_target = float(custom.get("open_target", 0.95))
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
                    open_target=open_target,
                    namespace="",
                    graph_path=f"/DoorGraph_{name}",
                )

omni.timeline.get_timeline_interface().play()


def _find_articulation_root(stage, robot_path: str) -> str:
    """Return the first prim under `robot_path` (inclusive) that has
    `UsdPhysics.ArticulationRootAPI` applied. Falls back to `robot_path`
    itself so callers still get a usable string in the odd case where the
    API isn't found (Isaac Sim will then raise its own clearer error).
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot_path)
    if not root.IsValid():
        return robot_path
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return robot_path


def _pose_probe_factory(robot_path: str):
    """Return a no-arg callable that prints the base_link world pose plus
    per-wheel joint velocity. Lazily resolves prims because the articulation
    wakes up a few ticks after `.play()`.
    """
    from pxr import Gf, Usd, UsdGeom
    import math

    base_link_path = f"{robot_path}/base_link"
    state = {
        "prim": None,
        "last_pos": None,
        "last_t": None,
        "art_view": None,
        "wl_idx": None,
        "wr_idx": None,
    }

    def _resolve_joint_velocity_source(stage_):
        """Lazy-build an ArticulationView keyed on the robot's root path so we
        can read PhysX joint velocities directly (bypasses /joint_states lag).
        Returns (art, left_idx, right_idx) or (None, None, None) on failure.
        """
        try:
            from isaacsim.core.prims import Articulation
        except Exception:
            return None, None, None
        try:
            # `isaacsim.core.prims.Articulation` requires the pattern to match
            # a prim with both RigidBodyAPI and ArticulationRootAPI. MirAMR
            # puts ArticulationRootAPI on `base_link`, not on the
            # `/World/<Robot>` Xform, so we descend explicitly.
            art_root_path = _find_articulation_root(stage_, robot_path)
            art = Articulation(prim_paths_expr=art_root_path)
            # `initialize` binds the view to the live PhysX articulation; it
            # only works once the sim has been played for >=1 step, so we wrap
            # in try/except and retry later if it's not yet ready.
            art.initialize()
        except Exception as exc:
            if state.get("logged_art_err") != str(exc):
                print(f"[pose] articulation not ready: {exc}")
                state["logged_art_err"] = str(exc)
            return None, None, None
        names = list(art.dof_names)
        try:
            return art, names.index("drivewhl_l_joint"), names.index("drivewhl_r_joint")
        except ValueError:
            print(f"[pose] wheel dofs not found in {names}")
            return None, None, None

    def _probe():
        import time as _time
        stage_ = omni.usd.get_context().get_stage()
        if state["prim"] is None or not state["prim"].IsValid():
            state["prim"] = stage_.GetPrimAtPath(base_link_path)
            if not state["prim"].IsValid():
                print(f"[pose] {base_link_path} not found yet")
                return
        if state["art_view"] is None:
            art, wl, wr = _resolve_joint_velocity_source(stage_)
            if art is not None:
                state["art_view"] = art
                state["wl_idx"], state["wr_idx"] = wl, wr
        xf = UsdGeom.Xformable(state["prim"]).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        pos = xf.ExtractTranslation()
        m = Gf.Matrix3d(xf.ExtractRotationMatrix())
        sy = math.sqrt(m[0][0] ** 2 + m[1][0] ** 2)
        if sy > 1e-6:
            roll = math.atan2(m[2][1], m[2][2])
            pitch = math.atan2(-m[2][0], sy)
            yaw = math.atan2(m[1][0], m[0][0])
        else:
            roll = math.atan2(-m[1][2], m[1][1])
            pitch = math.atan2(-m[2][0], sy)
            yaw = 0.0
        now = _time.time()
        lin = ""
        if state["last_pos"] is not None:
            dt = now - state["last_t"]
            if dt > 0:
                dx = pos[0] - state["last_pos"][0]
                dy = pos[1] - state["last_pos"][1]
                dz = pos[2] - state["last_pos"][2]
                lin = f"  v=({dx/dt:+.3f},{dy/dt:+.3f},{dz/dt:+.3f})m/s"
        state["last_pos"] = (pos[0], pos[1], pos[2])
        state["last_t"] = now
        wheels = ""
        phys = ""
        art = state["art_view"]
        if art is not None:
            try:
                vels = art.get_joint_velocities()
                wl_v = float(vels[0, state["wl_idx"]])
                wr_v = float(vels[0, state["wr_idx"]])
                wheels = f"  wheels=(L{wl_v:+.2f},R{wr_v:+.2f})rad/s"
            except Exception as exc:
                wheels = f"  wheels=<err:{exc}>"
            # Read the articulation root's PhysX world pose directly. This
            # bypasses any USD-staleness / Fabric-writeback issues and tells
            # us definitively whether PhysX is actually translating the base.
            try:
                positions, _ = art.get_world_poses()
                px, py, pz = float(positions[0, 0]), float(positions[0, 1]), float(positions[0, 2])
                phys = f"  phys=({px:+.3f},{py:+.3f},{pz:+.3f})"
            except Exception as exc:
                phys = f"  phys=<err:{exc}>"
        print(
            f"[pose] xyz=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"rpy=({math.degrees(roll):+.1f},{math.degrees(pitch):+.1f},"
            f"{math.degrees(yaw):+.1f})deg{lin}{wheels}{phys}"
        )

    return _probe


pose_probe = _pose_probe_factory(args.robot) if args.debug_pose else None

try:
    import math as _math
    import time
    next_cmdvel_debug = time.time() + 2.0
    next_pose = time.time() + 0.5
    # Track peak |pitch| between `[pose]` prints so we catch transients that
    # would otherwise fall between 0.5s samples. Reset after each print.
    peak_pitch = {"max": 0.0, "min": 0.0}
    last_peak_reset = time.time()
    while simulation_app.is_running():
        simulation_app.update()
        now = time.time()
        if args.debug_cmdvel and not args.no_cmdvel and now >= next_cmdvel_debug:
            print_cmd_vel_snapshot()
            next_cmdvel_debug = now + 2.0
        if pose_probe is not None:
            # Fast peak sampling every frame (no printing), so transient pitch
            # peaks during cmd_vel transitions aren't missed between the 0.5s
            # human-readable lines below.
            try:
                from pxr import Gf, Usd, UsdGeom
                import omni.usd
                _stage = omni.usd.get_context().get_stage()
                _bl = _stage.GetPrimAtPath(f"{args.robot}/base_link")
                if _bl and _bl.IsValid():
                    _xf = UsdGeom.Xformable(_bl).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    _m = Gf.Matrix3d(_xf.ExtractRotationMatrix())
                    _sy = _math.sqrt(_m[0][0] ** 2 + _m[1][0] ** 2)
                    _pitch_deg = _math.degrees(_math.atan2(-_m[2][0], _sy))
                    if _pitch_deg > peak_pitch["max"]:
                        peak_pitch["max"] = _pitch_deg
                    if _pitch_deg < peak_pitch["min"]:
                        peak_pitch["min"] = _pitch_deg
            except Exception:
                pass
            if now >= next_pose:
                print(f"[pose_peak_since_prev] pitch in "
                      f"[{peak_pitch['min']:+.2f}, {peak_pitch['max']:+.2f}] deg")
                peak_pitch["max"] = 0.0
                peak_pitch["min"] = 0.0
                pose_probe()
                next_pose = now + 0.5
finally:
    simulation_app.close()
