"""
Startup script for Isaac Sim --exec mode.
Builds the scene (STL floor plan or procedural warehouse), spawns the robot,
and sets up ROS2 sensor bridges.
"""

import os
import sys
import asyncio
import traceback

import carb

# Add src/ to path so we can import our modules
sys.path.insert(0, "/workspace/src")

# ------------------------------------------------------------------ #
#  Configuration (inline — avoids yaml dependency in container)
# ------------------------------------------------------------------ #
CONFIG = {
    "scene_mode": "stl",  # "warehouse" or "stl"
    "stl": {
        "file_path": "/workspace/models/dp1f/dp1f.stl",
        "scale": 0.1,
        "center_xy": True,
        "ground_plane_size": [20.0, 30.0],
    },
    "warehouse": {
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
    },
    "robot": {
        "model": "turtlebot3_burger",
        "spawn_position": [0.0, 0.0, 0.05],
        "wheel_radius": 0.033,
        "wheel_base": 0.160,
        "max_linear_speed": 0.22,
        "max_angular_speed": 2.84,
    },
}

# Try loading from YAML config if pyyaml is available
CONFIG_PATH = "/workspace/config/sim_config.yaml"
try:
    import yaml
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as f:
            CONFIG = yaml.safe_load(f)
        carb.log_info(f"Loaded config from {CONFIG_PATH}")
except ImportError:
    carb.log_warn("pyyaml not available — using built-in config defaults")
except Exception as e:
    carb.log_warn(f"Failed to load config: {e} — using defaults")

SCENE_PATH = "/tmp/scenes/warehouse.usda"


# ------------------------------------------------------------------ #
#  Step 1: Build warehouse scene
# ------------------------------------------------------------------ #
def build_scene():
    """Build the scene USD file (STL floor plan or procedural warehouse)."""
    os.makedirs(os.path.dirname(SCENE_PATH), exist_ok=True)

    scene_mode = CONFIG.get("scene_mode", "warehouse")

    if scene_mode == "stl":
        from syncai_omniverse.stl_to_usd import stl_to_usd

        stl_cfg = CONFIG["stl"]
        carb.log_warn(f"=== Building STL scene from {stl_cfg['file_path']} -> {SCENE_PATH} ===")
        saved = stl_to_usd(stl_cfg["file_path"], SCENE_PATH, stl_cfg)
        carb.log_warn(f"=== STL scene saved: {saved} ===")
    else:
        from syncai_omniverse.warehouse_builder import WarehouseBuilder

        carb.log_warn(f"=== Building warehouse scene -> {SCENE_PATH} ===")
        builder = WarehouseBuilder(CONFIG["warehouse"], SCENE_PATH)
        saved = builder.build()
        carb.log_warn(f"=== Warehouse scene saved: {saved} ===")


try:
    build_scene()
except Exception as e:
    carb.log_error(f"Failed to build scene: {e}")
    carb.log_error(traceback.format_exc())


# ------------------------------------------------------------------ #
#  Step 2: Load scene + spawn robot
# ------------------------------------------------------------------ #
async def setup_scene():
    """Open the warehouse stage, add the TurtleBot robot, and start simulation."""
    import omni.usd
    import omni.timeline
    from pxr import UsdGeom, Gf, Usd
    from isaacsim.storage.native import get_assets_root_path

    try:
        if not os.path.exists(SCENE_PATH):
            carb.log_error(f"Scene file does not exist: {SCENE_PATH}")
            return

        carb.log_warn(f"=== Opening stage: {SCENE_PATH} ===")

        usd_context = omni.usd.get_context()
        result, error = await usd_context.open_stage_async(SCENE_PATH)
        if not result:
            carb.log_error(f"Failed to open stage: {error}")
            return

        stage = usd_context.get_stage()
        carb.log_warn("=== Stage opened successfully ===")

        # -- Load TurtleBot from Nucleus assets --
        robot_cfg = CONFIG["robot"]
        spawn_pos = robot_cfg["spawn_position"]

        assets_root = get_assets_root_path()
        if assets_root is None:
            carb.log_error("Cannot resolve Isaac Sim assets root path.")
            return

        turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd"
        carb.log_warn(f"=== Loading TurtleBot from: {turtlebot_usd} ===")

        robot_prim_path = "/World/Robot"
        robot_prim = stage.DefinePrim(robot_prim_path)
        robot_prim.GetReferences().AddReference(turtlebot_usd)

        xformable = UsdGeom.Xformable(robot_prim)
        # Use existing translateOp if the referenced USD already defines one
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(spawn_pos[0], spawn_pos[1], spawn_pos[2]))

        carb.log_warn(f"=== TurtleBot robot added at {spawn_pos} ===")

        # Wait for asset to load
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # -- Verify the robot loaded --
        robot_check = stage.GetPrimAtPath(robot_prim_path)
        if robot_check.IsValid():
            children = [c.GetName() for c in robot_check.GetChildren()]
            carb.log_warn(f"[DIAG] Robot children: {children}")
        else:
            carb.log_error("=== Robot prim invalid after loading ===")

        # -- Tune wheel joint drives for better torque --
        from pxr import UsdPhysics
        for joint_name in ["wheel_left_joint", "wheel_right_joint"]:
            joint_prim = stage.GetPrimAtPath(f"{robot_prim_path}/joints/{joint_name}")
            if not joint_prim.IsValid():
                carb.log_warn(f"[DIAG] Joint not found: {joint_name}")
                continue

            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

            # Log current values
            cur_damping = drive.GetDampingAttr().Get()
            cur_stiffness = drive.GetStiffnessAttr().Get()
            cur_max_force = drive.GetMaxForceAttr().Get()
            carb.log_warn(f"[DIAG] {joint_name}: stiffness={cur_stiffness}, damping={cur_damping}, maxForce={cur_max_force}")

            # Set strong velocity drive: stiffness=0 (pure velocity mode), high damping & force
            drive.GetStiffnessAttr().Set(0.0)
            drive.GetDampingAttr().Set(10000.0)
            drive.GetMaxForceAttr().Set(100000.0)

        carb.log_warn("=== Wheel joint drives tuned ===")

        # -- Enable ROS2 Bridge --
        await _setup_ros2_bridge(stage, robot_prim_path)

        # -- Setup /cmd_vel control --
        await _setup_cmd_vel_control(stage, robot_prim_path)

        # -- Start simulation --
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        carb.log_warn("=== Simulation started ===")

    except Exception as e:
        carb.log_error(f"setup_scene failed: {e}")
        carb.log_error(traceback.format_exc())


# ------------------------------------------------------------------ #
#  Step 2.5: ROS2 Bridge for TurtleBot's sensors
# ------------------------------------------------------------------ #
async def _setup_ros2_bridge(stage, robot_prim_path: str):
    """Add a physics-based lidar to TurtleBot's base_scan and bridge to ROS2 /scan."""
    import omni.kit.app

    app = omni.kit.app.get_app()
    ext_manager = app.get_extension_manager()

    # Enable required extensions
    for ext in ["omni.isaac.ros2_bridge", "omni.isaac.range_sensor"]:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)
            carb.log_warn(f"=== Enabled extension: {ext} ===")

    for _ in range(20):
        await app.next_update_async()

    # ---- Step 1: Create a physics-based lidar under base_scan ----
    from pxr import Usd, UsdGeom
    from omni.isaac.range_sensor import _range_sensor

    lidar_parent = f"{robot_prim_path}/base_scan"
    lidar_prim_path = f"{lidar_parent}/lidar_sensor"

    parent_prim = stage.GetPrimAtPath(lidar_parent)
    if not parent_prim.IsValid():
        carb.log_error(f"=== base_scan prim not found at {lidar_parent} ===")
        return

    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    result, sensor = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="lidar_sensor",
        parent=lidar_parent,
        min_range=0.12,           # TurtleBot3 LDS-01 spec
        max_range=3.5,            # TurtleBot3 LDS-01 spec
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=1.0,
        horizontal_resolution=1.0,
        vertical_resolution=1.0,
        rotation_rate=5.0,        # 5 Hz like real LDS-01
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )

    carb.log_warn(f"=== Created physics lidar (result={result}) ===")

    for _ in range(10):
        await app.next_update_async()

    # Verify the lidar prim actually exists at the expected path
    lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
    if not lidar_prim.IsValid():
        # Search for it — the command may have placed it elsewhere
        from pxr import Usd
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World")):
            if prim.GetTypeName() == "Lidar":
                lidar_prim_path = prim.GetPath().pathString
                carb.log_warn(f"=== Lidar prim found at: {lidar_prim_path} ===")
                break
        else:
            carb.log_error("=== Lidar prim not found anywhere after creation ===")
            return
    else:
        carb.log_warn(f"=== Lidar prim confirmed at: {lidar_prim_path} ===")

    # ---- Step 2: Build OmniGraph to publish /scan via ROS2 ----
    import omni.graph.core as og

    all_node_types = og.get_registered_nodes()

    READER_CANDIDATES = [
        "isaacsim.sensors.physx.IsaacReadLidarBeams",
        "omni.isaac.range_sensor.IsaacReadLidarBeams",
        "omni.isaac.sensor.IsaacReadLidarBeams",
    ]
    PUB_CANDIDATES = [
        "isaacsim.ros2.bridge.ROS2PublishLaserScan",
        "omni.isaac.ros2_bridge.ROS2PublishLaserScan",
    ]
    TICK_CANDIDATES = [
        "omni.graph.action.OnPlaybackTick",
        "isaacsim.core.nodes.OnPlaybackTick",
        "omni.graph.action.OnTick",
    ]

    reader_node = next((c for c in READER_CANDIDATES if c in all_node_types), None)
    pub_node = next((c for c in PUB_CANDIDATES if c in all_node_types), None)
    tick_node = next((c for c in TICK_CANDIDATES if c in all_node_types), None)

    carb.log_warn(f"[DIAG] OG nodes: tick={tick_node}, reader={reader_node}, pub={pub_node}")

    if not all([reader_node, pub_node, tick_node]):
        carb.log_error("=== Missing OG nodes for ROS2 LaserScan ===")
        return

    try:
        keys = og.Controller.Keys
        graph_path = "/World/ROS2_Lidar_Graph"

        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_tick", tick_node),
                    ("read_lidar", reader_node),
                    ("pub_scan", pub_node),
                ],
                keys.SET_VALUES: [
                    ("pub_scan.inputs:topicName", "/scan"),
                    ("pub_scan.inputs:frameId", "base_scan"),
                ],
                keys.CONNECT: [
                    ("on_tick.outputs:tick", "read_lidar.inputs:execIn"),
                    ("read_lidar.outputs:execOut", "pub_scan.inputs:execIn"),
                    ("read_lidar.outputs:azimuthRange", "pub_scan.inputs:azimuthRange"),
                    ("read_lidar.outputs:depthRange", "pub_scan.inputs:depthRange"),
                    ("read_lidar.outputs:horizontalFov", "pub_scan.inputs:horizontalFov"),
                    ("read_lidar.outputs:horizontalResolution", "pub_scan.inputs:horizontalResolution"),
                    ("read_lidar.outputs:intensitiesData", "pub_scan.inputs:intensitiesData"),
                    ("read_lidar.outputs:linearDepthData", "pub_scan.inputs:linearDepthData"),
                    ("read_lidar.outputs:numCols", "pub_scan.inputs:numCols"),
                    ("read_lidar.outputs:numRows", "pub_scan.inputs:numRows"),
                    ("read_lidar.outputs:rotationRate", "pub_scan.inputs:rotationRate"),
                ],
            },
        )

        # Set lidarPrim target via USD relationship (OG target type)
        from pxr import Sdf
        read_lidar_og_path = f"{graph_path}/read_lidar"
        read_lidar_prim = stage.GetPrimAtPath(read_lidar_og_path)
        rel = read_lidar_prim.CreateRelationship("inputs:lidarPrim", False)
        rel.SetTargets([Sdf.Path(lidar_prim_path)])

        carb.log_warn(f"=== ROS2 LaserScan graph created — publishing /scan ===")

    except Exception as e:
        carb.log_error(f"Failed to create ROS2 OmniGraph: {e}")
        carb.log_error(traceback.format_exc())


# ------------------------------------------------------------------ #
#  Step 3: /cmd_vel → Differential Drive → Articulation Controller
# ------------------------------------------------------------------ #
async def _setup_cmd_vel_control(stage, robot_prim_path: str):
    """Subscribe to /cmd_vel and drive TurtleBot wheels via OmniGraph."""
    import omni.graph.core as og

    robot_cfg = CONFIG["robot"]

    try:
        keys = og.Controller.Keys
        graph_path = "/World/ROS2_CmdVel_Graph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_tick", "omni.graph.action.OnPlaybackTick"),
                    ("subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("break_linear", "omni.graph.nodes.BreakVector3"),
                    ("break_angular", "omni.graph.nodes.BreakVector3"),
                    ("diff_ctrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
                    ("art_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("subscribe_twist.inputs:topicName", "/cmd_vel"),
                    ("diff_ctrl.inputs:wheelRadius", robot_cfg.get("wheel_radius", 0.033)),
                    ("diff_ctrl.inputs:wheelDistance", robot_cfg.get("wheel_base", 0.160)),
                    ("diff_ctrl.inputs:maxLinearSpeed", robot_cfg.get("max_linear_speed", 0.22)),
                    ("diff_ctrl.inputs:maxAngularSpeed", robot_cfg.get("max_angular_speed", 2.84)),
                    ("art_ctrl.inputs:robotPath", robot_prim_path),
                    ("art_ctrl.inputs:jointNames", ["wheel_left_joint", "wheel_right_joint"]),
                ],
                keys.CONNECT: [
                    # Execution chain
                    ("on_tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                    ("subscribe_twist.outputs:execOut", "diff_ctrl.inputs:execIn"),
                    ("on_tick.outputs:tick", "art_ctrl.inputs:execIn"),
                    # Data flow
                    ("diff_ctrl.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
                    # Break linear velocity vector → x component (forward speed)
                    ("subscribe_twist.outputs:linearVelocity", "break_linear.inputs:tuple"),
                    ("break_linear.outputs:x", "diff_ctrl.inputs:linearVelocity"),
                    # Break angular velocity vector → z component (yaw rate)
                    ("subscribe_twist.outputs:angularVelocity", "break_angular.inputs:tuple"),
                    ("break_angular.outputs:z", "diff_ctrl.inputs:angularVelocity"),
                ],
            },
        )

        carb.log_warn("=== /cmd_vel control graph created — ready for teleop ===")

    except Exception as e:
        carb.log_error(f"Failed to create cmd_vel OmniGraph: {e}")
        carb.log_error(traceback.format_exc())


# ------------------------------------------------------------------ #
#  Launch
# ------------------------------------------------------------------ #
carb.log_warn("=== startup_scene.py loaded ===")
asyncio.ensure_future(setup_scene())
