"""Attach an RTX lidar under the SlotCar's lidar_link and publish
sensor_msgs/LaserScan on a ROS2 topic.

Graph shape:
    OnPlaybackTick ──> IsaacCreateRenderProduct(camera = lidar prim)
                                   │
                                   └─renderProductPath─> ROS2RtxLidarHelper(/scan)

Requires Isaac Sim runtime: `omni.graph.core`, `omni.kit.commands`, and the
`isaacsim.sensors.rtx` + `isaacsim.ros2.bridge` extensions must be enabled
before calling `attach_lidar_publisher`.
"""
from pxr import Sdf

from syncai_omniverse.ros2._ns import (
    apply_frame_namespace as _apply_frame_namespace,
    apply_namespace as _apply_namespace,
)


def attach_lidar_publisher(
    stage,
    robot_path: str = "/World/SlotCar",
    lidar_link: str = "lidar_link",
    lidar_name: str = "Lidar",
    config: str = "SICK_picoScan150",
    topic: str = "/scan",
    frame_id: str = "lidar_link",
    publish_type: str = "auto",
    namespace: str = "",
    graph_path: str = "/LidarActionGraph",
    debug_draw: bool = True,
) -> str:
    """
        Spawn an RTX lidar at `{robot_path}/{lidar_link}/{lidar_name}` using the
        named USD asset from `isaacsim.sensors.rtx`'s SUPPORTED_LIDAR_CONFIGS,
        then build an OmniGraph that streams `sensor_msgs/{LaserScan|PointCloud2}`
        to `topic`.

        `config` must match a stem in SUPPORTED_LIDAR_CONFIGS (e.g.
        "SICK_picoScan150", "SICK_tim781", "SICK_multiScan165"); the vendor
        folder prefix is NOT used.

        `publish_type` is "laser_scan", "point_cloud", or "auto". "auto" reads
        the config JSON's elevation range and picks `laser_scan` only for true
        2D configs (elevation=[0,0]); anything else (e.g. SICK_multiScan165
        with elevation [-7.2, +34.3]) publishes as `point_cloud`, because the
        ROS2RtxLidarHelper laser_scan mode rejects sensors with non-zero
        elevation.

        Returns the graph prim path.
    """
    import omni.graph.core as og
    import omni.kit.commands

    if publish_type == "auto":
        publish_type = _pick_publish_type(config)

    parent_path = f"{robot_path}/{lidar_link}"
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise RuntimeError(f"Lidar parent link not found: {parent_path}")

    # Pass `path` without leading slash so it is treated as relative to `parent`,
    # then trust the returned prim for the real path (the command may dedupe via
    # get_next_free_path, and a USD reference can resolve to a deeper child).
    result, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_name,
        parent=parent_path,
        config=config,
    )
    if not result or sensor is None or not sensor.IsValid():
        raise RuntimeError(
            f"IsaacSensorCreateRtxLidar failed "
            f"(config={config!r}, parent={parent_path}). "
            "Verify the config name appears in SUPPORTED_LIDAR_CONFIGS "
            "(stem only, e.g. 'SICK_picoScan150') and that the Isaac Sim "
            "assets root is reachable."
        )
    lidar_prim_path = str(sensor.GetPath())
    topic = _apply_namespace(namespace, topic)
    frame_id = _apply_frame_namespace(namespace, frame_id)
    print(f"[lidar] created RTX sensor prim at {lidar_prim_path}")

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRP.inputs:execIn"),
                ("CreateRP.outputs:execOut", "LidarHelper.inputs:execIn"),
                ("CreateRP.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CreateRP.inputs:cameraPrim", [Sdf.Path(lidar_prim_path)]),
                ("LidarHelper.inputs:topicName", topic),
                ("LidarHelper.inputs:frameId", frame_id),
                ("LidarHelper.inputs:type", publish_type),
                ("LidarHelper.inputs:fullScan", True),
            ],
        },
    )

    print(f"[lidar] graph={graph_path}  topic={topic}  type={publish_type}  frame={frame_id}")
    print(f"[lidar]   sensor prim={lidar_prim_path}  config={config}")

    if debug_draw:
        attach_lidar_debug_draw(lidar_prim_path)

    return graph_path


def attach_lidar_debug_draw(
    lidar_prim_path: str,
    color=(1.0, 0.0, 0.0, 1.0),
    size: float = 0.05,
) -> None:
    """Paint each RTX lidar return as a coloured point in the Isaac Sim viewport.

    Uses the non-accumulating writer (`RtxLidarDebugDrawPointCloud`, no
    `Buffer` suffix). Each viewport frame shows only the rays emitted during
    that tick, so the points stay in sync with the lidar's current pose --
    no motion smear while the robot drives. The tradeoff is a ~10 Hz flicker
    when the robot is stationary, because most viewport frames fall between
    rotations and have no new returns to draw.

    The buffered variant (`...Buffer`) trades this the other way: stable when
    still, smeared trails when moving, because it draws a full rotation's
    worth of points using the latest transform regardless of when each ray
    was shot. User picked no-smear; keep this variant.

    `color` (RGBA 0-1) and `size` are forwarded to the underlying
    `isaacsim.util.debug_draw.DebugDrawPointCloud` node.
    """
    import omni.replicator.core as rep

    render_product = rep.create.render_product(
        lidar_prim_path, [1, 1], name="IsaacLidarViz"
    )
    writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
    writer.initialize(color=list(color), size=size)
    writer.attach([render_product])
    print(f"[lidar] debug-draw attached to {lidar_prim_path} "
          f"(per-frame, color={tuple(color)}, size={size})")


def _pick_publish_type(config: str) -> str:
    """Peek at the shipped lidar JSON to decide laser_scan vs point_cloud.
    Defaults to `point_cloud` if the config can't be located (safer — the
    laser_scan node asserts elevation=0 and silently drops frames otherwise).
    """
    import glob
    import json
    import os

    roots = [
        "/isaac-sim/exts/isaacsim.sensors.rtx/data/lidar_configs",
        os.environ.get("ISAAC_PATH", ""),
    ]
    for root in roots:
        if not root:
            continue
        for path in glob.glob(os.path.join(root, "**", f"{config}.json"),
                              recursive=True):
            try:
                profile = json.load(open(path)).get("profile", {})
                up = profile.get("upElevationDeg", 0) or 0
                dn = profile.get("downElevationDeg", 0) or 0
                if abs(up) < 1e-6 and abs(dn) < 1e-6:
                    return "laser_scan"
                return "point_cloud"
            except Exception:
                pass
    return "point_cloud"
