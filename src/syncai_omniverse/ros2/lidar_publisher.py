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
import math

from pxr import Gf, Sdf, UsdGeom

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
    rotation_z_deg: float = 0.0,
    horizontal_fov_deg: float | None = None,
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

        `rotation_z_deg` rotates the created RTX sensor prim about its local
        Z axis (only the sensor, not the parent rigid-body link — that would
        fight the fixed joint's LocalRot). Used on MiR250 dual_diagonal to
        point front lidar's open sector forward (0°) and rear lidar's open
        sector backward (180°), placing each lidar's blind wedge on the
        opposite lidar so they don't cross-scan.

        `horizontal_fov_deg` (optional) asks the publisher to crop the
        ROS2 scan output to `horizontal_fov_deg` worth of azimuth by setting
        `horizontalMinAngle` / `horizontalMaxAngle` pins on the ROS2
        LidarHelper node. If those pins don't exist in this Isaac Sim
        build, a warning is printed and the native sensor FOV is used --
        the caller should then fall back to a custom lidar config JSON.

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

    # Rotate the RTX sensor's local frame without touching the parent
    # rigid-body link. MiR250 dual_diagonal: front=0° (open sector +X),
    # rear=180° (open sector -X) -- blind wedges face the opposite lidar.
    if rotation_z_deg != 0.0:
        UsdGeom.Xformable(sensor).AddRotateZOp().Set(float(rotation_z_deg))
        print(f"[lidar]   rotation_z={rotation_z_deg:.1f}° applied to sensor prim")

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

    if horizontal_fov_deg is not None:
        _try_set_horizontal_fov(graph_path, horizontal_fov_deg)

    print(f"[lidar] graph={graph_path}  topic={topic}  type={publish_type}  frame={frame_id}")
    print(f"[lidar]   sensor prim={lidar_prim_path}  config={config}")
    return lidar_prim_path


def _try_set_horizontal_fov(graph_path: str, fov_deg: float) -> None:
    """Crop the ROS2 scan output to `fov_deg` azimuth span by setting
    horizontalMinAngle / horizontalMaxAngle on the LidarHelper node.

    Isaac Sim 5.1 may or may not expose these pins. Introspect the node
    first; on miss, print guidance for the JSON-config fallback instead
    of crashing the graph.
    """
    import omni.graph.core as og

    helper_path = f"{graph_path}/LidarHelper"
    try:
        node = og.Controller.node(helper_path)
    except Exception as exc:
        print(f"[lidar]   fov: could not resolve LidarHelper node: {exc}")
        return

    attr_names = {a.get_name() for a in node.get_attributes()}
    min_candidates = ["inputs:horizontalMinAngle", "inputs:minAzimuthAngle",
                      "inputs:horizontalMinDeg"]
    max_candidates = ["inputs:horizontalMaxAngle", "inputs:maxAzimuthAngle",
                      "inputs:horizontalMaxDeg"]
    min_name = next((n for n in min_candidates if n in attr_names), None)
    max_name = next((n for n in max_candidates if n in attr_names), None)

    if min_name is None or max_name is None:
        # Print the angle-related attrs we saw so future debugging is one log
        # line away.
        angle_attrs = sorted(n for n in attr_names if "angle" in n.lower() or "azimuth" in n.lower())
        print(f"[lidar]   fov: horizontalMin/MaxAngle not exposed on ROS2RtxLidarHelper "
              f"(saw angle-related={angle_attrs or 'none'}). "
              f"Native sensor FOV kept; falling back to JSON config is required for true {fov_deg}° crop.")
        return

    # Most OmniGraph angle pins are radians. If the attr name contains "Deg"
    # we assume degrees instead.
    use_radians = not (min_name.endswith("Deg") or max_name.endswith("Deg"))
    half = fov_deg / 2.0
    if use_radians:
        min_val, max_val = -math.radians(half), math.radians(half)
        unit = "rad"
    else:
        min_val, max_val = -half, half
        unit = "deg"
    og.Controller.attribute(f"{helper_path}.{min_name}").set(min_val)
    og.Controller.attribute(f"{helper_path}.{max_name}").set(max_val)
    print(f"[lidar]   fov: {fov_deg:.1f}° applied via {min_name}/{max_name} "
          f"({min_val:.4f},{max_val:.4f} {unit})")


def attach_lidar_debug_draw(lidar_prim_path: str, size: float = 0.05) -> None:
    """Draw the RTX lidar's current-tick point cloud in the viewport as a
    cloud of colored points. Useful for confirming visually that rays are
    hitting the expected geometry and that the sensor's transform is right.

    Uses the `RtxLidarDebugDrawPointCloud` writer (NON-buffer). The buffer
    variant accumulates a full rotation before drawing — stable when the
    robot is still, but drags smeared trails while driving. Non-buffer
    flickers at ~10 Hz when stationary but stays clean under motion, which
    matters more for nav / teleop. See memory
    `feedback_lidar_debug_writer_choice` for the full rationale.

    `size` controls the per-point marker size in world units. Raise if
    the flicker (while stationary) makes the points hard to see.
    """
    import omni.replicator.core as rep

    # rep.create.render_product creates a 1x1 render product bound to the
    # lidar sensor prim; the debug-draw writer pulls RTX returns off it and
    # submits draw commands to the viewport every frame.
    render_product = rep.create.render_product(lidar_prim_path, [1, 1])
    writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
    writer.initialize(color=(1.0, 0.0, 0.0, 1.0), size=size)
    writer.attach([render_product])
    print(f"[lidar] debug-draw attached to {lidar_prim_path} (size={size})")


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
