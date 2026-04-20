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


def attach_lidar_publisher(
    stage,
    robot_path: str = "/World/SlotCar",
    lidar_link: str = "lidar_link",
    lidar_name: str = "Lidar",
    config: str = "SICK_picoScan150",
    topic: str = "/scan",
    frame_id: str = "lidar_link",
    publish_type: str = "laser_scan",
    graph_path: str = "/LidarActionGraph",
) -> str:
    """
        Spawn an RTX lidar at `{robot_path}/{lidar_link}/{lidar_name}` using the
        named USD asset from `isaacsim.sensors.rtx`'s SUPPORTED_LIDAR_CONFIGS,
        then build an OmniGraph that streams `sensor_msgs/{LaserScan|PointCloud2}`
        to `topic`.

        `config` must match a stem in SUPPORTED_LIDAR_CONFIGS (e.g.
        "SICK_picoScan150", "SICK_TIM781", "RPLIDAR_S2E"); the vendor folder
        prefix is NOT used.

        `publish_type` must be "laser_scan" or "point_cloud".

        Returns the graph prim path.
    """
    import omni.graph.core as og
    import omni.kit.commands

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
            ],
        },
    )

    print(f"[lidar] graph={graph_path}  topic={topic}  type={publish_type}  frame={frame_id}")
    print(f"[lidar]   sensor prim={lidar_prim_path}  config={config}")
    return graph_path
