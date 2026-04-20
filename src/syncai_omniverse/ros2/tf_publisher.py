"""Attach an OmniGraph that streams robot link transforms to ROS2 /tf.

Requires Isaac Sim runtime: `omni.graph.core` and the `isaacsim.ros2.bridge`
extension must be enabled before calling `attach_tf_publisher`.
Do NOT import this module before `SimulationApp` is instantiated.
"""
from pxr import UsdPhysics, Sdf


def attach_tf_publisher(
    stage,
    robot_path: str = "/World/SlotCar",
    parent_link: str = "base_link",
    target_links: list[str] | None = None,
    topic: str = "/tf",
    graph_path: str = "/TFActionGraph",
) -> str:
    """
        Build (or overwrite) an OmniGraph that publishes the articulation's link
        transforms to `topic` on every playback tick.

        Parent frame is `{robot_path}/{parent_link}` (defaults to base_link).
        If `target_links` is provided, only those link names (relative to
        `robot_path`) are published. Otherwise every direct child prim of
        `robot_path` that has UsdPhysics.RigidBodyAPI applied is published,
        excluding the parent link itself.

        Returns the graph prim path.
    """
    import omni.graph.core as og

    # Step 1: Validate the robot prim, parent link, and target links
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_path}")

    parent_prim_path = f"{robot_path}/{parent_link}"
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    if not parent_prim.IsValid():
        raise RuntimeError(f"Parent link not found: {parent_prim_path}")

    if target_links:
        target_paths = []
        for name in target_links:
            path = f"{robot_path}/{name}"
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Target link not found: {path}")
            target_paths.append(Sdf.Path(path))
    else:
        target_paths = []
        for child in robot_prim.GetChildren():
            if not child.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if child.GetName() == parent_link:
                continue
            target_paths.append(child.GetPath())
        if not target_paths:
            raise RuntimeError(f"No rigid-body child links under {robot_path}")


    # Step 2: use omni.graph.core to publish ROS topic 
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF.inputs:topicName", topic),
                ("PublishTF.inputs:parentPrim", [Sdf.Path(parent_prim_path)]),
                ("PublishTF.inputs:targetPrims", target_paths),
            ],
        },
    )
    return graph_path
