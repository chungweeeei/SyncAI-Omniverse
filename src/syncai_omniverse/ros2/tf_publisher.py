"""Attach an OmniGraph that publishes the robot's internal TF tree
(base_link -> <child_link>) to ROS2 /tf.

Each target link becomes its own `ROS2PublishRawTransformTree` node so we
can set explicit `parentFrameId` / `childFrameId` strings -- the richer
`ROS2PublishTransformTree` node derives frame ids from prim names, which
makes multi-robot tf_prefix (e.g. `robot01/base_link`) impossible.

The offsets are read from USD at graph-build time and treated as static
(TurtleBot3's `base_scan`, wheel mounts, etc. are rigidly attached to
`base_link`). Joint rotation of the wheels is reported via /joint_states
for robot_state_publisher to consume.

Requires Isaac Sim runtime: `omni.graph.core` and the `isaacsim.ros2.bridge`
extension must be enabled before calling `attach_tf_publisher`.
Do NOT import this module before `SimulationApp` is instantiated.
"""
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from syncai_omniverse.ros2._ns import apply_frame_namespace as _apply_frame_namespace


def attach_tf_publisher(
    stage,
    robot_path: str = "/World/SlotCar",
    parent_link: str = "base_link",
    target_links: list[str] | None = None,
    topic: str = "/tf",
    namespace: str = "",
    graph_path: str = "/TFActionGraph",
) -> str:
    """
        Build (or overwrite) an OmniGraph that publishes each
        `{parent_link} -> {target_link}` transform to `topic` every playback
        tick, using static offsets read from USD at build time.

        Frame ids follow the tf_prefix convention when `namespace` is set,
        e.g. `namespace="robot01"` produces `robot01/base_link ->
        robot01/base_scan`.

        If `target_links` is provided, only those link names (relative to
        `robot_path`) are published. Each entry may use `prim_name:frame_id`
        to decouple the USD prim name from the published TF frame id
        (e.g. `base_scan:scan` broadcasts `<ns>/base_link -> <ns>/scan` for
        the USD prim `base_scan`). A bare `name` reuses the prim name as the
        frame id. Otherwise every direct child prim of `robot_path` that has
        UsdPhysics.RigidBodyAPI applied is published, excluding the parent
        link itself.

        Returns the graph prim path.
    """
    import omni.graph.core as og
    import omni.kit.commands

    # Step 0: wipe any pre-existing graph at graph_path. `og.Controller.edit`
    # only adds/updates nodes -- stale nodes from a previous build (e.g. the
    # old `ROS2PublishTransformTree` that derived frame ids from prim names)
    # would keep publishing alongside the new per-target Raw TF nodes,
    # leaving un-namespaced `base_link -> base_scan` entries on /tf. Use the
    # Kit DeletePrims command so the OmniGraph runtime releases the graph
    # (raw `stage.RemovePrim` leaves the compiled graph alive in some builds).
    if stage.GetPrimAtPath(graph_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

    # Step 1: Validate the robot prim, parent link, and target links
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_path}")

    parent_prim_path = f"{robot_path}/{parent_link}"
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    if not parent_prim.IsValid():
        raise RuntimeError(f"Parent link not found: {parent_prim_path}")

    # Each target is (prim_name, frame_id). The `prim:frame` split lets us
    # keep the USD link name (e.g. `base_scan`) while publishing a cleaner
    # ROS frame id (e.g. `scan`).
    if target_links:
        targets: list[tuple[str, str]] = []
        for entry in target_links:
            if ":" in entry:
                prim_name, frame_id = entry.split(":", 1)
            else:
                prim_name, frame_id = entry, entry
            if not stage.GetPrimAtPath(f"{robot_path}/{prim_name}").IsValid():
                raise RuntimeError(f"Target link not found: {robot_path}/{prim_name}")
            targets.append((prim_name, frame_id))
    else:
        targets = []
        for child in robot_prim.GetChildren():
            if not child.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if child.GetName() == parent_link:
                continue
            targets.append((child.GetName(), child.GetName()))
        if not targets:
            raise RuntimeError(f"No rigid-body child links under {robot_path}")

    # Step 2: compute each child's pose in parent's local frame (USD uses
    # row-vector math, so `child_in_parent = child_world * parent_world^-1`).
    parent_world = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    parent_world_inv = parent_world.GetInverse()

    parent_frame_id = _apply_frame_namespace(namespace, parent_link)

    create_nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ]
    connect = []
    set_values = []

    for i, (prim_name, raw_frame_id) in enumerate(targets):
        child_prim = stage.GetPrimAtPath(f"{robot_path}/{prim_name}")
        child_world = UsdGeom.Xformable(child_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        rel = child_world * parent_world_inv
        translation = rel.ExtractTranslation()
        quat = rel.ExtractRotation().GetQuat()
        imag = quat.GetImaginary()
        # ROS quaternion order is (x, y, z, w); Gf stores imag=(x,y,z), real=w.
        rotation = (float(imag[0]), float(imag[1]), float(imag[2]), float(quat.GetReal()))

        node = f"PubTF_{i}"
        child_frame_id = _apply_frame_namespace(namespace, raw_frame_id)

        create_nodes.append((node, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        connect.extend([
            ("OnTick.outputs:tick", f"{node}.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", f"{node}.inputs:timeStamp"),
        ])
        set_values.extend([
            (f"{node}.inputs:topicName", topic),
            (f"{node}.inputs:parentFrameId", parent_frame_id),
            (f"{node}.inputs:childFrameId", child_frame_id),
            (f"{node}.inputs:translation",
             (float(translation[0]), float(translation[1]), float(translation[2]))),
            (f"{node}.inputs:rotation", rotation),
        ])

    # Step 3: author the graph
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )
    print(
        f"[tf] graph={graph_path}  topic={topic}  parent={parent_frame_id}  "
        f"targets={[_apply_frame_namespace(namespace, f) for _, f in targets]}"
    )
    return graph_path
