"""Internal helper: prepend a ROS2 namespace to an absolute topic name.

Example: _apply_namespace("tb3", "/cmd_vel") -> "/tb3/cmd_vel".
Empty or whitespace namespace leaves the topic unchanged.
"""


def apply_namespace(namespace: str, topic: str) -> str:
    """Prefix `/<namespace>` onto an absolute topic; no-op if namespace is empty."""
    ns = (namespace or "").strip().strip("/")
    if not ns:
        return topic
    if not topic.startswith("/"):
        topic = "/" + topic
    return f"/{ns}{topic}"


def apply_frame_namespace(namespace: str, frame_id: str) -> str:
    """Prefix `<namespace>/` onto a TF frame_id (tf_prefix convention);
    no-op if namespace is empty. Frame ids have no leading slash."""
    ns = (namespace or "").strip().strip("/")
    if not ns:
        return frame_id
    return f"{ns}/{frame_id.lstrip('/')}"


def find_articulation_root(stage, robot_path: str) -> str:
    """Return the first prim under `robot_path` (inclusive) that has
    `UsdPhysics.ArticulationRootAPI` applied. Falls back to `robot_path`
    on a miss.

    OmniGraph nodes like IsaacArticulationController and ROS2PublishJointState
    take `targetPrim` and expect it to resolve to an articulation's root rigid
    body. When we author ArticulationRootAPI on `base_link` (not on the
    wrapping Xform), passing the Xform path trips Isaac Sim's internal
    `Articulation(prim_paths_expr=...)` view with "did not match any rigid
    bodies". Call this helper to get the actual root path.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot_path)
    if not root.IsValid():
        return robot_path
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return robot_path
