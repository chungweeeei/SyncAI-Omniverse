"""Attach an OmniGraph that subscribes to std_msgs/Bool on /door/<name>/cmd
and slides a double-leaf door by writing each leaf's xformOp:translate.

Graph shape (on-demand pipeline, fires once per physics substep):
    OnPhysicsStep ──┬─> ROS2Subscriber(std_msgs/Bool)  (dynamic outputs:data)
                    └─> ScriptNode -- reads sub.data lazily, writes
                                      kinematic leaf xformOps

Design:

* `isaacsim.ros2.bridge.ROS2Subscriber` is the generic bridge node. Set
  messagePackage/messageSubfolder/messageName and it materialises
  outputs matching the message fields -- `outputs:data` (bool) for
  std_msgs/Bool. We read it via og.Controller.attribute().get() inside
  the ScriptNode, NOT via a graph CONNECT (the dynamic output is
  placeholder-typed at connect time, which causes
  "cannot connect path to bool" errors).

* Leaves are kinematic RigidBodies (see `auto_door.py` for rationale).
  Moving them is a USD xformOp:translate write -- PhysX picks up the
  new pose via Fabric sync and teleports the collider. This bypasses
  the articulation/drive stack entirely. An earlier design tried
  IsaacArticulationController with positionCommand writing drive
  targets; no configuration I tried (ArticulationRootAPI on Xform,
  kinematic base, dynamic base + FixedJoint-to-world) resulted in
  leaf motion. Writing xformOps directly is simpler and works.

* The ScriptNode reads per-leaf customData authored by auto_door.py
  (`closed_y`, `open_y`, `leaf_z`) so it knows where to slide. A
  linear interpolation with a per-tick step makes the motion smooth
  rather than an instant snap.

Bool -> leaf target mapping:
    true  -> each leaf interpolates toward its authored open_y
    false -> each leaf interpolates toward its authored closed_y

Requires Isaac Sim runtime; do not import before `SimulationApp` is up.
"""
from syncai_omniverse.ros2._ns import apply_namespace as _apply_namespace


# Runs inside omni.graph.scriptnode.ScriptNode. Module-level globals
# persist across compute calls.
_DOOR_SCRIPT = """\
_state = {
    "sub_attr": None,
    "left_prim": None,
    "right_prim": None,
    "left_xform_op": None,
    "right_xform_op": None,
    "left_closed_y": 0.0,
    "left_open_y": 0.0,
    "right_closed_y": 0.0,
    "right_open_y": 0.0,
    "leaf_z": 0.0,
    "left_cur_y": 0.0,
    "right_cur_y": 0.0,
}

# Max slide speed (m / physics-tick). At 60 Hz physics, 0.03 m/tick =
# 1.8 m/s, so a 1.45 m opening completes in ~0.8 s.
_STEP = 0.03


def setup(db):
    import omni.graph.core as og
    import omni.usd
    try:
        _state["sub_attr"] = og.Controller.attribute(str(db.inputs.subAttrPath))
    except Exception:
        _state["sub_attr"] = None
    stage = omni.usd.get_context().get_stage()
    left_path = str(db.inputs.leftPrimPath)
    right_path = str(db.inputs.rightPrimPath)
    _state["left_prim"] = stage.GetPrimAtPath(left_path)
    _state["right_prim"] = stage.GetPrimAtPath(right_path)
    # Pull per-leaf slide bounds from customData authored by auto_door.py.
    lcd = _state["left_prim"].GetCustomData() or {}
    rcd = _state["right_prim"].GetCustomData() or {}
    _state["left_closed_y"] = float(lcd.get("closed_y", 0.0))
    _state["left_open_y"] = float(lcd.get("open_y", 0.0))
    _state["right_closed_y"] = float(rcd.get("closed_y", 0.0))
    _state["right_open_y"] = float(rcd.get("open_y", 0.0))
    _state["leaf_z"] = float(lcd.get("leaf_z", 0.0))
    _state["left_cur_y"] = _state["left_closed_y"]
    _state["right_cur_y"] = _state["right_closed_y"]
    # Resolve each leaf's existing translate op once so we can rewrite it
    # fast each tick (avoids scanning xformOpOrder per compute).
    from pxr import UsdGeom
    for prim, key in [(_state["left_prim"], "left_xform_op"),
                      (_state["right_prim"], "right_xform_op")]:
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                _state[key] = op
                break
        if _state[key] is None:
            # auto_door.py always adds a translate op, but fall back to
            # creating one if some other authoring tool stripped it.
            _state[key] = xf.AddTranslateOp()


def _step_toward(cur, target, step):
    if cur < target:
        return min(cur + step, target)
    if cur > target:
        return max(cur - step, target)
    return cur


def compute(db):
    import omni.graph.core as og
    from pxr import Gf
    sub = _state["sub_attr"]
    if sub is None:
        try:
            _state["sub_attr"] = og.Controller.attribute(str(db.inputs.subAttrPath))
            sub = _state["sub_attr"]
        except Exception:
            sub = None
    is_open = False
    if sub is not None:
        try:
            is_open = bool(sub.get())
        except Exception:
            is_open = False

    left_target = _state["left_open_y"] if is_open else _state["left_closed_y"]
    right_target = _state["right_open_y"] if is_open else _state["right_closed_y"]
    _state["left_cur_y"] = _step_toward(_state["left_cur_y"], left_target, _STEP)
    _state["right_cur_y"] = _step_toward(_state["right_cur_y"], right_target, _STEP)

    z = _state["leaf_z"]
    if _state["left_xform_op"]:
        _state["left_xform_op"].Set(Gf.Vec3d(0.0, _state["left_cur_y"], z))
    if _state["right_xform_op"]:
        _state["right_xform_op"].Set(Gf.Vec3d(0.0, _state["right_cur_y"], z))
    return True
"""


def attach_door_controller(
    stage,
    door_path: str,
    topic: str,
    open_target: float = 0.95,
    namespace: str = "",
    graph_path: str | None = None,
    debug: bool = False,
    # `left_joint` / `right_joint` kept in the signature for source
    # compatibility with run_sim.py's current call site; not used now
    # that the controller writes xformOps directly on the leaves.
    left_joint: str = "leaf_left_joint",
    right_joint: str = "leaf_right_joint",
) -> str:
    """Build (or overwrite) the door command graph for one door."""
    import omni.graph.core as og

    door_prim = stage.GetPrimAtPath(door_path)
    if not door_prim.IsValid():
        raise RuntimeError(f"Door prim not found: {door_path}")

    left_path = f"{door_path}/leaf_left"
    right_path = f"{door_path}/leaf_right"
    for p in (left_path, right_path):
        if not stage.GetPrimAtPath(p).IsValid():
            raise RuntimeError(f"Door leaf prim not found: {p}")

    topic = _apply_namespace(namespace, topic)
    graph_path = graph_path or f"/DoorGraph_{door_prim.GetName()}"
    sub_attr_path = f"{graph_path}/SubBool.outputs:data"

    create_nodes = [
        ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubBool", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ("DoorScript", "omni.graph.scriptnode.ScriptNode"),
    ]
    create_attributes = [
        ("DoorScript.inputs:subAttrPath", "string"),
        ("DoorScript.inputs:leftPrimPath", "string"),
        ("DoorScript.inputs:rightPrimPath", "string"),
    ]
    connect = [
        ("OnPhysics.outputs:step", "SubBool.inputs:execIn"),
        ("OnPhysics.outputs:step", "DoorScript.inputs:execIn"),
    ]
    set_values = [
        ("SubBool.inputs:topicName", topic),
        ("SubBool.inputs:messagePackage", "std_msgs"),
        ("SubBool.inputs:messageSubfolder", "msg"),
        ("SubBool.inputs:messageName", "Bool"),
        ("DoorScript.inputs:script", _DOOR_SCRIPT),
        ("DoorScript.inputs:subAttrPath", sub_attr_path),
        ("DoorScript.inputs:leftPrimPath", left_path),
        ("DoorScript.inputs:rightPrimPath", right_path),
    ]

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CREATE_ATTRIBUTES: create_attributes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    print(f"[door] graph={graph_path}  topic={topic}  open_target={open_target}")
    print(f"[door]   leaves: {left_path}, {right_path}")

    if debug:
        print(f"[door][debug] nodes:")
        for name, type_ in create_nodes:
            print(f"[door][debug]   {graph_path}/{name}  <{type_}>")

    return graph_path
