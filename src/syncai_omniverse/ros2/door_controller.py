"""Attach an OmniGraph that subscribes to std_msgs/Bool on /door/<id>/cmd_topic,
slides a double-leaf door by writing each leaf's xformOp:translate, and
publishes a std_msgs/String state machine on /door/<id>/state.

Graph shape (on-demand pipeline, fires once per physics substep):
    OnPhysicsStep ──┬─> ROS2Subscriber(std_msgs/Bool)  (dynamic outputs:data)
                    └─> ScriptNode -- reads sub.data, slides leaves,
                                      derives state, publishes /state
                                      directly via rclpy

Design:

* `isaacsim.ros2.bridge.ROS2Subscriber` is the generic bridge node. Set
  messagePackage/messageSubfolder/messageName and it materialises
  outputs matching the message fields -- `outputs:data` (bool) for
  std_msgs/Bool. We read it via og.Controller.attribute().get() inside
  the ScriptNode, NOT via a graph CONNECT (the dynamic output is
  placeholder-typed at connect time).

* The /state publisher is NOT a `ROS2Publisher` OG node. An earlier
  design wired `DoorScript.outputs:execOut -> PubState.inputs:execIn`
  and tried to gate the rate by returning False from compute(). In
  Isaac Sim 5.1 that return value does not suppress downstream exec,
  so PubState fired every physics tick (~50 Hz) regardless. We now
  publish directly from the ScriptNode via `rclpy.create_node` +
  `create_publisher(String, ...)`. Gating lives in pure Python, which
  gives reliable "on change + 1 Hz heartbeat" behaviour.

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

State (published on /door/<id>/state, std_msgs/String):
    "closed"   -- last command was false and both leaves at closed_y
    "opening"  -- last command was true  but not yet at open_y
    "open"     -- last command was true  and both leaves at open_y
    "closing"  -- last command was false but not yet at closed_y

Publish policy: on state change (immediate) + 1 Hz heartbeat while
stable. Subscribers can use "no message for >2 s" as a liveness
timeout.

Requires Isaac Sim runtime; do not import before `SimulationApp` is up.
"""
from syncai_omniverse.ros2._ns import apply_namespace as _apply_namespace


# Runs inside omni.graph.scriptnode.ScriptNode. Module-level globals
# persist across compute calls.
_DOOR_SCRIPT = """\
import time as _time

_state = {
    "sub_attr": None,
    "ros_node": None,
    "ros_pub": None,
    "ros_msg": None,
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
    "last_state": None,
    "last_pub_time": 0.0,
}

# Max slide speed (m / physics-tick). At 60 Hz physics, 0.03 m/tick =
# 1.8 m/s, so a 1.45 m opening completes in ~0.8 s.
_STEP = 0.03
# "At target" tolerance for the state machine. Must be < _STEP so a leaf
# can't simultaneously satisfy "still stepping" and "at target".
_AT_TARGET_EPS = 1e-4
# Heartbeat period for /state publishing while the string is unchanged.
# State transitions bypass this and publish immediately.
_HEARTBEAT_PERIOD_S = 1.0


def setup(db):
    import omni.graph.core as og
    import omni.usd
    try:
        _state["sub_attr"] = og.Controller.attribute(str(db.inputs.subAttrPath))
    except Exception:
        _state["sub_attr"] = None

    # Direct rclpy publisher for /state. The isaacsim.ros2.bridge
    # extension has already initialised rclpy by the time this graph
    # runs, so we just create our own Node + Publisher and call
    # publish() from compute(). This gives us reliable Python-side
    # rate gating instead of relying on OG exec suppression.
    try:
        import rclpy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        node_name = str(db.inputs.rosNodeName)
        state_topic = str(db.inputs.stateTopic)
        _state["ros_node"] = rclpy.create_node(node_name)
        _state["ros_pub"] = _state["ros_node"].create_publisher(
            String, state_topic, 10
        )
        _state["ros_msg"] = String()
    except Exception as exc:
        print(f"[door] rclpy publisher setup failed: {exc}")
        _state["ros_node"] = None
        _state["ros_pub"] = None
        _state["ros_msg"] = None

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

    at_closed = (
        abs(_state["left_cur_y"] - _state["left_closed_y"]) < _AT_TARGET_EPS
        and abs(_state["right_cur_y"] - _state["right_closed_y"]) < _AT_TARGET_EPS
    )
    at_open = (
        abs(_state["left_cur_y"] - _state["left_open_y"]) < _AT_TARGET_EPS
        and abs(_state["right_cur_y"] - _state["right_open_y"]) < _AT_TARGET_EPS
    )
    if is_open:
        state_str = "open" if at_open else "opening"
    else:
        state_str = "closed" if at_closed else "closing"

    pub = _state["ros_pub"]
    msg = _state["ros_msg"]
    now = _time.time()
    should_publish = (
        state_str != _state["last_state"]
        or (now - _state["last_pub_time"]) >= _HEARTBEAT_PERIOD_S
    )
    if should_publish and pub is not None and msg is not None:
        msg.data = state_str
        try:
            pub.publish(msg)
            _state["last_state"] = state_str
            _state["last_pub_time"] = now
        except Exception as exc:
            print(f"[door] publish failed: {exc}")
    return True
"""


def attach_door_controller(
    stage,
    door_path: str,
    cmd_topic: str,
    state_topic: str,
    open_target: float = 0.95,
    namespace: str = "",
    graph_path: str | None = None,
    debug: bool = False,
) -> str:
    """Build (or overwrite) the door command + state graph for one door."""
    import omni.graph.core as og

    door_prim = stage.GetPrimAtPath(door_path)
    if not door_prim.IsValid():
        raise RuntimeError(f"Door prim not found: {door_path}")

    left_path = f"{door_path}/leaf_left"
    right_path = f"{door_path}/leaf_right"
    for p in (left_path, right_path):
        if not stage.GetPrimAtPath(p).IsValid():
            raise RuntimeError(f"Door leaf prim not found: {p}")

    cmd_topic = _apply_namespace(namespace, cmd_topic)
    state_topic = _apply_namespace(namespace, state_topic)
    graph_path = graph_path or f"/DoorGraph_{door_prim.GetName()}"
    sub_attr_path = f"{graph_path}/SubBool.outputs:data"
    # Per-door rclpy node name -- must be unique across doors so multiple
    # DoorGraph_* in the same process don't collide on the same Node.
    ros_node_name = f"door_{door_prim.GetName()}_state_pub".replace("/", "_")

    create_nodes = [
        ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubBool", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ("DoorScript", "omni.graph.scriptnode.ScriptNode"),
    ]
    create_attributes = [
        ("DoorScript.inputs:subAttrPath", "string"),
        ("DoorScript.inputs:stateTopic", "string"),
        ("DoorScript.inputs:rosNodeName", "string"),
        ("DoorScript.inputs:leftPrimPath", "string"),
        ("DoorScript.inputs:rightPrimPath", "string"),
    ]
    connect = [
        ("OnPhysics.outputs:step", "SubBool.inputs:execIn"),
        ("OnPhysics.outputs:step", "DoorScript.inputs:execIn"),
    ]
    set_values = [
        ("SubBool.inputs:topicName", cmd_topic),
        ("SubBool.inputs:messagePackage", "std_msgs"),
        ("SubBool.inputs:messageSubfolder", "msg"),
        ("SubBool.inputs:messageName", "Bool"),
        ("DoorScript.inputs:script", _DOOR_SCRIPT),
        ("DoorScript.inputs:subAttrPath", sub_attr_path),
        ("DoorScript.inputs:stateTopic", state_topic),
        ("DoorScript.inputs:rosNodeName", ros_node_name),
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

    print(f"[door] graph={graph_path}  cmd={cmd_topic}  state={state_topic}  "
          f"open_target={open_target}")
    print(f"[door]   leaves: {left_path}, {right_path}")

    if debug:
        print(f"[door][debug] nodes:")
        for name, type_ in create_nodes:
            print(f"[door][debug]   {graph_path}/{name}  <{type_}>")

    return graph_path
