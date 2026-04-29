"""Attach an OmniGraph that subscribes to std_msgs/Float32 on
/conveyor/<id>/speed_cmd, drives the belt's surface velocity via an
isaacsim.asset.gen.conveyor.IsaacConveyor node, and publishes
std_msgs/String state on /conveyor/<id>/status.

Graph shape (on-demand pipeline, fires once per physics substep):
    OnPhysicsStep ──┬─> ROS2Subscriber(std_msgs/Float32)  (dynamic outputs:data)
                    ├─> ScriptNode -- reads sub.data, optionally checks
                    │                 box position vs limit threshold,
                    │                 writes IsaacConveyor.inputs:velocity,
                    │                 publishes /status
                    └─> IsaacConveyor -- consumes velocity, drives surface

Mirrors src/syncai_omniverse/ros2/door_controller.py architecture line for
line: module-level _state dict, ScriptNode setup/compute, direct rclpy
publisher (no ROS2Publisher OG node), on-demand graph pipeline so
OnPhysicsStep actually fires (memory: feedback_ondemand_graph), generic
ROS2Subscriber + messageName="Float32" with attribute readback via
og.Controller.attribute() (memory: feedback_ros2_generic_subscriber --
Isaac Sim 5.1 has no SubscribeFloat32, and dynamic-attr CONNECT is
placeholder-typed at connect time).

Limit switch is a soft Python gate inside the ScriptNode rather than a
physical sensor: read the box's world position each tick, compare to
threshold, override velocity to 0 if past. Auto-recovers when the box
moves back upstream. No extra prims, single source of truth.

Speed semantics:
    Float32 commanded value is the belt's surface speed in m/s. 0 = stop;
    >0 = forward (along belt-local `direction`); <0 = reverse if the
    Isaac asset's belt animation supports it (the IsaacConveyor node
    accepts negative velocity and applies it directly).

State (published on /conveyor/<id>/status, std_msgs/String):
    "stopped"          -- effective speed is 0 and limit not triggered
    "running"          -- belt is moving (|effective speed| > 1e-3)
    "limit_triggered"  -- limit switch active; commanded speed gated to 0

Publish policy: on state change (immediate) + 1 Hz heartbeat while stable.

Requires Isaac Sim runtime; do not import before SimulationApp is up.
"""
from syncai_omniverse.ros2._ns import apply_namespace as _apply_namespace


# Runs inside omni.graph.scriptnode.ScriptNode. Module-level globals persist
# across compute calls (and across physics ticks for this graph instance).
_CONVEYOR_SCRIPT = """\
import time as _time

_state = {
    "sub_attr": None,
    "vel_attr": None,
    "ros_node": None,
    "ros_pub": None,
    "ros_msg": None,
    "stage": None,
    "box_prim_path": "",
    "limit_axis_idx": -1,        # -1 = limit switch disabled
    "limit_threshold": 0.0,
    "limit_cmp_ge": True,        # True => gate when pos >= threshold (le otherwise)
    "last_state": None,
    "last_pub_time": 0.0,
    "rollers_path": "",
    "rollers_sv_attr": None,
    "direction": (1.0, 0.0, 0.0),
}

# Heartbeat period for /status publishing while the string is unchanged.
# State transitions bypass this and publish immediately.
_HEARTBEAT_PERIOD_S = 1.0
# Below this magnitude the belt is considered "stopped".
_STOPPED_EPS = 1e-3


def setup(db):
    import omni.graph.core as og
    import omni.usd

    try:
        _state["sub_attr"] = og.Controller.attribute(str(db.inputs.speedCmdAttrPath))
    except Exception:
        _state["sub_attr"] = None

    try:
        _state["vel_attr"] = og.Controller.attribute(str(db.inputs.velocityAttrPath))
    except Exception:
        _state["vel_attr"] = None

    # Direct rclpy publisher for /status. The isaacsim.ros2.bridge extension
    # has already initialised rclpy by the time this graph runs, so we just
    # create our own Node + Publisher and call publish() from compute().
    try:
        import rclpy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        node_name = str(db.inputs.rosNodeName)
        status_topic = str(db.inputs.statusTopic)
        _state["ros_node"] = rclpy.create_node(node_name)
        _state["ros_pub"] = _state["ros_node"].create_publisher(
            String, status_topic, 10
        )
        _state["ros_msg"] = String()
    except Exception as exc:
        print(f"[conveyor] rclpy publisher setup failed: {exc}")
        _state["ros_node"] = None
        _state["ros_pub"] = None
        _state["ros_msg"] = None

    _state["stage"] = omni.usd.get_context().get_stage()
    _state["box_prim_path"] = str(db.inputs.boxPrim)
    axis = str(db.inputs.limitAxis).lower()
    _state["limit_axis_idx"] = {"x": 0, "y": 1, "z": 2}.get(axis, -1)
    _state["limit_threshold"] = float(db.inputs.limitThreshold)
    try:
        _state["limit_cmp_ge"] = str(db.inputs.limitComparator).lower() != "le"
    except Exception:
        _state["limit_cmp_ge"] = True
    try:
        _state["rollers_path"] = str(db.inputs.rollersPrim)
    except Exception:
        _state["rollers_path"] = ""
    try:
        _state["direction"] = (
            float(db.inputs.dirX), float(db.inputs.dirY), float(db.inputs.dirZ)
        )
    except Exception:
        _state["direction"] = (1.0, 0.0, 0.0)
    # Cache the rollers' surfaceVelocity attribute. The schema only exposes
    # this attribute after PhysxSurfaceVelocityAPI has been Apply()d in
    # run_sim.py; if we can't grab it now we'll fall back to per-tick lookup.
    if _state["rollers_path"]:
        try:
            r = _state["stage"].GetPrimAtPath(_state["rollers_path"])
            if r and r.IsValid():
                a = r.GetAttribute("physxSurfaceVelocity:surfaceVelocity")
                if a and a.IsValid():
                    _state["rollers_sv_attr"] = a
        except Exception:
            _state["rollers_sv_attr"] = None


def compute(db):
    import omni.graph.core as og

    sub = _state["sub_attr"]
    if sub is None:
        try:
            _state["sub_attr"] = og.Controller.attribute(str(db.inputs.speedCmdAttrPath))
            sub = _state["sub_attr"]
        except Exception:
            sub = None

    cmd = 0.0
    if sub is not None:
        try:
            v = sub.get()
            if v is not None:
                cmd = float(v)
        except Exception:
            cmd = 0.0

    # Limit-switch gate: pin effective velocity to 0 while the box is past
    # the configured world-axis threshold. Skipped entirely if no box prim
    # was supplied (limit_axis_idx == -1) or the prim has gone missing.
    gated = False
    axis_idx = _state["limit_axis_idx"]
    box_path = _state["box_prim_path"]
    if axis_idx >= 0 and box_path:
        stage = _state["stage"]
        box_prim = stage.GetPrimAtPath(box_path) if stage is not None else None
        if box_prim is not None and box_prim.IsValid():
            try:
                # World position via the rigid body's xformOp:translate. For a
                # DynamicCuboid the physics integrator updates this attribute
                # every step, so it reflects the current sim pose.
                t_attr = box_prim.GetAttribute("xformOp:translate")
                pos = t_attr.Get() if t_attr else None
                if pos is not None:
                    p = pos[axis_idx]
                    thr = _state["limit_threshold"]
                    if (_state["limit_cmp_ge"] and p >= thr) or \
                       (not _state["limit_cmp_ge"] and p <= thr):
                        gated = True
            except Exception:
                gated = False

    effective = 0.0 if gated else cmd

    vel_attr = _state["vel_attr"]
    if vel_attr is None:
        try:
            _state["vel_attr"] = og.Controller.attribute(str(db.inputs.velocityAttrPath))
            vel_attr = _state["vel_attr"]
        except Exception:
            vel_attr = None
    if vel_attr is not None:
        try:
            vel_attr.set(float(effective))
        except Exception:
            pass

    # Mirror the belt's surface velocity onto the Rollers body. The cargo
    # box can settle on the rollers (the belt mesh is a thin curved strip
    # and is not watertight against tunneling), and PhysX only applies a
    # surface push if SurfaceVelocityAPI is on the body the cargo actually
    # contacts.
    rsv = _state["rollers_sv_attr"]
    if rsv is None and _state["rollers_path"]:
        try:
            r = _state["stage"].GetPrimAtPath(_state["rollers_path"])
            if r and r.IsValid():
                rsv = r.GetAttribute("physxSurfaceVelocity:surfaceVelocity")
                if rsv and rsv.IsValid():
                    _state["rollers_sv_attr"] = rsv
                else:
                    rsv = None
        except Exception:
            rsv = None
    if rsv is not None:
        try:
            from pxr import Gf
            d = _state["direction"]
            rsv.Set(Gf.Vec3f(d[0] * effective, d[1] * effective, d[2] * effective))
        except Exception:
            pass

    if gated:
        state_str = "limit_triggered"
    elif abs(effective) > _STOPPED_EPS:
        state_str = "running"
    else:
        state_str = "stopped"

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
            print(f"[conveyor] publish failed: {exc}")
    return True
"""


def attach_conveyor_controller(
    stage,
    conveyor_path: str,
    speed_topic: str,
    status_topic: str,
    belt_surface_prim: str,
    direction: tuple = (1.0, 0.0, 0.0),
    box_prim: str | None = None,
    rollers_prim: str | None = None,
    limit_axis: str | None = None,
    limit_threshold: float | None = None,
    limit_comparator: str | None = None,
    namespace: str = "",
    graph_path: str | None = None,
    debug: bool = False,
) -> str:
    """Build the per-conveyor OmniGraph (one graph per conveyor prim).

    Args:
        stage: The live USD stage.
        conveyor_path: Wrapper prim path, e.g. /World/Conveyors/conveyor_01.
        speed_topic: ROS2 std_msgs/Float32 topic name (m/s commanded).
        status_topic: ROS2 std_msgs/String topic name (state machine).
        belt_surface_prim: Mesh prim that IsaacConveyor will drive (it
            applies kinematic surface velocity here).
        direction: Surface-actor-local direction unit vector (default +X).
            PhysxSurfaceVelocityAPI:surfaceVelocity is local-frame, so the
            wrapper's rotation_z_deg is already baked into how this maps
            to world.
        box_prim: Optional cargo-box prim path; passed to the ScriptNode
            for limit-switch position read. None disables the gate.
        rollers_prim: Optional path to the asset's Rollers Xform. Cargo
            on the A08 belt mesh can tunnel through and rest on the
            rollers; the ScriptNode mirrors the surface velocity onto
            this body so contact still pushes the cargo. Pass None to
            skip the mirror.
        limit_axis: 'x'|'y'|'z' world axis name; None disables the gate.
        limit_threshold: Trigger value; combined with limit_comparator.
        limit_comparator: 'ge' (gate when pos >= threshold) or 'le' (gate
            when pos <= threshold). Defaults to 'ge'.
        namespace: ROS2 namespace prefix for both topics.
        graph_path: Override the default /ConveyorGraph_<name> path.
        debug: Print extra diagnostics about the created nodes.

    Returns the graph path.
    """
    import omni.graph.core as og

    conv_prim = stage.GetPrimAtPath(conveyor_path)
    if not conv_prim.IsValid():
        raise RuntimeError(f"Conveyor prim not found: {conveyor_path}")
    if not stage.GetPrimAtPath(belt_surface_prim).IsValid():
        raise RuntimeError(f"Belt surface prim not found: {belt_surface_prim}")

    speed_topic = _apply_namespace(namespace, speed_topic)
    status_topic = _apply_namespace(namespace, status_topic)
    graph_path = graph_path or f"/ConveyorGraph_{conv_prim.GetName()}"
    sub_attr_path = f"{graph_path}/SubFloat32.outputs:data"
    vel_attr_path = f"{graph_path}/IsaacConveyor.inputs:velocity"
    # Per-conveyor rclpy node name -- must be unique across conveyors so
    # multiple ConveyorGraph_* in the same process don't collide.
    ros_node_name = f"conveyor_{conv_prim.GetName()}_status_pub".replace("/", "_")

    # Direction is used by IsaacConveyor's input as a Vec3f-ish list. Some
    # IsaacConveyor versions accept tuples; pass a list to be safe.
    direction_list = [float(direction[0]), float(direction[1]), float(direction[2])]

    create_nodes = [
        ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubFloat32", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ("ConveyorScript", "omni.graph.scriptnode.ScriptNode"),
        ("IsaacConveyor", "isaacsim.asset.gen.conveyor.IsaacConveyor"),
    ]
    create_attributes = [
        ("ConveyorScript.inputs:speedCmdAttrPath", "string"),
        ("ConveyorScript.inputs:velocityAttrPath", "string"),
        ("ConveyorScript.inputs:statusTopic", "string"),
        ("ConveyorScript.inputs:rosNodeName", "string"),
        ("ConveyorScript.inputs:boxPrim", "string"),
        ("ConveyorScript.inputs:rollersPrim", "string"),
        ("ConveyorScript.inputs:dirX", "float"),
        ("ConveyorScript.inputs:dirY", "float"),
        ("ConveyorScript.inputs:dirZ", "float"),
        ("ConveyorScript.inputs:limitAxis", "string"),
        ("ConveyorScript.inputs:limitComparator", "string"),
        ("ConveyorScript.inputs:limitThreshold", "float"),
    ]
    # IsaacConveyor needs both its onStep execution input AND a delta-time
    # value wired or it silently no-ops -- the OGN spec lists inputs:onStep
    # (execution) and inputs:delta (float) as required even though `enabled`
    # and `velocity` are set. Without these, /status reports "running" but
    # PhysxSurfaceVelocity never gets written.
    connect = [
        ("OnPhysics.outputs:step", "SubFloat32.inputs:execIn"),
        ("OnPhysics.outputs:step", "ConveyorScript.inputs:execIn"),
        ("OnPhysics.outputs:step", "IsaacConveyor.inputs:onStep"),
        ("OnPhysics.outputs:deltaSimulationTime", "IsaacConveyor.inputs:delta"),
    ]
    set_values = [
        ("SubFloat32.inputs:topicName", speed_topic),
        ("SubFloat32.inputs:messagePackage", "std_msgs"),
        ("SubFloat32.inputs:messageSubfolder", "msg"),
        ("SubFloat32.inputs:messageName", "Float32"),
        ("ConveyorScript.inputs:script", _CONVEYOR_SCRIPT),
        ("ConveyorScript.inputs:speedCmdAttrPath", sub_attr_path),
        ("ConveyorScript.inputs:velocityAttrPath", vel_attr_path),
        ("ConveyorScript.inputs:statusTopic", status_topic),
        ("ConveyorScript.inputs:rosNodeName", ros_node_name),
        ("ConveyorScript.inputs:boxPrim", box_prim or ""),
        ("ConveyorScript.inputs:rollersPrim", rollers_prim or ""),
        ("ConveyorScript.inputs:dirX", direction_list[0]),
        ("ConveyorScript.inputs:dirY", direction_list[1]),
        ("ConveyorScript.inputs:dirZ", direction_list[2]),
        ("ConveyorScript.inputs:limitAxis", limit_axis or ""),
        ("ConveyorScript.inputs:limitComparator",
         (limit_comparator or "ge").lower()),
        ("ConveyorScript.inputs:limitThreshold",
         float(limit_threshold) if limit_threshold is not None else 0.0),
        # IsaacConveyor reads inputs:conveyorPrim by relationship; also fall
        # back to the string-typed targetPrim shape used by some 5.x revs by
        # writing the path. The node implementation accepts the empty
        # rel + path-string combo and resolves it at evaluate.
        ("IsaacConveyor.inputs:velocity", 0.0),
        ("IsaacConveyor.inputs:direction", direction_list),
        ("IsaacConveyor.inputs:enabled", True),
    ]
    set_relationships = [
        ("IsaacConveyor.inputs:conveyorPrim", [belt_surface_prim]),
    ]

    keys = og.Controller.Keys
    edits = {
        keys.CREATE_NODES: create_nodes,
        keys.CREATE_ATTRIBUTES: create_attributes,
        keys.CONNECT: connect,
        keys.SET_VALUES: set_values,
    }
    # Older builds of og.Controller call this key SET_RELATIONSHIPS; newer
    # ones merge it under SET_VALUES. Try the explicit key first; fall back.
    if hasattr(keys, "SET_RELATIONSHIPS"):
        edits[keys.SET_RELATIONSHIPS] = set_relationships
    else:
        edits[keys.SET_VALUES] = list(edits[keys.SET_VALUES]) + set_relationships

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        edits,
    )

    print(f"[conveyor] graph={graph_path}  cmd={speed_topic}  "
          f"status={status_topic}")
    print(f"[conveyor]   belt={belt_surface_prim}  direction={direction_list}")
    if box_prim:
        print(f"[conveyor]   box={box_prim}")
    if limit_axis is not None and limit_threshold is not None:
        print(f"[conveyor]   limit: axis={limit_axis} threshold={limit_threshold}")

    if debug:
        print("[conveyor][debug] nodes:")
        for name, type_ in create_nodes:
            print(f"[conveyor][debug]   {graph_path}/{name}  <{type_}>")

    return graph_path
