"""Publish a fleet-wide JSON snapshot of pending pickups on /cargo/pending.

Each conveyor already publishes its own /conveyor/<id>/status string, but a
BT-based central controller would otherwise have to subscribe to N topics
and join them client-side. This module aggregates them into a single
heartbeat topic that any task planner can subscribe to.

Graph shape (one graph per simulation, NOT per conveyor):
    OnPlaybackTick ──> AggregatorScript
                          │
                          ├── rclpy node "cargo_pending_pub"
                          │     ├── Publisher  /cargo/pending  (std_msgs/String)
                          │     └── Subscriber x N (one per conveyor)
                          │           callback: latest_status[conv_id] = msg.data
                          │
                          └── compute() per tick:
                              1. rclpy.spin_once(timeout_sec=0)
                              2. parse latest status -> phase / box_id
                              3. fixed 1 Hz cadence -> publish JSON

Schema (std_msgs/String, JSON-encoded):
    {"stamp_sec": 1234.567,
     "pending": [{"conveyor_id": "conveyor01",
                  "phase": "handoff",      # belt | handoff
                  "box_id": "box01"}]}

Filter: only conveyors whose phase is `belt` or `handoff` are included --
`carried` (already on a robot) and `dropped` (no box on belt) are not
"pending pickup". An entry vanishes from `pending` once it transitions to
carried, and reappears when a new box is spawned.

Memory hints honoured:
    - Module-level globals in ScriptNodes are shared across instances; this
      file uses `_pending_state` to avoid clashing with conveyor_controller's
      `_state` (memory: feedback_scriptnode_globals_shared).
    - rclpy spin_once with timeout_sec=0 inside compute() is the cheapest
      way to drain subscriber callbacks without blocking the graph tick.

Requires Isaac Sim runtime; do not import before SimulationApp is up.
"""
from syncai_omniverse.ros2._ns import apply_namespace as _apply_namespace


_PENDING_SCRIPT = """\
import time as _time
import json as _json

# Prefixed to avoid clobbering other ScriptNodes' module-level state in
# the shared OgnScriptNode globals namespace.
_pending_state = {
    "ros_node": None,
    "ros_pub": None,
    "ros_msg": None,
    "subs": [],                     # keep refs so rclpy doesn't GC them
    "conveyors": [],                # [(conv_id, status_topic), ...]
    "latest_status": {},            # {conv_id: "handoff:box01" | "carried:..." | ...}
    "latest_box_id": {},            # {conv_id: "box01"} -- sticky across status strings
    "latest_phase": {},             # {conv_id: "belt" | "handoff" | "carried" | "dropped"}
    "last_pub_text": "",
    "last_pub_time": 0.0,
}

_PENDING_PERIOD_S = 1.0


def _pending_make_callback(conv_id):
    # Closure captures conv_id so every conveyor's sub writes to the right
    # slot in latest_status. Defined outside setup() so each create_subscription
    # call gets its own bound function rather than the loop variable trap.
    def _cb(msg):
        try:
            _pending_state["latest_status"][conv_id] = msg.data
        except Exception:
            pass
    return _cb


def _pending_parse_status(status_text, prev_box_id):
    \"\"\"Parse a /conveyor/<id>/status string into (phase, box_id).
    Returns (None, prev_box_id) for transient *_rejected strings -- caller
    should keep the previous phase.\"\"\"
    s = (status_text or "").strip()
    if not s:
        return ("belt", prev_box_id)
    if s.startswith("pickup_rejected:") or s.startswith("drop_rejected:"):
        return (None, prev_box_id)
    if s in ("running", "stopped", "limit_triggered"):
        # No pickup_target on this conveyor, OR pickup enabled but limit
        # not yet fired. Treat as "still on belt".
        return ("belt", prev_box_id)
    if s == "handoff":
        return ("handoff", prev_box_id)
    if s.startswith("handoff:"):
        return ("handoff", s.split(":", 1)[1] or prev_box_id)
    if s.startswith("carried:"):
        # carried:box01@SyncRobot01 OR carried:SyncRobot01 OR carried
        rest = s.split(":", 1)[1]
        if "@" in rest:
            bid = rest.split("@", 1)[0]
            return ("carried", bid or prev_box_id)
        return ("carried", prev_box_id)
    if s.startswith("dropped:"):
        rest = s.split(":", 1)[1]
        if "@" in rest:
            bid = rest.split("@", 1)[0]
            return ("dropped", bid or prev_box_id)
        return ("dropped", prev_box_id)
    return ("belt", prev_box_id)


def setup(db):
    # Parse the CSV "conv_id|status_topic,conv_id|status_topic,..." into
    # a list of (conv_id, status_topic) tuples. The CSV form keeps the
    # ScriptNode input schema flat (single string) the same way the
    # conveyor controller encodes drop_zones.
    try:
        conv_csv = str(db.inputs.conveyorsCsv)
    except Exception:
        conv_csv = ""
    conveyors = []
    for tok in conv_csv.split(","):
        tok = tok.strip()
        if not tok:
            continue
        parts = tok.split("|")
        if len(parts) != 2:
            print(f"[pending] bad conveyor token {tok!r}")
            continue
        conv_id, status_topic = parts[0].strip(), parts[1].strip()
        if conv_id and status_topic:
            conveyors.append((conv_id, status_topic))
    _pending_state["conveyors"] = conveyors

    try:
        publish_topic = str(db.inputs.publishTopic)
    except Exception:
        publish_topic = "/cargo/pending"
    try:
        node_name = str(db.inputs.rosNodeName) or "cargo_pending_pub"
    except Exception:
        node_name = "cargo_pending_pub"

    try:
        import rclpy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node(node_name)
        _pending_state["ros_node"] = node
        _pending_state["ros_pub"] = node.create_publisher(
            String, publish_topic, 10
        )
        _pending_state["ros_msg"] = String()
        # Subscribe to every conveyor's status topic. Keep handles in a list
        # so rclpy doesn't garbage-collect them.
        for conv_id, status_topic in conveyors:
            sub = node.create_subscription(
                String, status_topic, _pending_make_callback(conv_id), 10
            )
            _pending_state["subs"].append(sub)
        print(
            f"[pending] setup ok  pub={publish_topic} subs={len(conveyors)}"
        )
    except Exception as exc:
        print(f"[pending] rclpy setup failed: {exc}")
        _pending_state["ros_node"] = None
        _pending_state["ros_pub"] = None
        _pending_state["ros_msg"] = None


def compute(db):
    node = _pending_state["ros_node"]
    pub = _pending_state["ros_pub"]
    msg = _pending_state["ros_msg"]
    if node is None or pub is None:
        return True

    # Drain incoming /conveyor/<id>/status messages so latest_status reflects
    # whatever each conveyor most recently published. spin_once with timeout=0
    # processes at most one ready callback then returns; loop enough times to
    # cover every subscriber plus a small headroom.
    try:
        import rclpy
        n_drains = max(1, len(_pending_state["conveyors"])) + 2
        for _ in range(n_drains):
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as exc:
        print(f"[pending] spin_once failed: {exc}")

    # Re-derive phase + box_id for every conveyor, then build the JSON
    # snapshot containing only belt/handoff entries.
    pending_list = []
    for conv_id, _topic in _pending_state["conveyors"]:
        status_text = _pending_state["latest_status"].get(conv_id, "")
        prev_box_id = _pending_state["latest_box_id"].get(conv_id)
        prev_phase = _pending_state["latest_phase"].get(conv_id, "belt")
        phase, box_id = _pending_parse_status(status_text, prev_box_id)
        if phase is None:
            # rejected sentinel; keep last known phase.
            phase = prev_phase
        _pending_state["latest_phase"][conv_id] = phase
        if box_id is not None:
            _pending_state["latest_box_id"][conv_id] = box_id
        if phase in ("belt", "handoff"):
            pending_list.append({
                "conveyor_id": conv_id,
                "phase": phase,
                "box_id": box_id,
            })

    now = _time.time()
    # Fixed 1 Hz cadence -- no on-change short-circuit. Consumers (BT
    # central controller) get a steady heartbeat regardless of whether
    # any conveyor's state actually moved this second.
    if (now - _pending_state["last_pub_time"]) < _PENDING_PERIOD_S:
        return True
    payload = _json.dumps(
        {"stamp_sec": round(now, 3), "pending": pending_list},
        sort_keys=True,
    )
    msg.data = payload
    try:
        pub.publish(msg)
        _pending_state["last_pub_text"] = payload
        _pending_state["last_pub_time"] = now
    except Exception as exc:
        print(f"[pending] publish failed: {exc}")
    return True
"""


def attach_pending_publisher(
    stage,
    conveyors_info: list,
    topic: str = "/cargo/pending",
    namespace: str = "",
    graph_path: str = "/CargoPendingGraph",
    ros_node_name: str = "cargo_pending_pub",
) -> str:
    """Build the singleton OmniGraph that aggregates per-conveyor status into
    a single /cargo/pending JSON heartbeat.

    Args:
        stage: live USD stage (currently unused; kept for parity with other
            attach_* helpers in this package).
        conveyors_info: list of (conveyor_id, status_topic) tuples. Pass only
            the conveyors whose status should appear in /cargo/pending.
        topic: output topic (default /cargo/pending).
        namespace: optional ROS2 namespace prefix; usually empty (the
            published topic is fleet-shared infrastructure, not robot-bound).
        graph_path: USD prim path for the OmniGraph itself.
        ros_node_name: rclpy node name; must be unique in the process.

    Returns the graph path.
    """
    import omni.graph.core as og

    if not conveyors_info:
        print("[pending] no conveyors_info; skipping graph creation")
        return ""

    publish_topic = _apply_namespace(namespace, topic)
    # Pack (id, status_topic) pairs into the flat CSV the ScriptNode expects.
    # Apply the same namespace to status topics so subscribers see the same
    # names the conveyors publish on.
    conv_csv = ",".join(
        f"{cid}|{_apply_namespace(namespace, st)}"
        for cid, st in conveyors_info
    )

    create_nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("AggregatorScript", "omni.graph.scriptnode.ScriptNode"),
    ]
    create_attributes = [
        ("AggregatorScript.inputs:conveyorsCsv", "string"),
        ("AggregatorScript.inputs:publishTopic", "string"),
        ("AggregatorScript.inputs:rosNodeName", "string"),
    ]
    connect = [
        ("OnTick.outputs:tick", "AggregatorScript.inputs:execIn"),
    ]
    set_values = [
        ("AggregatorScript.inputs:script", _PENDING_SCRIPT),
        ("AggregatorScript.inputs:conveyorsCsv", conv_csv),
        ("AggregatorScript.inputs:publishTopic", publish_topic),
        ("AggregatorScript.inputs:rosNodeName", ros_node_name),
    ]

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CREATE_ATTRIBUTES: create_attributes,
            keys.CONNECT: connect,
            keys.SET_VALUES: set_values,
        },
    )

    print(
        f"[pending] graph={graph_path}  topic={publish_topic}  "
        f"conveyors={len(conveyors_info)}"
    )
    for cid, st in conveyors_info:
        print(f"[pending]   sub  {cid}  <-  {_apply_namespace(namespace, st)}")
    return graph_path
