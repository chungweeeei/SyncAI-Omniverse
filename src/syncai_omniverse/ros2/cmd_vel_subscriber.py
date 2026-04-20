"""Attach an OmniGraph that subscribes to /cmd_vel (geometry_msgs/Twist) and
drives the SlotCar's two wheel joints through a DifferentialController.

Graph shape (on-demand pipeline, fires once per physics substep):
    OnPhysicsStep ─┬─> ROS2SubscribeTwist ─> BreakVector3(linear).x ──┐
                   │                      └─ BreakVector3(angular).z ─┤
                   │                                                   ├─> DifferentialController
                   │                                                         │  (velocityCommand, data)
                   │                                                         ▼
                   └──────────────────────────────────────────> IsaacArticulationController

Why on-demand + OnPhysicsStep:
  * `OnPhysicsStep` only fires in an on-demand graph (otherwise Kit logs:
    "Physics OnSimulationStep node detected in a non on-demand Graph").
  * Ticking the drive chain on the physics clock (not render/app rate)
    applies drive targets exactly once per PhysX substep, eliminating
    the skipped/doubled writes that caused visible wheel jitter.
  * SubTwist poll rate is now the physics rate (typically 60 Hz) —
    higher and more stable than OnPlaybackTick under VSync.

Requires Isaac Sim runtime; do not import before `SimulationApp` is up.
"""
from pxr import Sdf


def attach_cmd_vel_subscriber(
    stage,
    robot_path: str = "/World/SlotCar",
    left_joint: str = "drivewhl_l_joint",
    right_joint: str = "drivewhl_r_joint",
    wheel_radius: float = 0.10,
    wheel_distance: float = 0.36,
    # `DifferentialController` clamps with min/max (one-sided), so a positive
    # cap silently truncates negative inputs to 0 — the robot would only move
    # forward. Default to 0.0 ("not set" per the OGN schema) so negative
    # linear and either-sign angular velocities pass through.
    max_linear_speed: float = 0.0,
    max_angular_speed: float = 0.0,
    max_wheel_speed: float = 0.0,
    topic: str = "/cmd_vel",
    graph_path: str = "/CmdVelActionGraph",
    debug: bool = False,
) -> str:
    """
        Build (or overwrite) the cmd_vel → wheel-command graph.

        When `debug=True`, prints a full snapshot after the graph is authored:
        node list, every set input value, and the current SubTwist outputs.
        Pair with `print_cmd_vel_snapshot(graph_path)` at runtime to watch
        live values while publishing /cmd_vel.
    """
    import omni.graph.core as og

    # -- Pre-flight validation --
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_path}")
    for jname in (left_joint, right_joint):
        if not stage.GetPrimAtPath(f"{robot_path}/joints/{jname}").IsValid():
            raise RuntimeError(f"Joint not found: {robot_path}/joints/{jname}")

    create_nodes = [
        ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLinear", "omni.graph.nodes.BreakVector3"),
        ("BreakAngular", "omni.graph.nodes.BreakVector3"),
        ("DiffCtrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("ArtCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    connect = [
        # Everything fires off the physics clock. SubTwist polls the rclpy
        # queue once per substep; DiffCtrl recomputes when a new Twist lands;
        # ArtCtrl always runs so the latest velocityCommand (data pull) is
        # applied every substep, even when no new ROS message arrived.
        ("OnPhysics.outputs:step", "SubTwist.inputs:execIn"),
        ("OnPhysics.outputs:step", "ArtCtrl.inputs:execIn"),
        ("SubTwist.outputs:execOut", "DiffCtrl.inputs:execIn"),
        ("SubTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
        ("SubTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
        ("BreakLinear.outputs:x", "DiffCtrl.inputs:linearVelocity"),
        ("BreakAngular.outputs:z", "DiffCtrl.inputs:angularVelocity"),
        ("DiffCtrl.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
    ]
    set_values = [
        ("SubTwist.inputs:topicName", topic),
        ("DiffCtrl.inputs:wheelRadius", wheel_radius),
        ("DiffCtrl.inputs:wheelDistance", wheel_distance),
        ("DiffCtrl.inputs:maxLinearSpeed", max_linear_speed),
        ("DiffCtrl.inputs:maxAngularSpeed", max_angular_speed),
        ("DiffCtrl.inputs:maxWheelSpeed", max_wheel_speed),
        ("ArtCtrl.inputs:targetPrim", [Sdf.Path(robot_path)]),
        ("ArtCtrl.inputs:jointNames", [left_joint, right_joint]),
    ]

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            # OnPhysicsStep only triggers when its parent graph is on-demand;
            # otherwise Kit prints "Physics OnSimulationStep node detected in
            # a non on-demand Graph" and the node silently never fires.
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    # -- Post-build summary --
    print(f"[cmd_vel] graph={graph_path}  topic={topic}")
    print(f"[cmd_vel]   target={robot_path}  joints=[{left_joint}, {right_joint}]")
    print(f"[cmd_vel]   wheelRadius={wheel_radius} m  wheelDistance={wheel_distance} m")
    print(f"[cmd_vel]   maxLinearSpeed={max_linear_speed} (0=unlimited)  "
          f"maxAngularSpeed={max_angular_speed} (0=unlimited)  "
          f"maxWheelSpeed={max_wheel_speed} (0=unlimited)")

    if debug:
        print(f"[cmd_vel][debug] nodes:")
        for name, type_ in create_nodes:
            print(f"[cmd_vel][debug]   {graph_path}/{name}  <{type_}>")
        print(f"[cmd_vel][debug] readback of key attributes:")
        _dump_attrs(graph_path, [
            "SubTwist.inputs:topicName",
            "SubTwist.outputs:linearVelocity",
            "SubTwist.outputs:angularVelocity",
            "DiffCtrl.inputs:wheelRadius",
            "DiffCtrl.inputs:wheelDistance",
            "DiffCtrl.outputs:velocityCommand",
            "ArtCtrl.inputs:jointNames",
        ])

    return graph_path


def print_cmd_vel_snapshot(graph_path: str = "/CmdVelActionGraph") -> None:
    """Print current live values from the cmd_vel graph. Call at runtime to
    confirm the ROS2 subscriber is receiving data and forwarding to the wheels.
    """
    print(f"[cmd_vel] snapshot of {graph_path}:")
    _dump_attrs(graph_path, [
        "SubTwist.inputs:topicName",
        "SubTwist.outputs:linearVelocity",
        "SubTwist.outputs:angularVelocity",
        "BreakLinear.outputs:x",
        "BreakAngular.outputs:z",
        "DiffCtrl.outputs:velocityCommand",
    ])


def _dump_attrs(graph_path: str, attrs: list[str]) -> None:
    import omni.graph.core as og
    for rel in attrs:
        full = f"{graph_path}/{rel}"
        try:
            val = og.Controller.attribute(full).get()
        except Exception as exc:
            val = f"<unreadable: {exc}>"
        print(f"[cmd_vel][debug]   {rel} = {val}")
