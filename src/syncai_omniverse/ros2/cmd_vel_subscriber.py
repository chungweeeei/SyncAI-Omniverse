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
from pxr import Sdf, Usd, UsdPhysics

from syncai_omniverse.ros2._ns import (
    apply_namespace as _apply_namespace,
    find_articulation_root as _find_articulation_root,
)


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
    # Acceleration limits ramp DiffCtrl's velocityCommand so wheel drive
    # doesn't see an instantaneous step on cmd_vel change. Units m/s^2,
    # rad/s^2. 0.0 = unlimited. DiffCtrl reads `dt` wired to
    # `OnPhysicsStep.outputs:deltaSimulationTime` so ramping advances by
    # the real physics substep.
    #
    # Tuned to be slightly LOOSER than nav2's velocity_smoother defaults
    # (max_accel=[2.5, 0, 1.5], max_decel=[-2.5, 0, -1.5]) so our rate
    # limiter never bottlenecks nav2's close-loop near-goal corrections
    # (e.g. `RegulatedPurePursuit` + `use_rotate_to_heading=True` sends
    # fast-tapering angular vel to finish yaw within yaw_goal_tolerance).
    # Physical smoothing still happens via wheel drive (maxForce=8 N·m)
    # and base_link linear/angular damping.
    max_linear_accel: float = 2.5,
    max_linear_decel: float = 2.5,
    max_angular_accel: float = 2.0,
    # Defaults match `slot_car.py::_revolute_joint`; we re-apply so stages
    # authored with stale values still get the intended gains. Lowered from
    # the original (2000/15) to smooth the wheel drive response — see the
    # block comment inside `_revolute_joint` for the force-budget math.
    wheel_drive_damping: float = 200.0,
    wheel_drive_max_force: float = 8.0,
    topic: str = "/cmd_vel",
    namespace: str = "",
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
    # IsaacArticulationController resolves drive joints by name from the
    # articulation root, so we don't need the joint's absolute prim path --
    # just confirm a RevoluteJoint with each name exists somewhere under
    # robot_path. The old check assumed a `{robot_path}/joints/{name}` layout
    # that only the procedural SlotCar authored; the TurtleBot3 reference has
    # joints scattered under various link prims.
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_path}")

    required = {left_joint, right_joint}
    wheel_joint_prims = []
    for p in Usd.PrimRange(robot_prim):
        if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() in required:
            wheel_joint_prims.append(p)
    found = {p.GetName() for p in wheel_joint_prims}
    missing = required - found
    if missing:
        raise RuntimeError(
            f"Drive joints not found under {robot_path}: {sorted(missing)}. "
            f"Check the articulation's joint names."
        )

    # Patch wheel drive gains (see `wheel_drive_*` docstrings above).
    for jp in wheel_joint_prims:
        drive = UsdPhysics.DriveAPI.Get(jp, "angular")
        if drive:
            prior_d = drive.GetDampingAttr().Get()
            prior_f = drive.GetMaxForceAttr().Get()
            drive.GetDampingAttr().Set(wheel_drive_damping)
            drive.GetMaxForceAttr().Set(wheel_drive_max_force)
            # Make sure stiffness is zero so we're in pure velocity mode.
            drive.GetStiffnessAttr().Set(0.0)
            print(f"[cmd_vel]   patched {jp.GetName()}: "
                  f"damping {prior_d}->{wheel_drive_damping}  "
                  f"maxForce {prior_f:.1e}->{wheel_drive_max_force}")

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
        # Feed physics substep size into DiffCtrl so its internal acceleration
        # limiter can ramp the wheel velocity command correctly. Without this
        # `dt` stays 0 and max{Linear,Angular}Acceleration are no-ops.
        ("OnPhysics.outputs:deltaSimulationTime", "DiffCtrl.inputs:dt"),
        ("DiffCtrl.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
    ]
    topic = _apply_namespace(namespace, topic)
    # IsaacArticulationController.targetPrim must resolve to the prim that
    # carries ArticulationRootAPI (the floating-base root RigidBody), not the
    # wrapping Xform. Passing a non-rigid ancestor fails with "Pattern ...
    # did not match any rigid bodies" once the node tries to bind its view.
    art_root_path = _find_articulation_root(stage, robot_path)
    set_values = [
        ("SubTwist.inputs:topicName", topic),
        ("DiffCtrl.inputs:wheelRadius", wheel_radius),
        ("DiffCtrl.inputs:wheelDistance", wheel_distance),
        ("DiffCtrl.inputs:maxLinearSpeed", max_linear_speed),
        ("DiffCtrl.inputs:maxAngularSpeed", max_angular_speed),
        ("DiffCtrl.inputs:maxWheelSpeed", max_wheel_speed),
        ("DiffCtrl.inputs:maxAcceleration", max_linear_accel),
        ("DiffCtrl.inputs:maxDeceleration", max_linear_decel),
        ("DiffCtrl.inputs:maxAngularAcceleration", max_angular_accel),
        ("ArtCtrl.inputs:targetPrim", [Sdf.Path(art_root_path)]),
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
    print(f"[cmd_vel]   maxLinearAccel={max_linear_accel} m/s^2  "
          f"maxLinearDecel={max_linear_decel} m/s^2  "
          f"maxAngularAccel={max_angular_accel} rad/s^2  (0=unlimited, "
          f"dt from OnPhysicsStep.deltaSimulationTime)")

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
