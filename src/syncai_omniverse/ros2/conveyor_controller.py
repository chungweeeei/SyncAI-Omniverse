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
    # Pickup / handoff state machine. While `phase == "carried"` the box is
    # kinematic and its world transform is rewritten each tick from the
    # winning robot's base_link pose * attach_local_offset.
    "pickup_enabled": False,
    "pickup_candidates": [],     # ["/World/SyncRobot01/base_link", ...]
    "pickup_dock_xy": (0.0, 0.0),
    "pickup_dock_radius_sq": 0.0,
    "pickup_offset": (0.0, 0.0, 0.0),
    "pickup_follow_yaw": True,
    "phase": "belt",             # belt | handoff | carried | dropped
    "carried_base_link": "",     # winning base_link prim path
    # Carry is implemented via a UsdPhysics.FixedJoint between the robot's
    # base_link and the box (both dynamic). PhysX maintains the constraint
    # so we don't need per-tick xformOp writes. Earlier iterations used a
    # kinematic-toggle approach which (a) PhysX never picked up at runtime
    # in Isaac Sim 5.1, and (b) the rigidBodyEnabled rebuild workaround
    # SEGFAULTED PhysX. The joint approach sidesteps both.
    "carry_joint_path": "",      # path of the active FixedJoint, "" if none
    "carry_active": False,       # True while joint is in place
    # Drop-off via /cargo/drop_cmd (std_msgs/String). last_drop_cmd_text
    # dedupes so each unique payload is processed exactly once; to retry
    # after rejection, the publisher sends a different value first to reset.
    "drop_zones": {},            # {id: (cx, cy, r, dx, dy, dz), ...}
    "drop_cmd_attr": None,       # cached og.Controller.attribute for sub.outputs:data
    "last_drop_cmd_text": "",    # dedupe key
    "dropped_zone": "",          # zone id captured at the moment of release
    # Multi-box sequential lifecycle (spawn_cmd respawns after dropped).
    "current_box_id": "",        # id of the prim currently in flight
    "box_count": 0,              # auto-increment counter for default ids
    "spawn_cmd_attr": None,      # cached og.Controller.attribute
    "last_spawn_cmd_text": "",   # dedupe key for spawn_cmd
    "spawn_pos": (0.0, 0.0, 0.0),
    "spawn_size": 0.3,
    "spawn_mass": 5.0,
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

    # Pickup / handoff configuration.
    try:
        _state["pickup_enabled"] = bool(db.inputs.pickupEnabled)
    except Exception:
        _state["pickup_enabled"] = False
    if _state["pickup_enabled"]:
        try:
            cands_csv = str(db.inputs.pickupCandidates)
        except Exception:
            cands_csv = ""
        # Resolve candidate robot ROOT paths to their /<root>/base_link form.
        _state["pickup_candidates"] = [
            c.strip().rstrip("/") + "/base_link"
            for c in cands_csv.split(",")
            if c.strip()
        ]
        try:
            _state["pickup_dock_xy"] = (
                float(db.inputs.pickupDockX),
                float(db.inputs.pickupDockY),
            )
        except Exception:
            _state["pickup_dock_xy"] = (0.0, 0.0)
        try:
            r = float(db.inputs.pickupDockRadius)
        except Exception:
            r = 0.0
        _state["pickup_dock_radius_sq"] = r * r
        try:
            _state["pickup_offset"] = (
                float(db.inputs.pickupOffX),
                float(db.inputs.pickupOffY),
                float(db.inputs.pickupOffZ),
            )
        except Exception:
            _state["pickup_offset"] = (0.0, 0.0, 0.0)
        try:
            _state["pickup_follow_yaw"] = bool(db.inputs.pickupFollowYaw)
        except Exception:
            _state["pickup_follow_yaw"] = True

    # Drop-cmd subscriber attribute (std_msgs/String, payload = drop-zone id).
    try:
        _state["drop_cmd_attr"] = og.Controller.attribute(
            str(db.inputs.dropCmdAttrPath)
        )
    except Exception:
        _state["drop_cmd_attr"] = None

    # dropZones is csv "id:cx:cy:r:dx:dy:dz,..." so the ScriptNode doesn't
    # need a complex input schema. (cx,cy,r) = zone center / radius for the
    # robot-in-zone check; (dx,dy,dz) = drop_pose the box is teleported to
    # the moment its FixedJoint is removed.
    try:
        zones_csv = str(db.inputs.dropZones)
    except Exception:
        zones_csv = ""
    zones = {}
    for tok in zones_csv.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            parts = tok.split(":")
            zid = parts[0]
            cx, cy, r = float(parts[1]), float(parts[2]), float(parts[3])
            dx, dy, dz = float(parts[4]), float(parts[5]), float(parts[6])
            zones[zid] = (cx, cy, r, dx, dy, dz)
        except Exception:
            print(f"[conveyor] bad drop_zone token {tok!r}")
    _state["drop_zones"] = zones

    # Spawn-cmd subscriber wiring (multi-box sequential lifecycle).
    try:
        _state["spawn_cmd_attr"] = og.Controller.attribute(
            str(db.inputs.spawnCmdAttrPath)
        )
    except Exception:
        _state["spawn_cmd_attr"] = None

    # Initial box id (spawned by run_sim.py before the graph attaches) and
    # seed the auto-increment counter from any trailing digits so the
    # first auto-generated id is +1 of the initial.
    try:
        initial = str(db.inputs.initialBoxId) or "box01"
    except Exception:
        initial = "box01"
    _state["current_box_id"] = initial
    import re as _re
    _m = _re.search(r"(\d+)$", initial)
    _state["box_count"] = int(_m.group(1)) if _m else 1

    # Spawn config used by _spawn_box for every subsequent box on this
    # conveyor (size / mass / spawn position match the YAML test_box).
    try:
        _state["spawn_pos"] = (
            float(db.inputs.boxSpawnX),
            float(db.inputs.boxSpawnY),
            float(db.inputs.boxSpawnZ),
        )
    except Exception:
        _state["spawn_pos"] = (0.0, 0.0, 0.0)
    try:
        _state["spawn_size"] = float(db.inputs.boxSpawnSize)
    except Exception:
        _state["spawn_size"] = 0.3
    try:
        _state["spawn_mass"] = float(db.inputs.boxSpawnMass)
    except Exception:
        _state["spawn_mass"] = 5.0


def _spawn_box(box_id):
    \"\"\"Create /World/CargoBoxes/<box_id> with the same DynamicCuboid
    setup as the initial run_sim.py spawn. Runs from compute() (main
    physics tick) so the isaacsim.core.api factory works. Updates the
    cursors `_state["current_box_id"]` and `_state["box_prim_path"]`.

    If mid-step DynamicCuboid creation ever destabilises PhysX, fall back
    to deferring spawn via run_sim.py (set a `_state["pending_spawn"]`
    flag and have run_sim.py's main loop poll it).\"\"\"
    try:
        from isaacsim.core.api.objects import DynamicCuboid
        import numpy as np
        prim_path = f"/World/CargoBoxes/{box_id}"
        DynamicCuboid(
            prim_path=prim_path,
            position=np.array([
                _state["spawn_pos"][0],
                _state["spawn_pos"][1],
                _state["spawn_pos"][2],
            ]),
            size=float(_state["spawn_size"]),
            mass=float(_state["spawn_mass"]),
            color=np.array([0.85, 0.55, 0.10]),
        )
        _state["current_box_id"] = box_id
        _state["box_prim_path"] = prim_path
        print(f"[conveyor] _spawn_box: created {prim_path}")
    except Exception as exc:
        print(f"[conveyor] _spawn_box failed: {exc}")


def _zero_box_velocity(box_prim):
    \"\"\"Author physics:velocity / physics:angularVelocity = (0,0,0) so a
    just-teleported or just-released box doesn't carry stale momentum.\"\"\"
    try:
        from pxr import Gf, UsdPhysics
        rb_api = UsdPhysics.RigidBodyAPI(box_prim)
        v_attr = box_prim.GetAttribute("physics:velocity")
        if not v_attr or not v_attr.IsValid():
            v_attr = rb_api.CreateVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        v_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        av_attr = box_prim.GetAttribute("physics:angularVelocity")
        if not av_attr or not av_attr.IsValid():
            av_attr = rb_api.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        av_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
    except Exception as exc:
        print(f"[conveyor] zero_box_velocity failed: {exc}")


def _enter_carried(box_prim, base_link_path, local_offset, follow_yaw):
    \"\"\"Carry mode = box stays DYNAMIC, constrained to the robot's base_link
    by a UsdPhysics.FixedJoint. PhysX maintains the constraint every step,
    so we don't need per-tick xformOp writes (the kinematic-toggle approach
    used previously didn't propagate at runtime in Isaac Sim 5.1).

    Sequence:
      1. Compute target world pose = base_link_world * local_offset.
      2. Snap the box to that pose with a single xformOp:translate (and
         optional xformOp:orient) write so the joint doesn't have to solve
         a large initial constraint error.
      3. Zero box velocities so the snap doesn't carry stale momentum.
      4. Author /World/CargoBoxes/<box>_attach_joint as a FixedJoint
         between base_link (body0) and the box (body1).\"\"\"
    if _state.get("carry_active"):
        return
    try:
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Usd
        stage = _state["stage"]

        rp = stage.GetPrimAtPath(base_link_path)
        if not (rp and rp.IsValid()):
            print(f"[conveyor] _enter_carried: base_link not found: {base_link_path}")
            return

        # 1. Compute target world pose from base_link transform.
        xf = UsdGeom.Xformable(rp).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        ox, oy, oz = local_offset
        offset_h = Gf.Vec4d(ox, oy, oz, 1.0)
        world_h = offset_h * xf
        target_pos = (float(world_h[0]), float(world_h[1]), float(world_h[2]))

        # 2. Snap box translate (precision-aware) before authoring the joint.
        xfb = UsdGeom.Xformable(box_prim)
        ops_dump = [
            (o.GetName(), str(o.GetOpType()).split(".")[-1],
             str(o.GetPrecision()).split(".")[-1])
            for o in xfb.GetOrderedXformOps()
        ]
        print(f"[conveyor] _enter_carried: box={box_prim.GetPath()} xformOps={ops_dump}")

        t_op = None
        for op in xfb.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                t_op = op
                break
        if t_op is None:
            t_op = xfb.AddTranslateOp()
        if t_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            t_op.Set(Gf.Vec3f(*target_pos))
        else:
            t_op.Set(Gf.Vec3d(*target_pos))

        if follow_yaw:
            rot_q = xf.ExtractRotation().GetQuaternion()
            o_op = None
            for op in xfb.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    o_op = op
                    break
            if o_op is None:
                o_op = xfb.AddOrientOp()
            real = float(rot_q.GetReal())
            im = rot_q.GetImaginary()
            if o_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                o_op.Set(Gf.Quatf(real,
                                  float(im[0]), float(im[1]), float(im[2])))
            else:
                o_op.Set(Gf.Quatd(real,
                                  float(im[0]), float(im[1]), float(im[2])))

        # 3. Zero velocities so the teleport doesn't carry momentum.
        _zero_box_velocity(box_prim)

        # 4. Author the FixedJoint. localPos0 in base_link frame is the
        # attach offset; localPos1 in box frame is origin.
        box_path = str(box_prim.GetPath())
        joint_path = f"{box_path}_attach_joint"
        if stage.GetPrimAtPath(joint_path).IsValid():
            stage.RemovePrim(joint_path)
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(base_link_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(box_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(float(ox), float(oy), float(oz)))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        _state["carry_joint_path"] = joint_path
        _state["carry_active"] = True
        print(f"[conveyor] _enter_carried: created FixedJoint {joint_path}")
    except Exception as exc:
        print(f"[conveyor] _enter_carried failed: {exc}")


def _exit_carried(box_prim, drop_pose=None):
    \"\"\"Drop sequence:

      1. Remove the FixedJoint so the box is no longer constrained to
         the robot.
      2. If `drop_pose` is given, snap the box's xformOp:translate to that
         world point (precision-aware). With the joint already gone PhysX
         takes the write as a teleport for an unconstrained dynamic body.
         If `drop_pose` is None, leave the box at its current carry pose.
      3. Zero linear/angular velocities so the teleport / release doesn't
         carry residual momentum from being whipped around on the robot.

    After this, the box is a free dynamic RigidBody at `drop_pose` (or its
    last carry pose) with zero velocity -- gravity takes over from there.\"\"\"
    try:
        stage = _state["stage"]
        # 1. Remove the joint first so subsequent translate writes aren't
        # fighting the constraint.
        joint_path = _state.get("carry_joint_path", "")
        if joint_path and stage.GetPrimAtPath(joint_path).IsValid():
            stage.RemovePrim(joint_path)
            print(f"[conveyor] _exit_carried: removed joint {joint_path}")
        else:
            print(f"[conveyor] _exit_carried: no joint to remove (path={joint_path!r})")

        # 2. Snap to drop_pose if requested.
        if drop_pose is not None:
            from pxr import Gf, UsdGeom
            xfb = UsdGeom.Xformable(box_prim)
            t_op = None
            for op in xfb.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    t_op = op
                    break
            if t_op is None:
                t_op = xfb.AddTranslateOp()
            dx, dy, dz = drop_pose
            if t_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                t_op.Set(Gf.Vec3f(float(dx), float(dy), float(dz)))
            else:
                t_op.Set(Gf.Vec3d(float(dx), float(dy), float(dz)))
            print(
                f"[conveyor] _exit_carried: snapped box to drop_pose "
                f"({dx:.2f},{dy:.2f},{dz:.2f})"
            )
    except Exception as exc:
        print(f"[conveyor] _exit_carried failed: {exc}")

    # 3. Zero velocities (always — protects against post-teleport drift
    # AND post-release angular kick from base_link rotation).
    _zero_box_velocity(box_prim)

    _state["carry_joint_path"] = ""
    _state["carry_active"] = False
    print(f"[conveyor] _exit_carried done for {box_prim.GetPath()}")


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

    # Pickup / handoff state machine. Runs only when configured AND there is
    # a box prim to chase. Belt motion is unaffected -- carry mode is purely
    # additive: it pose-locks the box on top of a robot once that robot
    # arrives in the dock zone.
    phase = _state.get("phase", "belt")
    if _state["pickup_enabled"] and box_path:
        stage = _state["stage"]
        box_prim = stage.GetPrimAtPath(box_path) if stage is not None else None

        chosen = ""
        if gated and box_prim and box_prim.IsValid():
            from pxr import UsdGeom, Usd
            dock_x, dock_y = _state["pickup_dock_xy"]
            r2 = _state["pickup_dock_radius_sq"]
            best_d2 = float("inf")
            for bl_path in _state["pickup_candidates"]:
                rp = stage.GetPrimAtPath(bl_path)
                if not (rp and rp.IsValid()):
                    continue
                try:
                    xf = UsdGeom.Xformable(rp).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    )
                    wp = xf.ExtractTranslation()
                    dx = wp[0] - dock_x
                    dy = wp[1] - dock_y
                    d2 = dx * dx + dy * dy
                    if d2 <= r2 and d2 < best_d2:
                        best_d2 = d2
                        chosen = bl_path
                except Exception:
                    pass

        # Phase transitions. carried -> dropped is handled in the drop-cmd
        # block below; this block only handles belt/handoff -> carried.
        if phase == "belt":
            if gated and chosen:
                phase = "carried"
                _state["carried_base_link"] = chosen
                if box_prim and box_prim.IsValid():
                    _enter_carried(
                        box_prim,
                        chosen,
                        _state["pickup_offset"],
                        _state["pickup_follow_yaw"],
                    )
            elif gated:
                phase = "handoff"
        elif phase == "handoff":
            if not gated:
                phase = "belt"
            elif chosen:
                phase = "carried"
                _state["carried_base_link"] = chosen
                if box_prim and box_prim.IsValid():
                    _enter_carried(
                        box_prim,
                        chosen,
                        _state["pickup_offset"],
                        _state["pickup_follow_yaw"],
                    )
        # During `carried` phase the FixedJoint authored by _enter_carried
        # holds the box on the robot; no per-tick xformOp writes needed.

        _state["phase"] = phase

    # Spawn-cmd handling. Same dedupe shape as drop_cmd: only the FIRST
    # tick that sees a unique non-empty payload acts. To retry, publisher
    # sends a different value (e.g. another `_reset_N` sentinel) first.
    spawn_text = ""
    sca = _state["spawn_cmd_attr"]
    if sca is None:
        try:
            _state["spawn_cmd_attr"] = og.Controller.attribute(
                str(db.inputs.spawnCmdAttrPath)
            )
            sca = _state["spawn_cmd_attr"]
        except Exception:
            sca = None
    if sca is not None:
        try:
            v = sca.get()
            spawn_text = str(v) if v else ""
        except Exception:
            spawn_text = ""

    if spawn_text and spawn_text != _state["last_spawn_cmd_text"]:
        _state["last_spawn_cmd_text"] = spawn_text
        if _state.get("phase") != "dropped":
            print(
                f"[conveyor] spawn_cmd ignored (phase={_state.get('phase')!r})"
            )
        else:
            # Auto-increment when payload is empty/any/sentinel; otherwise
            # honour the explicit id verbatim.
            if spawn_text in ("any", "") or spawn_text.startswith("_"):
                _state["box_count"] += 1
                new_id = f"box{_state['box_count']:02d}"
            else:
                new_id = spawn_text
            _spawn_box(new_id)
            _state["phase"] = "belt"
            _state["dropped_zone"] = ""
            _state["last_drop_cmd_text"] = ""   # next drop is fresh
            _state["carry_active"] = False
            _state["carry_joint_path"] = ""
            _state["carried_base_link"] = ""
            print(f"[conveyor] respawned: phase=belt box={new_id}")

    # Drop-cmd handling. Polls every tick; only acts on each unique non-empty
    # payload once (deduped via last_drop_cmd_text). To retry after a reject,
    # the publisher sends a different (e.g. empty) value first to reset, then
    # re-sends the desired zone id.
    drop_status_override = ""
    dca = _state["drop_cmd_attr"]
    if dca is None:
        try:
            _state["drop_cmd_attr"] = og.Controller.attribute(
                str(db.inputs.dropCmdAttrPath)
            )
            dca = _state["drop_cmd_attr"]
        except Exception:
            dca = None
    drop_text = ""
    if dca is not None:
        try:
            v = dca.get()
            drop_text = str(v) if v else ""
        except Exception:
            drop_text = ""

    if drop_text and drop_text != _state["last_drop_cmd_text"]:
        _state["last_drop_cmd_text"] = drop_text
        # Parse the payload. Strict format is "box_id:zone_id"; legacy
        # zone-only (no colon) and the `any`/`""` debug bypasses are still
        # accepted -- in those cases box-id check is skipped.
        incoming_box_id = ""
        zone_id_text = drop_text
        if drop_text not in ("any", "") and ":" in drop_text:
            incoming_box_id, zone_id_text = drop_text.split(":", 1)

        # Box-id mismatch check (only when caller specifies one).
        if incoming_box_id and incoming_box_id != _state.get("current_box_id", ""):
            drop_status_override = "drop_rejected:box_mismatch"
            print(
                f"[conveyor] drop_cmd rejected: box mismatch "
                f"(cmd={incoming_box_id!r} current="
                f"{_state.get('current_box_id')!r})"
            )
        elif _state.get("phase") == "carried":
            zone_ok = True
            zone_id = zone_id_text
            drop_pose = None  # None => keep current carry pose; else (x,y,z)
            if zone_id_text not in ("any", ""):
                zone = _state["drop_zones"].get(zone_id_text)
                if zone is None:
                    zone_ok = False
                    drop_status_override = f"drop_rejected:{zone_id_text}"
                    print(f"[conveyor] drop_cmd unknown zone {zone_id_text!r}")
                else:
                    # Zone tuple = (cx, cy, r, dx, dy, dz).
                    zx, zy, zr, dx, dy, dz = zone
                    drop_pose = (dx, dy, dz)
                    # Verify the carrying robot is in zone radius (XY only).
                    bl = _state["carried_base_link"]
                    rp = _state["stage"].GetPrimAtPath(bl) if bl else None
                    if rp and rp.IsValid():
                        try:
                            from pxr import UsdGeom, Usd
                            xf = UsdGeom.Xformable(rp).ComputeLocalToWorldTransform(
                                Usd.TimeCode.Default()
                            )
                            wp = xf.ExtractTranslation()
                            d2 = (wp[0] - zx) ** 2 + (wp[1] - zy) ** 2
                            if d2 > zr * zr:
                                zone_ok = False
                                drop_status_override = (
                                    f"drop_rejected:{zone_id_text}"
                                )
                                print(
                                    f"[conveyor] drop_cmd rejected: robot at "
                                    f"({wp[0]:.2f},{wp[1]:.2f}) not within "
                                    f"{zr:.2f}m of zone {zone_id_text} "
                                    f"({zx:.2f},{zy:.2f})"
                                )
                        except Exception:
                            zone_ok = False
            if zone_ok and box_path:
                box_prim_d = _state["stage"].GetPrimAtPath(box_path)
                if box_prim_d and box_prim_d.IsValid():
                    _exit_carried(box_prim_d, drop_pose)
                    _state["phase"] = "dropped"
                    _state["dropped_zone"] = zone_id
                    bid = _state.get("current_box_id", "")
                    print(
                        f"[conveyor] dropped {bid!r} at zone {zone_id}"
                        f"{' pose=' + str(drop_pose) if drop_pose else ''}"
                    )

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

    phase_now = _state.get("phase")
    bid = _state.get("current_box_id", "")
    if drop_status_override:
        # One-shot rejection notice; phase didn't change so next tick falls
        # back to whatever the resumed state is.
        state_str = drop_status_override
    elif phase_now == "dropped":
        zid = _state.get("dropped_zone", "")
        if bid and zid:
            state_str = f"dropped:{bid}@{zid}"
        elif zid:
            state_str = f"dropped:{zid}"
        else:
            state_str = "dropped"
    elif phase_now == "carried":
        bl = _state["carried_base_link"]
        # /World/<robot_name>/base_link -> "<robot_name>"
        try:
            robot_name = bl.rsplit("/", 2)[-2] if bl else ""
        except Exception:
            robot_name = ""
        if bid and robot_name:
            state_str = f"carried:{bid}@{robot_name}"
        elif robot_name:
            state_str = f"carried:{robot_name}"
        else:
            state_str = "carried"
    elif phase_now == "handoff":
        state_str = "handoff"
    elif gated:
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
    pickup_enabled: bool = False,
    pickup_candidates: list | None = None,
    pickup_dock_xy: tuple | None = None,
    pickup_dock_radius: float = 0.6,
    pickup_attach_offset: tuple = (0.0, 0.0, 0.30),
    pickup_follow_yaw: bool = True,
    drop_cmd_topic: str = "/cargo/drop_cmd",
    drop_zones: list | None = None,
    spawn_cmd_topic: str = "",
    initial_box_id: str = "box01",
    box_spawn_position: tuple = (0.0, 0.0, 0.0),
    box_spawn_size: float = 0.3,
    box_spawn_mass: float = 5.0,
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
    drop_cmd_topic_full = _apply_namespace(namespace, drop_cmd_topic)
    # Default spawn topic per-conveyor (parallels /conveyor/<id>/...).
    if not spawn_cmd_topic:
        spawn_cmd_topic = f"/cargo/{conv_prim.GetName()}/spawn_cmd"
    spawn_cmd_topic_full = _apply_namespace(namespace, spawn_cmd_topic)
    graph_path = graph_path or f"/ConveyorGraph_{conv_prim.GetName()}"
    sub_attr_path = f"{graph_path}/SubFloat32.outputs:data"
    vel_attr_path = f"{graph_path}/IsaacConveyor.inputs:velocity"
    drop_cmd_attr_path = f"{graph_path}/SubDropCmd.outputs:data"
    spawn_cmd_attr_path = f"{graph_path}/SubSpawnCmd.outputs:data"
    # Per-conveyor rclpy node name -- must be unique across conveyors so
    # multiple ConveyorGraph_* in the same process don't collide.
    ros_node_name = f"conveyor_{conv_prim.GetName()}_status_pub".replace("/", "_")

    # Direction is used by IsaacConveyor's input as a Vec3f-ish list. Some
    # IsaacConveyor versions accept tuples; pass a list to be safe.
    direction_list = [float(direction[0]), float(direction[1]), float(direction[2])]

    create_nodes = [
        ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubFloat32", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ("SubDropCmd", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ("SubSpawnCmd", "isaacsim.ros2.bridge.ROS2Subscriber"),
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
        ("ConveyorScript.inputs:pickupEnabled", "bool"),
        ("ConveyorScript.inputs:pickupCandidates", "string"),
        ("ConveyorScript.inputs:pickupDockX", "float"),
        ("ConveyorScript.inputs:pickupDockY", "float"),
        ("ConveyorScript.inputs:pickupDockRadius", "float"),
        ("ConveyorScript.inputs:pickupOffX", "float"),
        ("ConveyorScript.inputs:pickupOffY", "float"),
        ("ConveyorScript.inputs:pickupOffZ", "float"),
        ("ConveyorScript.inputs:pickupFollowYaw", "bool"),
        ("ConveyorScript.inputs:dropCmdAttrPath", "string"),
        ("ConveyorScript.inputs:dropZones", "string"),
        ("ConveyorScript.inputs:spawnCmdAttrPath", "string"),
        ("ConveyorScript.inputs:initialBoxId", "string"),
        ("ConveyorScript.inputs:boxSpawnX", "float"),
        ("ConveyorScript.inputs:boxSpawnY", "float"),
        ("ConveyorScript.inputs:boxSpawnZ", "float"),
        ("ConveyorScript.inputs:boxSpawnSize", "float"),
        ("ConveyorScript.inputs:boxSpawnMass", "float"),
    ]
    # IsaacConveyor needs both its onStep execution input AND a delta-time
    # value wired or it silently no-ops -- the OGN spec lists inputs:onStep
    # (execution) and inputs:delta (float) as required even though `enabled`
    # and `velocity` are set. Without these, /status reports "running" but
    # PhysxSurfaceVelocity never gets written.
    connect = [
        ("OnPhysics.outputs:step", "SubFloat32.inputs:execIn"),
        ("OnPhysics.outputs:step", "SubDropCmd.inputs:execIn"),
        ("OnPhysics.outputs:step", "SubSpawnCmd.inputs:execIn"),
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
        ("ConveyorScript.inputs:pickupEnabled", bool(pickup_enabled)),
        ("ConveyorScript.inputs:pickupCandidates",
         ",".join(pickup_candidates) if pickup_candidates else ""),
        ("ConveyorScript.inputs:pickupDockX",
         float(pickup_dock_xy[0]) if pickup_dock_xy else 0.0),
        ("ConveyorScript.inputs:pickupDockY",
         float(pickup_dock_xy[1]) if pickup_dock_xy else 0.0),
        ("ConveyorScript.inputs:pickupDockRadius", float(pickup_dock_radius)),
        ("ConveyorScript.inputs:pickupOffX", float(pickup_attach_offset[0])),
        ("ConveyorScript.inputs:pickupOffY", float(pickup_attach_offset[1])),
        ("ConveyorScript.inputs:pickupOffZ", float(pickup_attach_offset[2])),
        ("ConveyorScript.inputs:pickupFollowYaw", bool(pickup_follow_yaw)),
        ("SubDropCmd.inputs:topicName", drop_cmd_topic_full),
        ("SubDropCmd.inputs:messagePackage", "std_msgs"),
        ("SubDropCmd.inputs:messageSubfolder", "msg"),
        ("SubDropCmd.inputs:messageName", "String"),
        ("ConveyorScript.inputs:dropCmdAttrPath", drop_cmd_attr_path),
        ("ConveyorScript.inputs:dropZones",
         ",".join(
             # Each tuple: (id, cx, cy, r, dx, dy, dz). The 7-field CSV is
             # parsed back inside the ScriptNode's setup().
             f"{z[0]}:{float(z[1])}:{float(z[2])}:{float(z[3])}"
             f":{float(z[4])}:{float(z[5])}:{float(z[6])}"
             for z in (drop_zones or [])
         )),
        ("SubSpawnCmd.inputs:topicName", spawn_cmd_topic_full),
        ("SubSpawnCmd.inputs:messagePackage", "std_msgs"),
        ("SubSpawnCmd.inputs:messageSubfolder", "msg"),
        ("SubSpawnCmd.inputs:messageName", "String"),
        ("ConveyorScript.inputs:spawnCmdAttrPath", spawn_cmd_attr_path),
        ("ConveyorScript.inputs:initialBoxId", initial_box_id),
        ("ConveyorScript.inputs:boxSpawnX", float(box_spawn_position[0])),
        ("ConveyorScript.inputs:boxSpawnY", float(box_spawn_position[1])),
        ("ConveyorScript.inputs:boxSpawnZ", float(box_spawn_position[2])),
        ("ConveyorScript.inputs:boxSpawnSize", float(box_spawn_size)),
        ("ConveyorScript.inputs:boxSpawnMass", float(box_spawn_mass)),
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
    if pickup_enabled:
        dxy = pickup_dock_xy or (0.0, 0.0)
        print(
            f"[conveyor]   pickup: dock=({dxy[0]:.2f},{dxy[1]:.2f}) "
            f"r={pickup_dock_radius:.2f} offset={pickup_attach_offset} "
            f"follow_yaw={pickup_follow_yaw} cands={pickup_candidates or []}"
        )
    print(
        f"[conveyor]   drop: topic={drop_cmd_topic_full} "
        f"zones={drop_zones or []}"
    )
    print(
        f"[conveyor]   spawn: topic={spawn_cmd_topic_full} "
        f"initial_id={initial_box_id} pos={tuple(float(v) for v in box_spawn_position)} "
        f"size={float(box_spawn_size)} mass={float(box_spawn_mass)}"
    )

    if debug:
        print("[conveyor][debug] nodes:")
        for name, type_ in create_nodes:
            print(f"[conveyor][debug]   {graph_path}/{name}  <{type_}>")

    return graph_path
