"""Conveyor belt placement (USD reference to a downloaded Isaac asset).

Pure `pxr` / usd-core. The builder authors a thin wrapper Xform under
`/World/Conveyors/<id>` and uses `Usd.References.AddReference` to bring an
Isaac Sim conveyor USD (downloaded by `tools/fetch_isaac_conveyor.py`)
into the stage. Geometry, materials and physics colliders all come from
the referenced asset; we only own the placement transform plus a small
customData block for runtime ROS2 wiring.

Scene shape:
    /World/Conveyors                                Xform (lazy container)
    └── <id>                                        Xform (this builder authors)
            • xformOpOrder = [translate, rotateZ?, scale?]
            • references   = [@<usd_path>@</World>]
            • customData   = {conveyor_id, name, ros2_speed_topic, …}
        └── <Isaac /World subtree composed via reference>
            ├── SM_ConveyorBelt_A08_02   (belt mesh, has PhysicsCollider)
            ├── Rollers                  (visual)
            ├── Anchorpoint              (Xform; surface-velocity ref for IsaacConveyor OG node)
            ├── Looks/…
            └── Physics_materials/…

Why a wrapper Xform (not adding ops on the referenced prim itself):
    Reference composition can carry xformOps from the asset's defaultPrim.
    Authoring our own ops on the same prim composes on top and the order
    is fragile. A clean wrapper guarantees translate/rotate/scale apply
    OUTSIDE whatever Isaac authored, and our customData isn't shadowed
    by reference contents.

Surface-velocity / belt motion is intentionally NOT wired here. That
needs the `IsaacConveyor` OmniGraph node, which only exists at runtime
inside Isaac Sim, so it lives in `scripts/run_sim.py` (or future
`syncai_omniverse.ros2.conveyor_controller`). The customData on the
wrapper carries the surface-prim hint and ROS2 topics so a runtime
pass can attach the OG node by walking `/World/Conveyors`.
"""
from __future__ import annotations

from pathlib import Path

from pxr import Gf, Usd, UsdGeom


def build_conveyor(stage: Usd.Stage, config: dict | None = None) -> str:
    """Author a conveyor placement onto `stage` by referencing an Isaac USD.

    Config keys:
        id (required): unique conveyor id; becomes the wrapper prim name
            under /World/Conveyors/ and drives default ROS2 topic naming
            (/conveyor/<id>/speed_cmd, /conveyor/<id>/status).
        name: human-readable label stored in customData (default: id).
        usd_path (required): path to the conveyor USD relative to the
            repo root (e.g. "assets/usd/conveyors/ConveyorBelt_A08.usd").
            Absolute paths are passed through. The path is rewritten to
            be relative to the scene file (which lives under <repo>/scenes/),
            so both `usdview` on the host and `run_sim.py` inside the
            container resolve it identically.
        position: [x, y, z] world placement of the conveyor's local origin
            at floor level (default [0, 0, 0]).
        rotation_z_deg: yaw in degrees (default 0).
        scale: uniform scale factor (default 1.0). Isaac assets are in
            metres so 1.0 is true-size.
        belt_surface_prim: prim path of the belt mesh whose surface
            velocity will be driven at runtime (default `<wrapper>/Belt`,
            which is wrong for many Isaac variants -- override per-variant
            in YAML if you intend to drive belt motion later).
        default_speed: surface speed in m/s for runtime startup (default 0).
        direction: 3-vector belt-local direction of travel (default [1, 0, 0]).
        ros2_speed_topic / ros2_status_topic: override default topic names.

    Returns the wrapper prim path.
    """
    cfg = config or {}
    try:
        conveyor_id = cfg["id"]
    except KeyError as exc:
        raise KeyError(
            "conveyor config missing required 'id' (used to build the prim "
            "path /World/Conveyors/<id> and ROS2 topic /conveyor/<id>/...)"
        ) from exc
    try:
        usd_path = cfg["usd_path"]
    except KeyError as exc:
        raise KeyError(
            f"conveyor config id={conveyor_id!r} missing required 'usd_path' "
            f"(repo-root-relative path to the Isaac conveyor USD; "
            f"run tools/fetch_isaac_conveyor.py first)"
        ) from exc

    name = cfg.get("name", conveyor_id)
    position = cfg.get("position", [0.0, 0.0, 0.0])
    rotation_z_deg = float(cfg.get("rotation_z_deg", 0.0))
    scale = float(cfg.get("scale", 1.0))

    # Lazy-create the shared container so the first conveyor wins and later
    # entries reuse it. Mirrors the /World/Doors / /World/ChargingStations
    # convention so run_sim.py can walk one root per kind.
    conveyors_root_path = "/World/Conveyors"
    if not stage.GetPrimAtPath(conveyors_root_path).IsValid():
        UsdGeom.Xform.Define(stage, conveyors_root_path)

    prim_path = f"{conveyors_root_path}/{conveyor_id}"
    xform = UsdGeom.Xform.Define(stage, prim_path)

    # Translate before Rotate before Scale (project rule -- AddTranslateOp
    # before AddScaleOp, otherwise the scale multiplies the offset and the
    # prim ends up in the wrong place; see feedback_usd_op_order memory).
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotation_z_deg != 0.0:
        xform.AddRotateZOp().Set(rotation_z_deg)
    if scale != 1.0:
        xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))

    # Reference the Isaac conveyor USD. Scene file lives at <repo>/scenes/<x>.usda;
    # the YAML usd_path is repo-root-relative, so prepend "../" to make the
    # reference path relative to the scene file's directory. Absolute paths
    # (e.g. /workspace/...) pass through untouched.
    if Path(usd_path).is_absolute():
        ref_path = usd_path
    else:
        ref_path = f"../{usd_path}"
    xform.GetPrim().GetReferences().AddReference(ref_path)

    # Stash runtime-discovery metadata on the wrapper. run_sim.py walks
    # /World/Doors today reading customData keys -- mirror that pattern so a
    # future runtime pass can attach an IsaacConveyor OG node + ROS2 bridge
    # without touching this builder.
    belt_surface_prim = cfg.get("belt_surface_prim", f"{prim_path}/Belt")
    ros2_speed_topic = cfg.get("ros2_speed_topic", f"/conveyor/{conveyor_id}/speed_cmd")
    ros2_status_topic = cfg.get("ros2_status_topic", f"/conveyor/{conveyor_id}/status")

    prim = xform.GetPrim()
    prim.SetCustomDataByKey("conveyor_id", conveyor_id)
    prim.SetCustomDataByKey("name", name)
    prim.SetCustomDataByKey("usd_path", usd_path)
    prim.SetCustomDataByKey("ros2_speed_topic", ros2_speed_topic)
    prim.SetCustomDataByKey("ros2_status_topic", ros2_status_topic)
    prim.SetCustomDataByKey("belt_surface_prim", belt_surface_prim)
    prim.SetCustomDataByKey("default_speed", float(cfg.get("default_speed", 0.0)))
    direction = cfg.get("direction", [1.0, 0.0, 0.0])
    prim.SetCustomDataByKey(
        "direction",
        Gf.Vec3f(float(direction[0]), float(direction[1]), float(direction[2])),
    )

    # Optional cargo-box spawn config consumed by run_sim.py at runtime
    # (DynamicCuboid). Authored as customData so the .usda is self-describing.
    test_box = cfg.get("test_box") or {}
    if test_box.get("enabled"):
        try:
            box_pos = test_box["position"]
        except KeyError as exc:
            raise KeyError(
                f"conveyor id={conveyor_id!r} test_box.enabled=true but "
                f"test_box.position is missing"
            ) from exc
        prim.SetCustomDataByKey("test_box_enabled", True)
        prim.SetCustomDataByKey("test_box_size", float(test_box.get("size", 0.3)))
        prim.SetCustomDataByKey("test_box_mass", float(test_box.get("mass", 5.0)))
        prim.SetCustomDataByKey(
            "test_box_position",
            Gf.Vec3f(float(box_pos[0]), float(box_pos[1]), float(box_pos[2])),
        )
    else:
        prim.SetCustomDataByKey("test_box_enabled", False)

    # Optional limit switch -- soft Python gate inside the runtime ScriptNode
    # that pins effective velocity to 0 when the cargo box's world position
    # along `axis` reaches `threshold`.
    limit = cfg.get("limit_switch") or {}
    if limit.get("enabled"):
        axis = str(limit.get("axis", "y")).lower()
        if axis not in ("x", "y", "z"):
            raise ValueError(
                f"conveyor id={conveyor_id!r} limit_switch.axis must be "
                f"'x'|'y'|'z' (got {axis!r})"
            )
        comparator = str(limit.get("comparator", "ge")).lower()
        if comparator not in ("ge", "le"):
            raise ValueError(
                f"conveyor id={conveyor_id!r} limit_switch.comparator must "
                f"be 'ge'|'le' (got {comparator!r})"
            )
        try:
            threshold = float(limit["threshold"])
        except KeyError as exc:
            raise KeyError(
                f"conveyor id={conveyor_id!r} limit_switch.enabled=true but "
                f"limit_switch.threshold is missing"
            ) from exc
        prim.SetCustomDataByKey("limit_switch_enabled", True)
        prim.SetCustomDataByKey("limit_switch_axis", axis)
        prim.SetCustomDataByKey("limit_switch_comparator", comparator)
        prim.SetCustomDataByKey("limit_switch_threshold", threshold)
    else:
        prim.SetCustomDataByKey("limit_switch_enabled", False)

    return prim_path
