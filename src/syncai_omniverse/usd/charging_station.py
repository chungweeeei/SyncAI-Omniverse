"""MiR-style rear-docking charging station (static prop).

Authored as a USD scene fragment that plugs into `scene.build_combined_scene`.
Pure `pxr` / usd-core, no Isaac Sim dependency. The dock is world-anchored:
all colliders carry only `UsdPhysics.CollisionAPI`, no `RigidBodyAPI`. This
mirrors the static-jamb pattern in `auto_door.py`.

Local frame:
    +X = docking face normal (the AMR reverses along -X to dock)
    +Y = dock width
    +Z = up

Scene shape:
    /World/ChargingStations/<name>          Xform (pose carrier)
    ├── enclosure                           Xform
    │   └── body_{visual,collision}         Cube (white body)
    ├── faceplate                           Xform (translated +X to front)
    │   ├── plate_{visual,collision}        Cube (white faceplate slab)
    │   └── window_visual                   Cube (dark recessed visual landmark)
    └── approach_pad                        Xform (floor docking plate with electrodes)
        ├── base_visual                     Cube (white plate)
        ├── electrode_1_visual              Cube (copper contact strip, -Y cluster)
        ├── electrode_2_visual              Cube (copper contact strip, -Y cluster)
        └── electrode_3_visual              Cube (copper contact strip, -Y cluster)

Geometry is hard-coded to the real MiR Charge proportions (~0.58 W x 0.27 D
x 0.30 H). YAML only exposes pose. The approach pad is a small (0.30 x 0.58 m)
floor plate at the dock face -- mirroring the real MiR charging plate
footprint. The 3 copper strips on the pad are the floor-side electrode
contacts; the AMR drives over them and contacts from below. Decorative only
(no collider) so the robot drives straight over it.
"""
from pxr import Gf, Usd, UsdGeom

from syncai_omniverse.usd._amr_common import (
    _add_box,
    _define_preview_material,
)


# MiR Charge real-world reference dimensions (metres). Local axes:
# X = depth (faceplate normal direction), Y = width, Z = height.
_BODY_DEPTH = 0.27
_BODY_WIDTH = 0.58
_BODY_HEIGHT = 0.30

_PLATE_THICKNESS = 0.01
_PLATE_HEIGHT = _BODY_HEIGHT          # flush with body top/bottom
_PLATE_WIDTH = _BODY_WIDTH - 0.02     # slight inset from body side edges
_FACEPLATE_OFFSET_X = _BODY_DEPTH / 2.0 + _PLATE_THICKNESS / 2.0  # 0.140

# Dark recessed window panel on the faceplate. Acts as a visual fiducial for
# camera-based dock detection (the high-contrast dark rectangle on a white
# body is what a vision pipeline can latch onto).
_WINDOW_WIDTH = 0.34
_WINDOW_HEIGHT = 0.14
_WINDOW_THICKNESS = 0.005
_WINDOW_PROUD = 0.003                 # sits this far in front of plate front
_WINDOW_Z = 0.20                      # vertical centre on the faceplate

# Approach pad: small floor plate matching the real MiR charging-plate
# footprint. Holds 3 copper electrode strips parallel to the docking
# direction (X). The AMR reverses over the pad and contacts from below.
_PAD_DEPTH = 0.30                     # X extent away from dock face
_PAD_WIDTH = _BODY_WIDTH              # 0.58 m, matches dock body width
_PAD_THICKNESS = 0.005                # 5 mm slab; sits just above the floor
_PAD_GAP = 0.005                      # small air gap between dock and pad

_ELECTRODE_LENGTH = 0.20              # X extent (along docking direction)
_ELECTRODE_WIDTH = 0.04               # Y extent (strip width)
_ELECTRODE_THICKNESS = 0.003          # 3 mm raised above pad top
_ELECTRODE_GROUP_CENTER_Y = -0.15     # Y centre of the (right-clustered) group
_ELECTRODE_PITCH = 0.08               # centre-to-centre spacing in Y
_ELECTRODE_PROUD = 0.0005             # 0.5 mm above pad top to avoid z-fight


def build_charging_station(stage: Usd.Stage, config: dict | None = None) -> str:
    """Author one MiR-style rear-docking charger onto `stage`.

    Config keys:
        name: prim name under /World/ChargingStations/ (default "Charger01").
        id: logical id (required). Stamped on customData["charger_id"]; not
            used to derive any topic since this is a static prop.
        position: [x, y, z] world position of the dock base centre at floor
            (default [0, 0, 0]).
        rotation_z_deg: yaw of the dock (default 0.0). 0 = faceplate normal
            points +X; AMR reverses along -X to dock.
        approach_pad: bool (default True). When True, author the white
            floor plate with 3 copper electrode strips. No collider -- the
            robot drives straight over it.

    Returns the charger root prim path.
    """
    config = config or {}
    name = config.get("name", "Charger01")
    try:
        charger_id = config["id"]
    except KeyError as exc:
        raise KeyError(
            f"charging_station config for name={name!r} is missing required 'id'"
        ) from exc
    position = config.get("position", [0.0, 0.0, 0.0])
    rotation_z_deg = float(config.get("rotation_z_deg", 0.0))
    enable_approach_pad = bool(config.get("approach_pad", True))

    # Ensure the shared container Xform exists. Multiple chargers can live
    # under /World/ChargingStations.
    root_path = "/World/ChargingStations"
    if not stage.GetPrimAtPath(root_path).IsValid():
        UsdGeom.Xform.Define(stage, root_path)

    charger_path = f"{root_path}/{name}"

    # -- Charger root: TranslateOp BEFORE RotateZOp so yaw spins around the
    # dock base centre, not around the world origin (memory rule).
    charger_xform = UsdGeom.Xform.Define(stage, charger_path)
    charger_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotation_z_deg != 0.0:
        charger_xform.AddRotateZOp().Set(rotation_z_deg)

    charger_prim = charger_xform.GetPrim()
    charger_prim.SetCustomDataByKey("charger_id", charger_id)
    charger_prim.SetCustomDataByKey("version", "v2")
    # Future docking-pose queries can read these without re-parsing YAML.
    charger_prim.SetCustomDataByKey("faceplate_offset_x", float(_FACEPLATE_OFFSET_X))
    charger_prim.SetCustomDataByKey("body_depth", float(_BODY_DEPTH))
    charger_prim.SetCustomDataByKey("body_width", float(_BODY_WIDTH))
    charger_prim.SetCustomDataByKey("body_height", float(_BODY_HEIGHT))

    # -- Materials (idempotent on /World/Materials, shared across chargers).
    body_mat = _define_preview_material(
        stage, "/World/Materials/ChargerBodyMat", Gf.Vec3f(0.92, 0.92, 0.92)
    )
    window_mat = _define_preview_material(
        stage, "/World/Materials/ChargerWindowMat", Gf.Vec3f(0.08, 0.08, 0.10)
    )
    electrode_mat = _define_preview_material(
        stage, "/World/Materials/ChargerElectrodeMat", Gf.Vec3f(0.78, 0.50, 0.25)
    )

    # -- Enclosure subassembly --
    UsdGeom.Xform.Define(stage, f"{charger_path}/enclosure")

    def _static_pair(path: str, translate, size, vis_material):
        """Visual + collision twin (auto_door pattern). No RigidBodyAPI."""
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
        _add_box(stage, f"{path}/visual", size=size,
                 material=vis_material, collision=False)
        _add_box(stage, f"{path}/collision", size=size,
                 material=None, collision=True)

    def _decorative(path: str, translate, size, vis_material):
        """Visual-only sibling (no collider)."""
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
        _add_box(stage, f"{path}/visual", size=size,
                 material=vis_material, collision=False)

    # Body: single white enclosure cube. Centre at z = body_height/2.
    _static_pair(
        f"{charger_path}/enclosure/body",
        translate=(0.0, 0.0, _BODY_HEIGHT / 2.0),
        size=(_BODY_DEPTH, _BODY_WIDTH, _BODY_HEIGHT),
        vis_material=body_mat,
    )

    # -- Faceplate subassembly: translated to sit flush on body's +X face.
    faceplate_path = f"{charger_path}/faceplate"
    faceplate = UsdGeom.Xform.Define(stage, faceplate_path)
    faceplate.AddTranslateOp().Set(Gf.Vec3d(_FACEPLATE_OFFSET_X, 0.0, 0.0))

    # Faceplate slab: visual + collider so the AMR rear bumps it.
    _static_pair(
        f"{faceplate_path}/plate",
        translate=(0.0, 0.0, _PLATE_HEIGHT / 2.0),
        size=(_PLATE_THICKNESS, _PLATE_WIDTH, _PLATE_HEIGHT),
        vis_material=body_mat,
    )

    # Dark window panel = high-contrast visual landmark for vision-based
    # dock detection. Sits proud of the faceplate so it reads cleanly from
    # any approach angle.
    _decorative(
        f"{faceplate_path}/window",
        translate=(_WINDOW_PROUD, 0.0, _WINDOW_Z),
        size=(_WINDOW_THICKNESS, _WINDOW_WIDTH, _WINDOW_HEIGHT),
        vis_material=window_mat,
    )

    # -- Approach pad: white floor plate with 3 copper electrode strips
    # running along the docking direction (X). The AMR reverses over the
    # pad and contacts from below. No collider so the robot drives over.
    if enable_approach_pad:
        body_outer_x = _BODY_DEPTH / 2.0
        pad_x_start = body_outer_x + _PAD_GAP
        pad_x_center = pad_x_start + _PAD_DEPTH / 2.0
        pad_z_center = _PAD_THICKNESS / 2.0
        electrode_z = _PAD_THICKNESS + _ELECTRODE_THICKNESS / 2.0 + _ELECTRODE_PROUD

        pad_root = f"{charger_path}/approach_pad"
        UsdGeom.Xform.Define(stage, pad_root)

        # White base plate.
        _decorative(
            f"{pad_root}/base",
            translate=(pad_x_center, 0.0, pad_z_center),
            size=(_PAD_DEPTH, _PAD_WIDTH, _PAD_THICKNESS),
            vis_material=body_mat,
        )

        # 3 copper electrode strips, parallel along X, clustered on the -Y
        # (right) half of the pad. Centre-to-centre spacing = _ELECTRODE_PITCH.
        for idx, k in enumerate((-1, 0, +1), start=1):
            y_off = _ELECTRODE_GROUP_CENTER_Y + k * _ELECTRODE_PITCH
            _decorative(
                f"{pad_root}/electrode_{idx}",
                translate=(pad_x_center, y_off, electrode_z),
                size=(_ELECTRODE_LENGTH, _ELECTRODE_WIDTH, _ELECTRODE_THICKNESS),
                vis_material=electrode_mat,
            )

    return charger_path
