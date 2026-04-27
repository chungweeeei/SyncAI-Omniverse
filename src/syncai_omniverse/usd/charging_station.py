"""MiR250-style rear-docking charging station (static prop).

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
    │   ├── body_{visual,collision}         Cube (visible / invisible+collider)
    │   ├── top_cap_{visual,collision}      Cube (yellow accent)
    │   └── base_skirt_visual               Cube (decorative, no collider)
    ├── faceplate                           Xform (translated +X to front)
    │   ├── plate_{visual,collision}        Cube (dark grey faceplate)
    │   ├── contact_left_visual             Cube (copper pad)
    │   ├── contact_right_visual            Cube (copper pad)
    │   └── led_strip_visual                Cube (emissive green)
    └── approach_pad                        Xform (small floor docking plate)
        ├── base_visual                     Cube (yellow plate)
        ├── border_front_visual             Cube (thin black edge stripe)
        ├── border_left_visual              Cube (thin black edge stripe)
        └── border_right_visual             Cube (thin black edge stripe)

Geometry is hard-coded to the real MiR Charge 24V proportions
(~0.58 W x 0.27 D x 0.35 H). YAML only exposes pose. The approach pad is
a small (0.30 x 0.58 m) floor plate at the dock face -- mirroring the
real MiR charging plate footprint, not a full landing zone. Decorative
only (no collider) so the robot drives straight over it.
"""
from pxr import Gf, Usd, UsdGeom

from syncai_omniverse.usd._amr_common import (
    _add_box,
    _define_emissive_material,
    _define_preview_material,
)


# MiR Charge 24V real-world reference dimensions (metres). Local axes:
# X = depth (faceplate normal direction), Y = width, Z = height.
_BODY_DEPTH = 0.27
_BODY_WIDTH = 0.58
_BODY_HEIGHT = 0.30
_TOP_CAP_HEIGHT = 0.05
_SKIRT_OVERHANG = 0.03   # extra X/Y over the body
_SKIRT_HEIGHT = 0.04

_PLATE_THICKNESS = 0.01
_PLATE_HEIGHT = 0.32
_PLATE_WIDTH = _BODY_WIDTH - 0.03   # slight inset from body edges
_FACEPLATE_OFFSET_X = _BODY_DEPTH / 2.0 + _PLATE_THICKNESS / 2.0  # 0.140

_CONTACT_PROUD = 0.005   # contacts/LED/AprilTag protrude this far past plate
_CONTACT_SIZE = (0.005, 0.04, 0.06)
_CONTACT_Y = 0.06         # ±Y offset of the two contact pads
_CONTACT_Z = 0.18

_LED_SIZE = (0.005, 0.40, 0.012)
_LED_Z = 0.305

# Approach pad: small floor plate matching real MiR charging-plate footprint.
# Sits flush with the dock skirt on the +X side and extends only ~0.3 m
# forward -- not a full landing zone.
_PAD_DEPTH = 0.30                # X extent away from dock face
_PAD_WIDTH = _BODY_WIDTH         # 0.58 m, matches dock body width
_PAD_THICKNESS = 0.005           # 5 mm slab; sits just above the floor
_PAD_GAP = 0.005                 # small air gap between dock skirt and pad start
_PAD_BORDER_W = 0.02             # 2 cm thin black border on three sides


def build_charging_station(stage: Usd.Stage, config: dict | None = None) -> str:
    """Author one MiR250-style rear-docking charger onto `stage`.

    Config keys:
        name: prim name under /World/ChargingStations/ (default "Charger01").
        id: logical id (required). Stamped on customData["charger_id"]; not
            used to derive any topic since this is a static prop.
        position: [x, y, z] world position of the dock base centre at floor
            (default [0, 0, 0]).
        rotation_z_deg: yaw of the dock (default 0.0). 0 = faceplate normal
            points +X; AMR reverses along -X to dock.
        approach_pad: bool (default True). When True, author a small
            yellow/black floor docking plate on the +X side. No collider --
            the robot drives straight over it.

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

    # -- Charger root: Xform with TranslateOp BEFORE RotateZOp so yaw spins
    # around the dock base centre, not around the world origin (memory rule).
    charger_xform = UsdGeom.Xform.Define(stage, charger_path)
    charger_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotation_z_deg != 0.0:
        charger_xform.AddRotateZOp().Set(rotation_z_deg)

    charger_prim = charger_xform.GetPrim()
    charger_prim.SetCustomDataByKey("charger_id", charger_id)
    charger_prim.SetCustomDataByKey("version", "v1")
    # Future docking-pose queries can read these without re-parsing YAML.
    charger_prim.SetCustomDataByKey("faceplate_offset_x", float(_FACEPLATE_OFFSET_X))
    charger_prim.SetCustomDataByKey("body_depth", float(_BODY_DEPTH))
    charger_prim.SetCustomDataByKey("body_width", float(_BODY_WIDTH))
    charger_prim.SetCustomDataByKey("body_height", float(_BODY_HEIGHT))

    # -- Materials (idempotent on /World/Materials, shared across chargers).
    body_mat = _define_preview_material(
        stage, "/World/Materials/ChargerBodyMat", Gf.Vec3f(0.20, 0.20, 0.22)
    )
    accent_mat = _define_preview_material(
        stage, "/World/Materials/ChargerAccentMat", Gf.Vec3f(0.95, 0.78, 0.10)
    )
    copper_mat = _define_preview_material(
        stage, "/World/Materials/ChargerCopperMat", Gf.Vec3f(0.72, 0.45, 0.20)
    )
    led_mat = _define_emissive_material(
        stage, "/World/Materials/ChargerLedMat",
        emissive_color=Gf.Vec3f(0.10, 1.0, 0.20),
        base_color=Gf.Vec3f(0.05, 0.20, 0.05),
    )
    pad_black_mat = _define_preview_material(
        stage, "/World/Materials/ChargerPadBlackMat", Gf.Vec3f(0.05, 0.05, 0.05)
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

    # Body: main dark-grey enclosure. Centre at z = body_height/2.
    _static_pair(
        f"{charger_path}/enclosure/body",
        translate=(0.0, 0.0, _BODY_HEIGHT / 2.0),
        size=(_BODY_DEPTH, _BODY_WIDTH, _BODY_HEIGHT),
        vis_material=body_mat,
    )
    # Top cap: yellow accent slab on top of body.
    _static_pair(
        f"{charger_path}/enclosure/top_cap",
        translate=(0.0, 0.0, _BODY_HEIGHT + _TOP_CAP_HEIGHT / 2.0),
        size=(_BODY_DEPTH, _BODY_WIDTH, _TOP_CAP_HEIGHT),
        vis_material=accent_mat,
    )
    # Base skirt: yellow flange at floor (decorative; body collider already
    # handles floor contact).
    _decorative(
        f"{charger_path}/enclosure/base_skirt",
        translate=(0.0, 0.0, _SKIRT_HEIGHT / 2.0),
        size=(_BODY_DEPTH + 2 * _SKIRT_OVERHANG,
              _BODY_WIDTH + 2 * _SKIRT_OVERHANG,
              _SKIRT_HEIGHT),
        vis_material=accent_mat,
    )

    # -- Faceplate subassembly: translated to sit flush on body's +X face.
    faceplate_path = f"{charger_path}/faceplate"
    faceplate = UsdGeom.Xform.Define(stage, faceplate_path)
    faceplate.AddTranslateOp().Set(Gf.Vec3d(_FACEPLATE_OFFSET_X, 0.0, 0.0))

    # Faceplate slab: visual + collider so the AMR rear bumps it. Wrapped in
    # a translated Xform so the slab base sits at faceplate-local z=0.
    _static_pair(
        f"{faceplate_path}/plate",
        translate=(0.0, 0.0, _PLATE_HEIGHT / 2.0),
        size=(_PLATE_THICKNESS, _PLATE_WIDTH, _PLATE_HEIGHT),
        vis_material=body_mat,
    )

    # Two copper contact pads, raised slightly proud (+X) of the plate.
    _decorative(
        f"{faceplate_path}/contact_left",
        translate=(_CONTACT_PROUD, +_CONTACT_Y, _CONTACT_Z),
        size=_CONTACT_SIZE,
        vis_material=copper_mat,
    )
    _decorative(
        f"{faceplate_path}/contact_right",
        translate=(_CONTACT_PROUD, -_CONTACT_Y, _CONTACT_Z),
        size=_CONTACT_SIZE,
        vis_material=copper_mat,
    )
    # Emissive LED status strip across the top of the faceplate.
    _decorative(
        f"{faceplate_path}/led_strip",
        translate=(_CONTACT_PROUD, 0.0, _LED_Z),
        size=_LED_SIZE,
        vis_material=led_mat,
    )

    # -- Approach pad: small yellow plate on +X side, matching the real MiR
    # charging-plate footprint (~0.30 x 0.58 m). Black border on the three
    # outer edges (front + sides; the dock-side edge merges into the skirt).
    # Border strips z-stack 0.5 mm above the base to avoid z-fighting.
    if enable_approach_pad:
        skirt_outer_x = _BODY_DEPTH / 2.0 + _SKIRT_OVERHANG
        pad_x_start = skirt_outer_x + _PAD_GAP
        pad_x_center = pad_x_start + _PAD_DEPTH / 2.0
        pad_z_center = _PAD_THICKNESS / 2.0
        border_z_center = _PAD_THICKNESS + 0.0005   # 0.5 mm above base top
        border_thickness = 0.001                     # 1 mm thin strip

        pad_root = f"{charger_path}/approach_pad"
        UsdGeom.Xform.Define(stage, pad_root)

        # Yellow base.
        _decorative(
            f"{pad_root}/base",
            translate=(pad_x_center, 0.0, pad_z_center),
            size=(_PAD_DEPTH, _PAD_WIDTH, _PAD_THICKNESS),
            vis_material=accent_mat,
        )
        # Front border (far edge from dock, runs along Y).
        front_x = pad_x_start + _PAD_DEPTH - _PAD_BORDER_W / 2.0
        _decorative(
            f"{pad_root}/border_front",
            translate=(front_x, 0.0, border_z_center),
            size=(_PAD_BORDER_W, _PAD_WIDTH, border_thickness),
            vis_material=pad_black_mat,
        )
        # Side borders (run along X). Inset by half the front-border width
        # so the three borders meet cleanly at the front corners.
        side_x_center = pad_x_center - _PAD_BORDER_W / 2.0
        side_x_extent = _PAD_DEPTH - _PAD_BORDER_W
        side_y = (_PAD_WIDTH - _PAD_BORDER_W) / 2.0
        _decorative(
            f"{pad_root}/border_left",
            translate=(side_x_center, +side_y, border_z_center),
            size=(side_x_extent, _PAD_BORDER_W, border_thickness),
            vis_material=pad_black_mat,
        )
        _decorative(
            f"{pad_root}/border_right",
            translate=(side_x_center, -side_y, border_z_center),
            size=(side_x_extent, _PAD_BORDER_W, border_thickness),
            vis_material=pad_black_mat,
        )

    return charger_path
