"""Drop-zone registry. customData under /World/DropZones/<id>; run_sim.py
walks this hierarchy at startup, the same way it walks /World/Conveyors /
/World/Doors, and forwards each zone's tuple into the conveyor ScriptNode
so it can validate incoming /cargo/drop_cmd messages.

Schema (per child Xform):
    customData:
        drop_zone_id : string
        position_xy  : Gf.Vec2f      # zone center for radius check
        radius       : float
        drop_pose    : Gf.Vec3f      # world XYZ box snaps to before release

Optional visual: when a zone provides `boundary` (a list of 4 world-XY
corners), this builder also authors a thin colored line outline on the
ground under /World/DropZones/<id>/Boundary so the area is visible in the
viewport. Pure cosmetic -- no collision, no physics; the boundary does
NOT affect the radius check (that still uses position_xy + radius).

Pure pxr / usd-core; no Isaac Sim runtime needed.
"""
from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


def _define_boundary_material(stage: Usd.Stage, mat_path: str) -> UsdShade.Material:
    """Yellow preview material for the line outline. Idempotent: returns
    the existing material if already authored."""
    existing = stage.GetPrimAtPath(mat_path)
    if existing and existing.IsValid():
        return UsdShade.Material(existing)
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 0.85, 0.10)  # warm yellow
    )
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.6, 0.5, 0.0)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    mat.CreateSurfaceOutput().ConnectToSource(
        UsdShade.ConnectableAPI(shader), "surface"
    )
    return mat


def _build_boundary_outline(
    stage: Usd.Stage,
    parent_path: str,
    corners: list,
    thickness: float = 0.10,
    height: float = 0.02,
    z_lift: float = 0.005,
) -> list[str]:
    """Author 4 thin cubes (one per edge) tracing the rectangle defined by
    `corners` (4 world-XY pairs in order, no duplicate close vertex). Cubes
    sit just above the ground so they don't z-fight with the floor mesh.

    Order matters for the AddTranslate/AddScale pair (project rule from
    feedback_usd_op_order memory): translate first, then scale. The cube
    primitive has unit extents [-0.5, +0.5] so scale by (length, thickness,
    height) gives an edge of size length x thickness x height centred on
    the translate point.
    """
    if len(corners) != 4:
        raise ValueError(
            f"boundary expects 4 corners, got {len(corners)} for {parent_path}"
        )
    boundary_root = f"{parent_path}/Boundary"
    UsdGeom.Xform.Define(stage, boundary_root)

    # Define / fetch the shared yellow material.
    mat_path = "/World/Materials/DropZoneBoundaryMat"
    mat = _define_boundary_material(stage, mat_path)

    paths: list[str] = []
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    for idx, (a, b) in enumerate(edges):
        x1, y1 = float(corners[a][0]), float(corners[a][1])
        x2, y2 = float(corners[b][0]), float(corners[b][1])
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        # Yaw of the edge in the XY plane.
        import math
        yaw_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))

        edge_path = f"{boundary_root}/edge_{idx}"
        cube = UsdGeom.Cube.Define(stage, edge_path)
        cube.CreateSizeAttr(1.0)
        cube_prim = cube.GetPrim()
        # Translate (centre of the edge, half a thickness above the floor),
        # then yaw to align with the edge direction, then scale to a thin
        # plank: x = length, y = thickness (across the edge), z = height.
        UsdGeom.Xformable(cube).AddTranslateOp().Set(
            Gf.Vec3d(mid_x, mid_y, z_lift + height / 2.0)
        )
        UsdGeom.Xformable(cube).AddRotateZOp().Set(yaw_deg)
        UsdGeom.Xformable(cube).AddScaleOp().Set(
            Gf.Vec3f(float(length), float(thickness), float(height))
        )
        UsdShade.MaterialBindingAPI(cube_prim).Bind(mat)
        paths.append(edge_path)
    return paths


def build_drop_zones(stage: Usd.Stage, configs: list | None) -> list[str]:
    """Author /World/DropZones/<id> Xforms with customData for each entry.

    Args:
        stage: Live USD stage.
        configs: List of dicts with keys:
            id (required)         : unique zone id; becomes prim name and
                                    the payload value of /cargo/drop_cmd.
            position (required)   : [x, y] world-XY of the zone center.
            radius (optional)     : tolerance for "robot is in zone";
                                    defaults to 1.0 metre.
            drop_pose (optional)  : [x, y, z] world pose the box is
                                    teleported to at the moment of release.
                                    Defaults to [position.x, position.y, 0.5].
            boundary (optional)   : 4 [x, y] world corners (in order) to
                                    draw a yellow line outline on the
                                    ground under /World/DropZones/<id>/
                                    Boundary. Pure visual marker; does NOT
                                    affect the radius check.

    Returns the list of authored prim paths.
    """
    paths: list[str] = []
    if not configs:
        return paths

    root_path = "/World/DropZones"
    if not stage.GetPrimAtPath(root_path).IsValid():
        UsdGeom.Xform.Define(stage, root_path)

    for cfg in configs:
        try:
            zone_id = cfg["id"]
        except KeyError as exc:
            raise KeyError(
                "drop_zone config missing required 'id' (used to build the "
                "prim path /World/DropZones/<id> and matched against the "
                "payload of /cargo/drop_cmd)"
            ) from exc
        try:
            position = cfg["position"]
        except KeyError as exc:
            raise KeyError(
                f"drop_zone id={zone_id!r} missing required 'position' "
                f"([x, y] world-XY of the zone center)"
            ) from exc
        radius = float(cfg.get("radius", 1.0))
        drop_pose = cfg.get(
            "drop_pose",
            [float(position[0]), float(position[1]), 0.5],
        )

        prim_path = f"{root_path}/{zone_id}"
        xform = UsdGeom.Xform.Define(stage, prim_path)
        prim = xform.GetPrim()
        prim.SetCustomDataByKey("drop_zone_id", str(zone_id))
        prim.SetCustomDataByKey(
            "position_xy",
            Gf.Vec2f(float(position[0]), float(position[1])),
        )
        prim.SetCustomDataByKey("radius", radius)
        prim.SetCustomDataByKey(
            "drop_pose",
            Gf.Vec3f(
                float(drop_pose[0]),
                float(drop_pose[1]),
                float(drop_pose[2]),
            ),
        )

        boundary = cfg.get("boundary")
        if boundary:
            _build_boundary_outline(stage, prim_path, boundary)

        paths.append(prim_path)

    return paths
