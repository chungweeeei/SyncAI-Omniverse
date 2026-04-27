import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from syncai_omniverse.usd.stl_to_usd import build_warehouse
from syncai_omniverse.usd.mir_amr import build_mir_amr
from syncai_omniverse.usd.auto_door import build_auto_door
from syncai_omniverse.usd.charging_station import build_charging_station


# Registry of available robot model builders. `robot.model` in sim_config.yaml
# selects which one runs. Add entries here when adding a new AMR module.
_BUILDERS = {
    "mir250": build_mir_amr,
}


def _build_robot(stage, robot_cfg: dict) -> str:
    """Dispatch to the correct robot builder based on `robot_cfg["model"]`."""
    model = robot_cfg.get("model", "mir250")
    if model not in _BUILDERS:
        raise ValueError(
            f"Unknown robot.model={model!r}; expected one of {sorted(_BUILDERS)}"
        )
    return _BUILDERS[model](stage, robot_cfg)


def _resolve_robots(config: dict) -> list:
    """Return list of enabled robot configs.

    Prefers plural `robots:` (list); falls back to legacy singular `robot:`
    (dict) so existing single-robot configs keep working.
    """
    robots = config.get("robots")
    if robots is None:
        legacy = config.get("robot")
        robots = [legacy] if legacy else []
    return [r for r in robots if r and r.get("enabled", True)]


def build_combined_scene(output_path: str, stl_path: str, config: dict) -> str:
    """
        Compose warehouse (from STL) and robot onto a single USD stage.

        `config["stl"]`: warehouse build options (scale, center_xy, ground_plane_size).
        `config["robot"]` (optional): robot build options
            {enabled: bool, model: "mir250", robot_name: str,
             spawn_position: [x, y, z], mass: float}.
            Defaults to enabled=True, model=mir250.
    """
    stage = _new_stage(output_path)
    build_warehouse(stage, stl_path, config["stl"])

    for robot_cfg in _resolve_robots(config):
        _build_robot(stage, robot_cfg)

    for door_cfg in config.get("doors") or []:
        build_auto_door(stage, door_cfg)

    for station_cfg in config.get("charging_stations") or []:
        build_charging_station(stage, station_cfg)

    stage.GetRootLayer().Save()
    return output_path


def build_robot_only_scene(output_path: str, config: dict) -> str:
    """
        Build a diagnostic scene: flat ground plane + robot, no warehouse STL.

        Used to rule out collisions/contacts introduced by the warehouse mesh
        when debugging robot motion issues (e.g. wheels spin but chassis
        doesn't translate).

        `config["stl"].ground_plane_size` (optional) -- [x, y] ground size.
        `config["robot"]` -- see `build_combined_scene`.
    """
    stage = _new_stage(output_path)

    ground_size = (config.get("stl") or {}).get("ground_plane_size", [20.0, 20.0])
    _build_ground_plane(stage, ground_size)

    for robot_cfg in _resolve_robots(config):
        _build_robot(stage, robot_cfg)

    for door_cfg in config.get("doors") or []:
        build_auto_door(stage, door_cfg)

    for station_cfg in config.get("charging_stations") or []:
        build_charging_station(stage, station_cfg)

    stage.GetRootLayer().Save()
    return output_path


def _new_stage(output_path: str) -> Usd.Stage:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        os.remove(output_path)

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))
    return stage


def _build_ground_plane(stage: Usd.Stage, ground_size) -> None:
    """Ground-only variant of stl_to_usd.build_warehouse's ground plane:
    same cube geometry + physics material but without the STL mesh wrapping
    it, so we can test robot dynamics in isolation.
    """
    gx, gy = ground_size
    ground = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Mesh")
    ground.CreateSizeAttr(1.0)
    # Translate-first so the 0.02m thickness centers on z=-0.01 (top at z=0).
    # Scale-first would compose as Scale*Translate*p, shrinking the offset by
    # scale_z=0.02 and raising the ground top to ~+0.01m -- enough to hover
    # a TurtleBot3 on an invisible ledge a full centimeter above the visual
    # surface, which is how this bug originally surfaced.
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    ground.AddScaleOp().Set(Gf.Vec3f(gx, gy, 0.02))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    phys_mat = UsdShade.Material.Define(stage, "/World/Materials/GroundPhysMat")
    mat_api = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
    mat_api.CreateStaticFrictionAttr().Set(0.8)
    mat_api.CreateDynamicFrictionAttr().Set(0.6)
    mat_api.CreateRestitutionAttr().Set(0.0)
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(
        phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    vis_mat = UsdShade.Material.Define(stage, "/World/Materials/GroundMat")
    shader = UsdShade.Shader.Define(stage, "/World/Materials/GroundMat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.4, 0.4, 0.4)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    vis_mat.CreateSurfaceOutput().ConnectToSource(
        UsdShade.ConnectableAPI(shader), "surface"
    )
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(vis_mat)
