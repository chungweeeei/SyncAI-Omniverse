import os

from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf

from syncai_omniverse.usd.stl_to_usd import build_warehouse
from syncai_omniverse.usd.slot_car import build_slotcar


def build_combined_scene(output_path: str, stl_path: str, config: dict) -> str:
    """
        Compose warehouse (from STL) and SlotCar robot onto a single USD stage.

        `config["stl"]`: warehouse build options (scale, center_xy, ground_plane_size).
        `config["robot"]` (optional): robot build options
            {enabled: bool, robot_name: str, spawn_position: [x, y, z]}.
            Defaults to enabled=True.
    """
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

    build_warehouse(stage, stl_path, config["stl"])

    robot_cfg = config.get("robot", {}) or {}
    if robot_cfg.get("enabled", True):
        build_slotcar(stage, robot_cfg)

    stage.GetRootLayer().Save()
    return output_path
