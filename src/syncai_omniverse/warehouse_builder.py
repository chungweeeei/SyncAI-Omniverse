"""Programmatically build a warehouse USD scene using the pxr API."""

import os
import random

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Gf, Sdf


class WarehouseBuilder:
    def __init__(self, config: dict, output_path: str):
        self._cfg = config
        self._output_path = os.path.abspath(output_path)

    def build(self) -> str:
        os.makedirs(os.path.dirname(self._output_path), exist_ok=True)

        # CreateNew fails if file already exists — remove it first
        if os.path.exists(self._output_path):
            os.remove(self._output_path)

        # Step 1: Create a new USD Stage
        stage = Usd.Stage.CreateNew(self._output_path)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        
        # Step 2: Define the root Xform
        root = UsdGeom.Xform.Define(stage, "/World")

        # Step 3: Add scene elements
        self._add_physics_scene(stage)
        self._add_ground_plane(stage)
        self._add_walls(stage)
        self._add_shelves(stage)
        self._add_obstacle_boxes(stage)
        self._add_lighting(stage)

        stage.SetDefaultPrim(root.GetPrim())
        stage.GetRootLayer().Save()
        return self._output_path

    # ------------------------------------------------------------------ #
    #  Physics
    # ------------------------------------------------------------------ #
    def _add_physics_scene(self, stage: Usd.Stage):
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    # ------------------------------------------------------------------ #
    #  Ground
    # ------------------------------------------------------------------ #
    def _add_ground_plane(self, stage: Usd.Stage):
        gx, gy = self._cfg["ground_size"]
        xform = UsdGeom.Xform.Define(stage, "/World/GroundPlane")

        cube = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Mesh")
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(gx, gy, 0.02))
        cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))

        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # Physics material with friction for ground
        ground_phys_mat = UsdShade.Material.Define(stage, "/World/Materials/GroundPhysMat")
        mat_api = UsdPhysics.MaterialAPI.Apply(ground_phys_mat.GetPrim())
        mat_api.CreateStaticFrictionAttr().Set(0.8)
        mat_api.CreateDynamicFrictionAttr().Set(0.6)
        mat_api.CreateRestitutionAttr().Set(0.0)
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(
            ground_phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

        self._assign_material(
            stage, cube.GetPrim(),
            "/World/Materials/GroundMat",
            Gf.Vec3f(0.4, 0.4, 0.4),
        )

    # ------------------------------------------------------------------ #
    #  Walls
    # ------------------------------------------------------------------ #
    def _add_walls(self, stage: Usd.Stage):
        UsdGeom.Xform.Define(stage, "/World/Walls")
        gx, gy = self._cfg["ground_size"]
        h = self._cfg["wall_height"]
        t = self._cfg["wall_thickness"]

        walls = {
            "North": (Gf.Vec3d(0, gy / 2, h / 2), Gf.Vec3f(gx, t, h)),
            "South": (Gf.Vec3d(0, -gy / 2, h / 2), Gf.Vec3f(gx, t, h)),
            "East":  (Gf.Vec3d(gx / 2, 0, h / 2), Gf.Vec3f(t, gy, h)),
            "West":  (Gf.Vec3d(-gx / 2, 0, h / 2), Gf.Vec3f(t, gy, h)),
        }

        for name, (pos, scale) in walls.items():
            path = f"/World/Walls/{name}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(pos)
            cube.AddScaleOp().Set(scale)

            prim = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateKinematicEnabledAttr().Set(True)

            self._assign_material(
                stage, prim,
                f"/World/Materials/WallMat_{name}",
                Gf.Vec3f(0.75, 0.75, 0.7),
            )

    # ------------------------------------------------------------------ #
    #  Shelves
    # ------------------------------------------------------------------ #
    def _add_shelves(self, stage: Usd.Stage):
        UsdGeom.Xform.Define(stage, "/World/Shelves")
        rows = self._cfg["num_shelf_rows"]
        cols = self._cfg["num_shelf_cols"]
        spacing = self._cfg["shelf_spacing"]
        sh = self._cfg["shelf_height"]
        sw = self._cfg["shelf_width"]
        sd = self._cfg["shelf_depth"]

        for r in range(rows):
            for c in range(cols):
                x = (c - (cols - 1) / 2) * spacing
                y = (r - (rows - 1) / 2) * spacing
                self._create_shelf_unit(stage, r, c, x, y, sh, sw, sd)

    def _create_shelf_unit(self, stage, row, col, x, y, h, w, d):
        base = f"/World/Shelves/Shelf_{row}_{col}"
        UsdGeom.Xform.Define(stage, base)

        upright_w = 0.05
        shelf_thickness = 0.03
        num_levels = 3  # bottom + 2 shelves

        # Two vertical uprights
        for i, dx in enumerate([-w / 2 + upright_w / 2, w / 2 - upright_w / 2]):
            path = f"{base}/Upright_{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(x + dx, y, h / 2))
            cube.AddScaleOp().Set(Gf.Vec3f(upright_w, d, h))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rb.CreateKinematicEnabledAttr().Set(True)

        # Horizontal shelf boards
        for level in range(num_levels):
            z = (level / (num_levels - 1)) * h if num_levels > 1 else 0
            path = f"{base}/Board_{level}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            cube.AddScaleOp().Set(Gf.Vec3f(w, d, shelf_thickness))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rb.CreateKinematicEnabledAttr().Set(True)

        self._assign_material(
            stage,
            stage.GetPrimAtPath(f"{base}/Board_0"),
            f"/World/Materials/ShelfMat_{row}_{col}",
            Gf.Vec3f(0.55, 0.35, 0.15),
        )

    # ------------------------------------------------------------------ #
    #  Obstacle boxes
    # ------------------------------------------------------------------ #
    def _add_obstacle_boxes(self, stage: Usd.Stage):
        UsdGeom.Xform.Define(stage, "/World/Obstacles")
        gx, gy = self._cfg["ground_size"]
        n = self._cfg["num_obstacle_boxes"]
        lo, hi = self._cfg["box_size_range"]

        for i in range(n):
            size = random.uniform(lo, hi)
            # Random position avoiding 1.5 m radius around origin
            while True:
                bx = random.uniform(-gx / 2 + 1, gx / 2 - 1)
                by = random.uniform(-gy / 2 + 1, gy / 2 - 1)
                if bx * bx + by * by > 2.25:
                    break

            path = f"/World/Obstacles/Box_{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(bx, by, size / 2))
            cube.AddScaleOp().Set(Gf.Vec3f(size, size, size))

            prim = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateKinematicEnabledAttr().Set(False)
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(1.0)

            self._assign_material(
                stage, prim,
                f"/World/Materials/BoxMat_{i}",
                Gf.Vec3f(
                    random.uniform(0.3, 0.9),
                    random.uniform(0.3, 0.9),
                    random.uniform(0.2, 0.5),
                ),
            )

    # ------------------------------------------------------------------ #
    #  Lighting
    # ------------------------------------------------------------------ #
    def _add_lighting(self, stage: Usd.Stage):
        light = UsdLux.DistantLight.Define(stage, "/World/Light")
        light.CreateIntensityAttr(3000)
        light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

    # ------------------------------------------------------------------ #
    #  Material helper
    # ------------------------------------------------------------------ #
    @staticmethod
    def _assign_material(stage, prim, mat_path, color: Gf.Vec3f):
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
        mat.CreateSurfaceOutput().ConnectToSource(
            UsdShade.ConnectableAPI(shader), "surface"
        )
        UsdShade.MaterialBindingAPI(prim).Bind(mat)
