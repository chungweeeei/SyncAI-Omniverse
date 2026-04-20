import os

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Gf, Sdf, Vt

from syncai_omniverse.helpers.stl_helper import (
    parse_stl,
    deduplicate_vertices,
    compute_bounds,
)


def build_warehouse(stage: Usd.Stage, stl_path: str, config: dict) -> None:
    """
        Author warehouse content (ground plane + building mesh + materials) onto an
        existing stage. The caller owns stage creation, `/World`, `/World/PhysicsScene`,
        lighting, and the final Save().
    """
    scale = config.get("scale", 1.0)
    center_xy = config.get("center_xy", True)
    ground_size = config.get("ground_plane_size", [20.0, 20.0])

    # -- Parse STL --
    raw_vertices, raw_normals, num_triangles = parse_stl(stl_path)

    # -- Deduplicate vertices and build index arrays --
    points, face_indices = deduplicate_vertices(raw_vertices)

    # -- Compute bounds and apply transforms --
    min_xyz, max_xyz = compute_bounds(points)

    # Center XY at origin and shift Z so floor sits at Z=0
    offset_x = (min_xyz[0] + max_xyz[0]) / 2.0 if center_xy else 0.0
    offset_y = (min_xyz[1] + max_xyz[1]) / 2.0 if center_xy else 0.0
    offset_z = min_xyz[2]

    scaled_points = Vt.Vec3fArray(len(points))
    for i, p in enumerate(points):
        scaled_points[i] = Gf.Vec3f(
            (p[0] - offset_x) * scale,
            (p[1] - offset_y) * scale,
            (p[2] - offset_z) * scale,
        )

    face_vertex_counts = Vt.IntArray(num_triangles, 3)
    face_vertex_indices = Vt.IntArray(face_indices)

    # -- Ground plane --
    gx, gy = ground_size
    ground_cube = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Mesh")
    ground_cube.CreateSizeAttr(1.0)
    ground_cube.AddScaleOp().Set(Gf.Vec3f(gx, gy, 0.02))
    ground_cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    UsdPhysics.CollisionAPI.Apply(ground_cube.GetPrim())

    ground_phys_mat = UsdShade.Material.Define(
        stage, "/World/Materials/GroundPhysMat"
    )
    mat_api = UsdPhysics.MaterialAPI.Apply(ground_phys_mat.GetPrim())
    mat_api.CreateStaticFrictionAttr().Set(0.8)
    mat_api.CreateDynamicFrictionAttr().Set(0.6)
    mat_api.CreateRestitutionAttr().Set(0.0)
    UsdShade.MaterialBindingAPI(ground_cube.GetPrim()).Bind(
        ground_phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    _assign_material(
        stage, ground_cube.GetPrim(),
        "/World/Materials/GroundMat",
        Gf.Vec3f(0.4, 0.4, 0.4),
    )

    # -- Building mesh --
    UsdGeom.Xform.Define(stage, "/World/Building")
    mesh = UsdGeom.Mesh.Define(stage, "/World/Building/Mesh")
    mesh.CreatePointsAttr(scaled_points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")

    mesh_prim = mesh.GetPrim()

    # Static collision — no RigidBodyAPI means immovable
    UsdPhysics.CollisionAPI.Apply(mesh_prim)
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    mesh_collision.CreateApproximationAttr("none")  # exact triangle mesh

    # Physics material for walls
    wall_phys_mat = UsdShade.Material.Define(
        stage, "/World/Materials/WallPhysMat"
    )
    wall_mat_api = UsdPhysics.MaterialAPI.Apply(wall_phys_mat.GetPrim())
    wall_mat_api.CreateStaticFrictionAttr().Set(0.5)
    wall_mat_api.CreateDynamicFrictionAttr().Set(0.4)
    wall_mat_api.CreateRestitutionAttr().Set(0.0)
    UsdShade.MaterialBindingAPI(mesh_prim).Bind(
        wall_phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    _assign_material(
        stage, mesh_prim,
        "/World/Materials/BuildingMat",
        Gf.Vec3f(0.75, 0.75, 0.7),
    )


def stl_to_usd(stl_path: str, output_path: str, config: dict) -> str:
    """
        Parse an STL file and build a complete USD scene with physics.

        The scene includes a ground plane, physics scene, lighting, and the STL mesh as a static collision body.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        os.remove(output_path)

    # -- Create USD stage --
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # -- Create root Xform --
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())

    # -- Physics scene --
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # -- Lighting --
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

    build_warehouse(stage, stl_path, config)

    stage.GetRootLayer().Save()
    return output_path


def _assign_material(stage, prim, mat_path: str, color: Gf.Vec3f):
    """Assign a simple UsdPreviewSurface material to a prim."""
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    mat.CreateSurfaceOutput().ConnectToSource(
        UsdShade.ConnectableAPI(shader), "surface"
    )
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
