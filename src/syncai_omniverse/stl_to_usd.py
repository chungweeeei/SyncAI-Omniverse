"""Convert an STL mesh file to a USD scene for Isaac Sim."""

import os
import struct

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Gf, Sdf, Vt


def stl_to_usd(stl_path: str, output_path: str, config: dict) -> str:
    """Parse an STL file and build a complete USD scene with physics.

    The scene includes a ground plane, physics scene, lighting,
    and the STL mesh as a static collision body.
    """
    scale = config.get("scale", 1.0)
    center_xy = config.get("center_xy", True)
    ground_size = config.get("ground_plane_size", [20.0, 20.0])

    # -- Parse STL --
    raw_vertices, raw_normals, num_triangles = _parse_stl(stl_path)

    # -- Deduplicate vertices and build index arrays --
    points, face_indices = _deduplicate_vertices(raw_vertices)

    # -- Compute bounds and apply transforms --
    min_xyz, max_xyz = _compute_bounds(points)

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

    # -- Create USD stage --
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        os.remove(output_path)

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())

    # -- Physics scene --
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

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

    # -- Lighting --
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

    stage.GetRootLayer().Save()
    return output_path


# ------------------------------------------------------------------ #
#  STL parsing helpers
# ------------------------------------------------------------------ #

def _parse_stl(file_path: str):
    """Parse an STL file (binary or ASCII) and return raw triangles.

    Returns:
        raw_vertices: list of (x, y, z) tuples, 3 per triangle
        raw_normals: list of (nx, ny, nz) tuples, 1 per triangle
        num_triangles: int
    """
    if _is_binary_stl(file_path):
        return _parse_binary_stl(file_path)
    return _parse_ascii_stl(file_path)


def _is_binary_stl(file_path: str) -> bool:
    """Detect whether an STL file is binary or ASCII."""
    file_size = os.path.getsize(file_path)
    with open(file_path, "rb") as f:
        header = f.read(80)
        if file_size < 84:
            return False
        num_triangles = struct.unpack("<I", f.read(4))[0]
        expected_size = 84 + num_triangles * 50
        return file_size == expected_size


def _parse_binary_stl(file_path: str):
    """Parse a binary STL file."""
    raw_vertices = []
    raw_normals = []

    with open(file_path, "rb") as f:
        f.read(80)  # skip header
        num_triangles = struct.unpack("<I", f.read(4))[0]

        for _ in range(num_triangles):
            data = struct.unpack("<12fH", f.read(50))
            nx, ny, nz = data[0], data[1], data[2]
            raw_normals.append((nx, ny, nz))

            v1 = (data[3], data[4], data[5])
            v2 = (data[6], data[7], data[8])
            v3 = (data[9], data[10], data[11])
            raw_vertices.extend([v1, v2, v3])

    return raw_vertices, raw_normals, num_triangles


def _parse_ascii_stl(file_path: str):
    """Parse an ASCII STL file."""
    raw_vertices = []
    raw_normals = []
    num_triangles = 0

    with open(file_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == "facet" and parts[1] == "normal":
                raw_normals.append(
                    (float(parts[2]), float(parts[3]), float(parts[4]))
                )
                num_triangles += 1
            elif parts[0] == "vertex":
                raw_vertices.append(
                    (float(parts[1]), float(parts[2]), float(parts[3]))
                )

    return raw_vertices, raw_normals, num_triangles


def _deduplicate_vertices(raw_vertices):
    """Deduplicate vertices and build face index array.

    Returns:
        unique_points: list of (x, y, z) tuples
        face_indices: list of int indices into unique_points
    """
    vertex_map = {}
    unique_points = []
    face_indices = []

    for v in raw_vertices:
        key = (round(v[0], 6), round(v[1], 6), round(v[2], 6))
        if key not in vertex_map:
            vertex_map[key] = len(unique_points)
            unique_points.append(v)
        face_indices.append(vertex_map[key])

    return unique_points, face_indices


def _compute_bounds(points):
    """Compute axis-aligned bounding box of a point list."""
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for x, y, z in points:
        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y
        if z < min_z:
            min_z = z
        if z > max_z:
            max_z = z

    return (min_x, min_y, min_z), (max_x, max_y, max_z)


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
