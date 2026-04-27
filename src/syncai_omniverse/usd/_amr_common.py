"""Shared USD authoring helpers for AMR articulations.

Pure `pxr` / usd-core only -- no Isaac Sim runtime imports. These helpers
encode the PhysX / USD gotchas that every AMR model in this repo must
respect (sphere-collider wheels, AddTranslateOp-before-AddScaleOp,
frictionCombineMode="min", velocity-drive damping/maxForce defaults).
Keep them in one place so the memory rules don't bit-rot across models.
"""
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade


def define_shared_materials(stage):
    """Create the AMR-wide wheel + frictionless physics materials (idempotent).

    Returns (wheel_phys_mat, frictionless_mat) as UsdShade.Material objects.

    The frictionless material is REQUIRED for caster colliders: without
    `frictionCombineMode="min"`, PhysX falls back to its default "average"
    combine and mu=0 caster vs mu=0.8 ground yields mu_eff=0.4 -- enough
    drag to stall the chassis even though the drive wheels spin.
    """
    wheel_phys = _define_physics_material(
        stage, "/World/Materials/WheelPhys",
        static_friction=2.0, dynamic_friction=1.6, restitution=0.0,
    )
    frictionless = _define_physics_material(
        stage, "/World/Materials/FrictionlessPhys",
        static_friction=0.0, dynamic_friction=0.0, restitution=0.0,
        friction_combine_mode="min",
    )
    return wheel_phys, frictionless


def _define_rigid_link(stage, path: str, translate=(0.0, 0.0, 0.0)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    return xform


def _apply_mass(prim, mass: float, inertia):
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))


def _add_box(stage, path: str, size, material, collision: bool):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    if material is not None:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    if collision:
        UsdGeom.Imageable(cube).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _add_cylinder(stage, path: str, radius: float, height: float,
                  rotate_xyz, material, collision: bool,
                  physics_material=None):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(UsdGeom.Tokens.z)
    if rotate_xyz is not None:
        cyl.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
    if material is not None:
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(material)
    if collision:
        UsdGeom.Imageable(cyl).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        if physics_material is not None:
            UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(
                physics_material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
    return cyl


def _add_sphere(stage, path: str, radius: float, material, collision: bool,
                physics_material=None):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)
    if material is not None:
        UsdShade.MaterialBindingAPI(sphere.GetPrim()).Bind(material)
    if collision:
        UsdGeom.Imageable(sphere).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        if physics_material is not None:
            UsdShade.MaterialBindingAPI(sphere.GetPrim()).Bind(
                physics_material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
    return sphere


def _fixed_joint(stage, path: str, body0: str, body1: str,
                 local_pos0, local_pos1):
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    # Empty body0 -> anchored to the inertial world frame; skip SetTargets
    # because passing Sdf.Path("") errors with "Cannot map <> to layer".
    if body0:
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    return joint


def _revolute_joint(stage, path: str, body0: str, body1: str,
                    local_pos0, local_pos1, axis: str = "Z",
                    add_drive: bool = False,
                    drive_type: str = "force",
                    drive_damping: float = 200.0,
                    drive_max_force: float = 8.0):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr(axis)
    if add_drive:
        # Velocity-mode angular drive (stiffness=0, damping>0): PhysX treats
        # ArticulationController.velocityCommand as the target velocity, and
        # applies torque = damping * velocity_error, clamped by maxForce.
        #
        # Defaults here (damping=200, maxForce=8) are tuned for SlotCar's
        # 10 kg chassis. Heavier AMRs (MiR-scale, 100 kg) should pass
        # drive_damping=800, drive_max_force=40 to keep steady-state
        # tracking without stalling on startup. Pitch transient grows
        # if maxForce exceeds ~12 on the light chassis; tune together.
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr(drive_type)
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(drive_damping)
        drive.CreateMaxForceAttr(drive_max_force)
        drive.CreateTargetVelocityAttr(0.0)
    return joint


def _prismatic_joint(stage, path: str, body0: str, body1: str,
                     local_pos0, local_pos1, axis: str = "Y",
                     lower_limit: float | None = None,
                     upper_limit: float | None = None,
                     add_drive: bool = False,
                     drive_type: str = "force",
                     drive_stiffness: float = 500.0,
                     drive_damping: float = 50.0,
                     drive_max_force: float = 200.0,
                     target_position: float = 0.0):
    joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    # Empty body0 -> anchored to the inertial world frame. USD Physics
    # treats a missing body0 relationship target as "world", so we only
    # set the target when a prim path is actually supplied.
    if body0:
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr(axis)
    if lower_limit is not None:
        joint.CreateLowerLimitAttr(lower_limit)
    if upper_limit is not None:
        joint.CreateUpperLimitAttr(upper_limit)
    if add_drive:
        # Position-mode linear drive (stiffness>0, damping>0): PhysX computes
        # force = stiffness * (targetPosition - pos) + damping * (targetVel - vel),
        # clamped by maxForce. Use for doors / lifts where the joint must hold
        # a commanded pose under disturbance. Velocity mode (like wheels) sets
        # stiffness=0 and lets the controller choose vel directly -- not what
        # we want for a door that needs to stop at the jamb and stay there.
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr(drive_type)
        drive.CreateStiffnessAttr(drive_stiffness)
        drive.CreateDampingAttr(drive_damping)
        drive.CreateMaxForceAttr(drive_max_force)
        drive.CreateTargetPositionAttr(target_position)
    return joint


def _define_preview_material(stage, path: str, color: Gf.Vec3f):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    mat.CreateSurfaceOutput().ConnectToSource(
        UsdShade.ConnectableAPI(shader), "surface"
    )
    return mat


def _define_physics_material(stage, path: str, static_friction: float,
                             dynamic_friction: float, restitution: float,
                             friction_combine_mode: str | None = None):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(static_friction)
    api.CreateDynamicFrictionAttr().Set(dynamic_friction)
    api.CreateRestitutionAttr().Set(restitution)
    if friction_combine_mode is not None:
        # PhysX-specific attribute (PhysxSchema.PhysxMaterialAPI). We author
        # it raw so this module keeps its pure-`pxr` / usd-core dependency.
        # Tokens: "average" (default), "min", "multiply", "max".
        mat.GetPrim().CreateAttribute(
            "physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token
        ).Set(friction_combine_mode)
    return mat
