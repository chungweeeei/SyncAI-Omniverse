import os

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Gf, Sdf


def build_slotcar(stage: Usd.Stage, config: dict | None = None) -> str:
    """
        Author the SlotCar (sync-robot) articulation onto an existing stage.

        Links: base_link, base_footprint, drivewhl_l_link, drivewhl_r_link,
               front_caster, rear_caster, lidar_link.
        Joints: base_joint (fixed), drivewhl_l/r_joint (revolute, y-axis),
                caster_joint / rear_caster_joint / lidar_joint (fixed).

        Returns the robot prim path.
    """
    config = config or {}
    robot_name = config.get("robot_name", "SlotCar")
    spawn_position = config.get("spawn_position", [0.0, 0.0, 0.15])

    # -- Materials --
    body_mat = _define_preview_material(
        stage, "/World/Materials/BodyMat", Gf.Vec3f(0.0, 1.0, 1.0)
    )
    wheel_mat = _define_preview_material(
        stage, "/World/Materials/WheelMat", Gf.Vec3f(0.7, 0.7, 0.7)
    )
    lidar_mat = _define_preview_material(
        stage, "/World/Materials/LidarMat", Gf.Vec3f(0.1, 0.1, 0.1)
    )
    # friction_combine_mode="min" is REQUIRED. Without it PhysX falls back to
    # its default "average" combine, and μ=0 caster × μ=0.8 ground yields
    # μ_eff=0.4 — enough drag from the two fixed-joint casters (front+rear)
    # to stall forward/backward motion even though the wheels spin. "min"
    # forces the contact friction to 0, so the casters slide freely.
    frictionless = _define_physics_material(
        stage, "/World/Materials/FrictionlessPhys",
        static_friction=0.0, dynamic_friction=0.0, restitution=0.0,
        friction_combine_mode="min",
    )
    # High friction for drive wheels so they grip the floor instead of spinning.
    # PhysX combines wheel + ground friction (typically min/avg). Ground is 0.8 static
    # / 0.6 dynamic, so we set wheel materials high to give the contact decent grip.
    wheel_phys = _define_physics_material(
        stage, "/World/Materials/WheelPhys",
        static_friction=2.0, dynamic_friction=1.6, restitution=0.0,
    )

    # -- Robot root Xform (pose carrier only — NOT the articulation root) --
    # ArticulationRootAPI must live on a RigidBody to produce a floating-base
    # articulation. Applied to a plain Xform parent, Isaac Sim / PhysX treats
    # it as a FIXED-BASE articulation: wheels spin but base_link is pinned in
    # world space, so the chassis never translates. See `base_link` below.
    robot_path = f"/World/{robot_name}"
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(*spawn_position))

    # -- base_link : box 0.42 x 0.31 x 0.18 (articulation root / floating base) --
    base_link = _define_rigid_link(stage, f"{robot_path}/base_link", translate=(0.0, 0.0, 0.0))
    UsdPhysics.ArticulationRootAPI.Apply(base_link.GetPrim())
    # Linear / angular damping smooth out the acceleration profile seen by
    # the chassis. DifferentialController's accel limit only ramps velocity
    # (acceleration is still a step at ramp start/end), so the wheel drive
    # reaction still hits base_link as a force step. Damping turns base_link
    # into a first-order system, which absorbs that step and collapses the
    # pitch transient / post-step oscillation. Tuned so yaw response (0.8
    # rad/s rotate-in-place) keeps >90% of nominal torque headroom.
    # Authored raw (PhysxSchema.PhysxRigidBodyAPI fields) to keep this
    # module's pure-`pxr` dependency.
    for attr_name, value in [
        ("physxRigidBody:linearDamping", 0.5),
        ("physxRigidBody:angularDamping", 5.0),
    ]:
        base_link.GetPrim().CreateAttribute(
            attr_name, Sdf.ValueTypeNames.Float
        ).Set(value)
    _add_box(stage, f"{robot_path}/base_link/visual",
             size=(0.42, 0.31, 0.18), material=body_mat, collision=False)
    _add_box(stage, f"{robot_path}/base_link/collision",
             size=(0.42, 0.31, 0.18), material=None, collision=True)
    # Give base_link a sensible mass so wheel torque can actually push the chassis.
    # Inertia left to PhysX auto-compute from the collision box.
    UsdPhysics.MassAPI.Apply(base_link.GetPrim()).CreateMassAttr(10.0)

    # -- base_footprint : inertial body, 0.15m below base_link --
    footprint = _define_rigid_link(stage, f"{robot_path}/base_footprint",
                                   translate=(0.0, 0.0, -0.15))
    _apply_mass(footprint.GetPrim(), mass=15.0, inertia=(0.261, 0.341, 0.161))

    # -- drivewhl_l_link : cylinder visual + SPHERE collider at (0, 0.18, -0.05).
    # Sphere collider (r=0.10) is a diagnostic substitute for the thin cylinder:
    # PhysX natively supports spheres without convex-hull approximation, so
    # rolling contact is perfectly symmetric. A flat cylinder (r=0.10 × h=0.04)
    # is faceted by PhysX's convex-hull approximation into a polygonal disc,
    # which we suspect locks the wheel-ground contact against reverse slip
    # even though forward slip rolls fine.
    left_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_l_link",
                                    translate=(0.0, 0.18, -0.05))
    _add_cylinder(stage, f"{robot_path}/drivewhl_l_link/visual",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/drivewhl_l_link/collision",
                radius=0.10, material=None, collision=True,
                physics_material=wheel_phys)
    # Spin axis is Y (see revolute joint below), so Iyy is the spin inertia
    # (0.5*m*r² = 0.0025) and Ixx/Izz are the transverse inertias
    # ((1/12)*m*(3r²+h²) = 0.00132). Had these swapped earlier.
    _apply_mass(left_wheel.GetPrim(), mass=0.5, inertia=(0.00132, 0.0025, 0.00132))

    # -- drivewhl_r_link : cylinder visual + SPHERE collider at (0, -0.18, -0.05) --
    right_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_r_link",
                                     translate=(0.0, -0.18, -0.05))
    _add_cylinder(stage, f"{robot_path}/drivewhl_r_link/visual",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/drivewhl_r_link/collision",
                radius=0.10, material=None, collision=True,
                physics_material=wheel_phys)
    _apply_mass(right_wheel.GetPrim(), mass=0.5, inertia=(0.00132, 0.0025, 0.00132))

    # -- front_caster : sphere r=0.06 at (0.14, 0, -0.09), VISUAL ONLY --
    # No collider: two fixed-joint casters + two wheels all at z=0 contact
    # height is an over-constrained 4-contact system for PhysX. Observed
    # symptom: the static-friction solver locks the chassis in x/y even
    # though wheels spin at target velocity. A classic diff-drive with one
    # caster (tricycle) has a statically determinate 3-contact support.
    # Kept as an inertial body so mass distribution matches the URDF-like
    # authoring; only collision is disabled.
    front_caster = _define_rigid_link(stage, f"{robot_path}/front_caster",
                                      translate=(0.14, 0.0, -0.09))
    _add_sphere(stage, f"{robot_path}/front_caster/visual",
                radius=0.06, material=body_mat, collision=False)
    _apply_mass(front_caster.GetPrim(), mass=0.5, inertia=(0.00072, 0.00072, 0.00072))

    # -- rear_caster : sphere r=0.06, at (-0.14, 0, -0.09), frictionless --
    rear_caster = _define_rigid_link(stage, f"{robot_path}/rear_caster",
                                     translate=(-0.14, 0.0, -0.09))
    _add_sphere(stage, f"{robot_path}/rear_caster/visual",
                radius=0.06, material=body_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/rear_caster/collision",
                radius=0.06, material=None, collision=True,
                physics_material=frictionless)
    _apply_mass(rear_caster.GetPrim(), mass=0.5, inertia=(0.00072, 0.00072, 0.00072))

    # -- lidar_link : pure inertial body at (0, 0, 0.20) --
    # No visual / no collider authored here: `IsaacSensorCreateRtxLidar`
    # (called from `attach_lidar_publisher` at runtime) loads its own USD
    # visual for the chosen model (e.g. SICK picoScan150 mesh) as a child
    # of this prim. Our previous authoring added a r=0.04m cylinder at
    # the sensor's exact origin — RTX raycasting hit the cylinder's inner
    # wall every scan and produced a ring of self-noise points around the
    # robot at ~4 cm range. Removing it leaves the sensor's own vendor
    # model (which is designed not to self-occlude) as the only geometry
    # at the lidar position.
    lidar_link = _define_rigid_link(stage, f"{robot_path}/lidar_link",
                                    translate=(0.0, 0.0, 0.20))
    _apply_mass(lidar_link.GetPrim(), mass=0.1, inertia=(0.000041, 0.000041, 0.00008))

    # -- Joints --
    joints_path = f"{robot_path}/joints"
    UsdGeom.Scope.Define(stage, joints_path)

    _fixed_joint(stage, f"{joints_path}/base_joint",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/base_footprint",
                 local_pos0=(0.0, 0.0, -0.15), local_pos1=(0.0, 0.0, 0.0))

    _revolute_joint(stage, f"{joints_path}/drivewhl_l_joint",
                    body0=f"{robot_path}/base_link",
                    body1=f"{robot_path}/drivewhl_l_link",
                    local_pos0=(0.0, 0.18, -0.05), local_pos1=(0.0, 0.0, 0.0),
                    axis="Y", add_drive=True)

    _revolute_joint(stage, f"{joints_path}/drivewhl_r_joint",
                    body0=f"{robot_path}/base_link",
                    body1=f"{robot_path}/drivewhl_r_link",
                    local_pos0=(0.0, -0.18, -0.05), local_pos1=(0.0, 0.0, 0.0),
                    axis="Y", add_drive=True)

    _fixed_joint(stage, f"{joints_path}/caster_joint",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/front_caster",
                 local_pos0=(0.14, 0.0, -0.09), local_pos1=(0.0, 0.0, 0.0))

    _fixed_joint(stage, f"{joints_path}/rear_caster_joint",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/rear_caster",
                 local_pos0=(-0.14, 0.0, -0.09), local_pos1=(0.0, 0.0, 0.0))

    _fixed_joint(stage, f"{joints_path}/lidar_joint",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/lidar_link",
                 local_pos0=(0.0, 0.0, 0.20), local_pos1=(0.0, 0.0, 0.0))

    return robot_path


def slotcar_to_usd(output_path: str, config: dict | None = None) -> str:
    """
        Standalone wrapper: create a fresh stage with /World, PhysicsScene, and Light,
        author the SlotCar articulation, and save to `output_path`.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        os.remove(output_path)

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

    build_slotcar(stage, config)

    stage.GetRootLayer().Save()
    return output_path


# ---------- Helpers ----------

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
        # damping=200 is intentionally moderate: with a velocity ramp from
        # DifferentialController.maxAcceleration (0.5 m/s^2 ⇒ ~0.08 rad/s
        # vel_err per 60Hz step), drive torque = 200 * 0.08 = 16 N·m. That
        # then gets clamped by maxForce. We keep maxForce low (8 N·m ≈ 80 N
        # of traction per wheel) so the peak chassis reaction during big
        # target-velocity jumps (FWD→STOP→BWD transitions) is <=160 N total
        # instead of 300 N with the old 15 N·m cap. Net effect: visibly
        # smaller pitch transient without sacrificing steady-state tracking
        # (steady-state torque is ~1 N·m to hold ~0.3 m/s against rolling
        # resistance — well under either cap).
        #
        # Going lower than 6 N·m starts to miss startup friction. Going
        # higher than 12 N·m brings pitch spikes back.
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr(drive_type)
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(drive_damping)
        drive.CreateMaxForceAttr(drive_max_force)
        drive.CreateTargetVelocityAttr(0.0)
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
