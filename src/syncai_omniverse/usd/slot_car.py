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
    frictionless = _define_physics_material(
        stage, "/World/Materials/FrictionlessPhys",
        static_friction=0.0, dynamic_friction=0.0, restitution=0.0,
    )
    # High friction for drive wheels so they grip the floor instead of spinning.
    # PhysX combines wheel + ground friction (typically min/avg). Ground is 0.8 static
    # / 0.6 dynamic, so we set wheel materials high to give the contact decent grip.
    wheel_phys = _define_physics_material(
        stage, "/World/Materials/WheelPhys",
        static_friction=2.0, dynamic_friction=1.6, restitution=0.0,
    )

    # -- Robot root (articulation root) --
    robot_path = f"/World/{robot_name}"
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(*spawn_position))
    UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

    # -- base_link : box 0.42 x 0.31 x 0.18 --
    base_link = _define_rigid_link(stage, f"{robot_path}/base_link", translate=(0.0, 0.0, 0.0))
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

    # -- drivewhl_l_link : cylinder r=0.10 l=0.04, at (0, 0.18, -0.05) --
    left_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_l_link",
                                    translate=(0.0, 0.18, -0.05))
    _add_cylinder(stage, f"{robot_path}/drivewhl_l_link/visual",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_cylinder(stage, f"{robot_path}/drivewhl_l_link/collision",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=None, collision=True, physics_material=wheel_phys)
    # Spin axis is Y (see revolute joint below), so Iyy is the spin inertia
    # (0.5*m*r² = 0.0025) and Ixx/Izz are the transverse inertias
    # ((1/12)*m*(3r²+h²) = 0.00132). Had these swapped earlier.
    _apply_mass(left_wheel.GetPrim(), mass=0.5, inertia=(0.00132, 0.0025, 0.00132))

    # -- drivewhl_r_link : cylinder r=0.10 l=0.04, at (0, -0.18, -0.05) --
    right_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_r_link",
                                     translate=(0.0, -0.18, -0.05))
    _add_cylinder(stage, f"{robot_path}/drivewhl_r_link/visual",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_cylinder(stage, f"{robot_path}/drivewhl_r_link/collision",
                  radius=0.10, height=0.04, rotate_xyz=(90.0, 0.0, 0.0),
                  material=None, collision=True, physics_material=wheel_phys)
    _apply_mass(right_wheel.GetPrim(), mass=0.5, inertia=(0.00132, 0.0025, 0.00132))

    # -- front_caster : sphere r=0.06, at (0.14, 0, -0.09), frictionless --
    front_caster = _define_rigid_link(stage, f"{robot_path}/front_caster",
                                      translate=(0.14, 0.0, -0.09))
    _add_sphere(stage, f"{robot_path}/front_caster/visual",
                radius=0.06, material=body_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/front_caster/collision",
                radius=0.06, material=None, collision=True,
                physics_material=frictionless)
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

    # -- lidar_link : cylinder r=0.04 l=0.04, at (0, 0, 0.12) --
    lidar_link = _define_rigid_link(stage, f"{robot_path}/lidar_link",
                                    translate=(0.0, 0.0, 0.12))
    _add_cylinder(stage, f"{robot_path}/lidar_link/visual",
                  radius=0.04, height=0.04, rotate_xyz=None,
                  material=lidar_mat, collision=False)
    _add_cylinder(stage, f"{robot_path}/lidar_link/collision",
                  radius=0.04, height=0.04, rotate_xyz=None,
                  material=None, collision=True)
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
                 local_pos0=(0.0, 0.0, 0.12), local_pos1=(0.0, 0.0, 0.0))

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
                    drive_damping: float = 2000.0,
                    drive_max_force: float = 15.0):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr(axis)
    if add_drive:
        # Velocity-mode angular drive (stiffness=0, damping>>0): PhysX treats
        # ArticulationController.velocityCommand as the target velocity.
        #
        # `type="force"` applies torque = damping * velocity_error directly,
        # clamped by maxForce. Chose this over "acceleration" because the
        # latter scales torque by the joint's effective inertia (~wheel spin
        # inertia ~0.001-0.003 kg·m²) — at small velocity errors the torque
        # falls below the wheel/ground static-friction threshold (~9 N·m for
        # this 27 kg robot) and the robot refuses to start moving.
        #
        # maxForce=15 N·m is ~1.6x the friction limit: enough to break static
        # friction from any non-trivial vel_err, not so much that it launches
        # the chassis. damping=2000 means vel_err >= 0.0075 rad/s saturates
        # to maxForce — effectively on/off control that velocity-clamps once
        # the wheel catches up.
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
                             dynamic_friction: float, restitution: float):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(static_friction)
    api.CreateDynamicFrictionAttr().Set(dynamic_friction)
    api.CreateRestitutionAttr().Set(restitution)
    return mat
