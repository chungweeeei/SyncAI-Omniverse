"""MiR250-style AMR articulation.

Silhouette: ~0.80 x 0.58 x 0.30 m chassis with a thin cosmetic load plate
on top, two centered differential-drive wheels, two diagonal frictionless
casters (front-left + rear-right), and two diagonal lidar mount points
(front-left + rear-right) matching real MiR safety-lidar layout.

Physics design respects the AMR gotchas encoded in `_amr_common.py`:
sphere-collider wheels, ArticulationRootAPI on base_link (a RigidBody),
frictionless casters with frictionCombineMode="min". Drive tuning is
scaled up from SlotCar (damping 200 -> 800, maxForce 8 -> 40 N*m) to
support the ~10x heavier chassis without stalling on startup.
"""
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from syncai_omniverse.usd._amr_common import (
    _add_box,
    _add_cylinder,
    _add_sphere,
    _apply_mass,
    _define_preview_material,
    _define_rigid_link,
    _fixed_joint,
    _revolute_joint,
    define_shared_materials,
)


def build_mir_amr(stage: Usd.Stage, config: dict | None = None) -> str:
    """Author a MiR250-style AMR onto an existing stage.

    Links: base_link, base_footprint, drivewhl_l/r_link, caster_fl/rr_link,
           lidar_link_front, lidar_link_rear.
    Joints: base_joint (fixed), drivewhl_l/r_joint (revolute Y, velocity drive),
            caster_fl/rr_joint (fixed), lidar_joint_front/rear (fixed).

    Config keys:
        robot_name: str -- defaults to "MirAMR"
        spawn_position: [x, y, z] -- defaults to [0, 0, 0.20]
        mass: float -- base_link mass in kg, defaults to 100.0
        wheel_drive_damping: float -- defaults to 800.0
        wheel_drive_max_force: float -- defaults to 40.0 N*m

    Returns the robot prim path.
    """
    config = config or {}
    robot_name = config.get("robot_name", "MirAMR")
    spawn_position = config.get("spawn_position", [0.0, 0.0, 0.20])
    base_mass = float(config.get("mass", 100.0))
    drive_damping = float(config.get("wheel_drive_damping", 800.0))
    drive_max_force = float(config.get("wheel_drive_max_force", 40.0))

    # -- Preview materials --
    body_mat = _define_preview_material(
        stage, "/World/Materials/MirBodyMat", Gf.Vec3f(0.85, 0.85, 0.85)
    )
    plate_mat = _define_preview_material(
        stage, "/World/Materials/MirPlateMat", Gf.Vec3f(0.10, 0.30, 0.60)
    )
    wheel_mat = _define_preview_material(
        stage, "/World/Materials/WheelMat", Gf.Vec3f(0.2, 0.2, 0.2)
    )
    caster_mat = _define_preview_material(
        stage, "/World/Materials/CasterMat", Gf.Vec3f(0.5, 0.5, 0.5)
    )
    # Shared physics materials (WheelPhys + FrictionlessPhys).
    wheel_phys, frictionless = define_shared_materials(stage)

    # -- Robot root Xform (pose carrier only, NOT the articulation root) --
    robot_path = f"/World/{robot_name}"
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(*spawn_position))

    # -- base_link : box 0.80 x 0.58 x 0.30 (articulation root / floating base) --
    base_link = _define_rigid_link(stage, f"{robot_path}/base_link", translate=(0.0, 0.0, 0.0))
    UsdPhysics.ArticulationRootAPI.Apply(base_link.GetPrim())
    # Linear / angular damping: same values as SlotCar. They turn the chassis
    # into a first-order system that absorbs the torque step from
    # DifferentialController velocity ramps, flattening the pitch transient.
    for attr_name, value in [
        ("physxRigidBody:linearDamping", 0.5),
        ("physxRigidBody:angularDamping", 5.0),
    ]:
        base_link.GetPrim().CreateAttribute(
            attr_name, Sdf.ValueTypeNames.Float
        ).Set(value)
    _add_box(stage, f"{robot_path}/base_link/visual",
             size=(0.80, 0.58, 0.30), material=body_mat, collision=False)
    _add_box(stage, f"{robot_path}/base_link/collision",
             size=(0.80, 0.58, 0.30), material=None, collision=True)
    UsdPhysics.MassAPI.Apply(base_link.GetPrim()).CreateMassAttr(base_mass)

    # -- load_plate : thin cosmetic top deck, visual only, no rigid body --
    # Child of base_link so the fixed-body semantics inherit. No collider so
    # the top surface won't block the RTX lidars mounted on the diagonal
    # corner stems above it.
    plate_xf = UsdGeom.Xform.Define(stage, f"{robot_path}/base_link/load_plate")
    plate_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.16))
    _add_box(stage, f"{robot_path}/base_link/load_plate/visual",
             size=(0.78, 0.56, 0.02), material=plate_mat, collision=False)

    # -- base_footprint : inertial body 0.20m below base_link (ROS footprint frame) --
    footprint = _define_rigid_link(stage, f"{robot_path}/base_footprint",
                                   translate=(0.0, 0.0, -0.20))
    _apply_mass(footprint.GetPrim(), mass=1.0, inertia=(0.1, 0.1, 0.1))

    # -- drivewhl_l_link : cylinder visual + SPHERE collider at (0, +0.2225, -0.10) --
    # Sphere collider (r=0.10): PhysX natively supports spheres so rolling
    # contact is symmetric. A flat cylinder collider would be faceted into
    # a convex hull polygon, locking reverse slip. Same lesson as SlotCar.
    left_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_l_link",
                                    translate=(0.0, 0.2225, -0.10))
    _add_cylinder(stage, f"{robot_path}/drivewhl_l_link/visual",
                  radius=0.10, height=0.05, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/drivewhl_l_link/collision",
                radius=0.10, material=None, collision=True,
                physics_material=wheel_phys)
    # Spin axis Y: Iyy = 0.5*m*r^2 = 0.01 (spin); Ixx = Izz = (1/12)*m*(3r^2+h^2) = 0.00542
    _apply_mass(left_wheel.GetPrim(), mass=2.0, inertia=(0.00542, 0.01, 0.00542))

    # -- drivewhl_r_link : mirror at (0, -0.2225, -0.10) --
    right_wheel = _define_rigid_link(stage, f"{robot_path}/drivewhl_r_link",
                                     translate=(0.0, -0.2225, -0.10))
    _add_cylinder(stage, f"{robot_path}/drivewhl_r_link/visual",
                  radius=0.10, height=0.05, rotate_xyz=(90.0, 0.0, 0.0),
                  material=wheel_mat, collision=False)
    _add_sphere(stage, f"{robot_path}/drivewhl_r_link/collision",
                radius=0.10, material=None, collision=True,
                physics_material=wheel_phys)
    _apply_mass(right_wheel.GetPrim(), mass=2.0, inertia=(0.00542, 0.01, 0.00542))

    # -- Four corner casters, all frictionless --
    # Real MiR250 has 4 swivel casters at the chassis corners. Here we fix
    # them (no vertical swivel joint) but bind FrictionlessPhys (mu=0 +
    # frictionCombineMode="min") so they only transmit vertical force. Memory
    # rule "2 wheels + 2+ fixed casters over-constrain static friction" does
    # NOT apply here because frictionless casters don't participate in the
    # lateral friction solve -- only the 2 drive wheels do. combineMode="min"
    # is required, else PhysX averages mu=0 with mu=0.8 ground -> mu_eff=0.4
    # -> chassis stalls.
    #
    # Z height is critical: caster BOTTOM must touch ground when wheel bottoms
    # do. Wheel bottom = -0.20 (center -0.10 - radius 0.10); caster bottom
    # must also be -0.20, so caster center = -0.20 + r_caster = -0.16.
    caster_positions = {
        "caster_fl_link": (0.36, 0.24, -0.16),   # front-left
        "caster_fr_link": (0.36, -0.24, -0.16),  # front-right
        "caster_rl_link": (-0.36, 0.24, -0.16),  # rear-left
        "caster_rr_link": (-0.36, -0.24, -0.16), # rear-right
    }
    for name, pos in caster_positions.items():
        link = _define_rigid_link(stage, f"{robot_path}/{name}", translate=pos)
        _add_sphere(stage, f"{robot_path}/{name}/visual",
                    radius=0.04, material=caster_mat, collision=False)
        _add_sphere(stage, f"{robot_path}/{name}/collision",
                    radius=0.04, material=None, collision=True,
                    physics_material=frictionless)
        # Solid sphere inertia: I = (2/5)*m*r^2 = 0.000192, symmetric
        _apply_mass(link.GetPrim(), mass=0.3,
                    inertia=(0.000192, 0.000192, 0.000192))

    # -- lidar_link_front : front-left mount point for RTX lidar #1 --
    # No visual / no collider: the RTX sensor's vendor model (loaded at
    # runtime by IsaacSensorCreateRtxLidar) supplies its own geometry. An
    # authored cylinder at the sensor origin causes the raycast to hit its
    # own inner wall and emit a ring of self-noise points. Same lesson as
    # SlotCar's lidar_link.
    #
    # Z height is critical: RTX raycasts against VISUAL meshes (not
    # colliders), so if the sensor origin sits inside chassis/visual or
    # load_plate/visual, every horizontal ray immediately hits a robot-
    # owned surface and produces a ring of self-points around the robot.
    # Chassis visual box extends to local z=+0.15; load_plate visual sits
    # at z=[+0.15, +0.17]. Raise lidars to z=+0.25 so horizontal rays
    # clear the chassis top by 10 cm and clear the load_plate top by 8 cm.
    lidar_front = _define_rigid_link(stage, f"{robot_path}/lidar_link_front",
                                     translate=(0.38, 0.26, 0.25))
    _apply_mass(lidar_front.GetPrim(), mass=0.1, inertia=(0.00004, 0.00004, 0.00004))

    # -- lidar_link_rear : rear-right mount point for RTX lidar #2 --
    lidar_rear = _define_rigid_link(stage, f"{robot_path}/lidar_link_rear",
                                    translate=(-0.38, -0.26, 0.25))
    _apply_mass(lidar_rear.GetPrim(), mass=0.1, inertia=(0.00004, 0.00004, 0.00004))

    # -- Joints --
    joints_path = f"{robot_path}/joints"
    UsdGeom.Scope.Define(stage, joints_path)

    _fixed_joint(stage, f"{joints_path}/base_joint",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/base_footprint",
                 local_pos0=(0.0, 0.0, -0.20), local_pos1=(0.0, 0.0, 0.0))

    _revolute_joint(stage, f"{joints_path}/drivewhl_l_joint",
                    body0=f"{robot_path}/base_link",
                    body1=f"{robot_path}/drivewhl_l_link",
                    local_pos0=(0.0, 0.2225, -0.10), local_pos1=(0.0, 0.0, 0.0),
                    axis="Y", add_drive=True,
                    drive_damping=drive_damping,
                    drive_max_force=drive_max_force)

    _revolute_joint(stage, f"{joints_path}/drivewhl_r_joint",
                    body0=f"{robot_path}/base_link",
                    body1=f"{robot_path}/drivewhl_r_link",
                    local_pos0=(0.0, -0.2225, -0.10), local_pos1=(0.0, 0.0, 0.0),
                    axis="Y", add_drive=True,
                    drive_damping=drive_damping,
                    drive_max_force=drive_max_force)

    for name, pos in caster_positions.items():
        joint_name = name.replace("_link", "_joint")
        _fixed_joint(stage, f"{joints_path}/{joint_name}",
                     body0=f"{robot_path}/base_link",
                     body1=f"{robot_path}/{name}",
                     local_pos0=pos, local_pos1=(0.0, 0.0, 0.0))

    _fixed_joint(stage, f"{joints_path}/lidar_joint_front",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/lidar_link_front",
                 local_pos0=(0.38, 0.26, 0.25), local_pos1=(0.0, 0.0, 0.0))

    _fixed_joint(stage, f"{joints_path}/lidar_joint_rear",
                 body0=f"{robot_path}/base_link",
                 body1=f"{robot_path}/lidar_link_rear",
                 local_pos0=(-0.38, -0.26, 0.25), local_pos1=(0.0, 0.0, 0.0))

    return robot_path
