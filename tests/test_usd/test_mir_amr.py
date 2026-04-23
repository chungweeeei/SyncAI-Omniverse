"""Smoke test: build_mir_amr authors a well-formed articulation with all
physics invariants the sim depends on.
"""
import pytest

pxr = pytest.importorskip("pxr")
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from syncai_omniverse.usd.mir_amr import build_mir_amr


@pytest.fixture
def stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    world = UsdGeom.Xform.Define(s, "/World")
    s.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(s, "/World/PhysicsScene")
    return s


def test_articulation_root_on_base_link_not_parent(stage):
    """Memory rule: ArticulationRootAPI MUST be on a RigidBody, not on a
    parent Xform -- else PhysX creates a fixed-base articulation and wheels
    spin while the chassis stays pinned.
    """
    robot_path = build_mir_amr(stage)
    parent = stage.GetPrimAtPath(robot_path)
    base_link = stage.GetPrimAtPath(f"{robot_path}/base_link")

    assert not parent.HasAPI(UsdPhysics.ArticulationRootAPI), (
        "ArticulationRootAPI must not be on the parent Xform"
    )
    assert base_link.HasAPI(UsdPhysics.ArticulationRootAPI), (
        "ArticulationRootAPI must be applied to base_link"
    )
    assert base_link.HasAPI(UsdPhysics.RigidBodyAPI), (
        "base_link must carry RigidBodyAPI (floating-base articulation)"
    )


def test_drive_wheels_use_sphere_colliders(stage):
    """Memory rule: wheel colliders must be spheres, not cylinders. A thin
    cylinder collider gets convex-faceted by PhysX and locks reverse slip.
    """
    robot_path = build_mir_amr(stage)
    for wheel in ("drivewhl_l_link", "drivewhl_r_link"):
        coll = stage.GetPrimAtPath(f"{robot_path}/{wheel}/collision")
        assert coll.IsValid(), f"{wheel}/collision not authored"
        assert coll.IsA(UsdGeom.Sphere), (
            f"{wheel}/collision should be a Sphere, got {coll.GetTypeName()}"
        )
        assert coll.HasAPI(UsdPhysics.CollisionAPI), (
            f"{wheel}/collision missing CollisionAPI"
        )


def test_casters_bound_to_frictionless_material(stage):
    """Memory rule: caster colliders must bind a frictionless physics
    material with frictionCombineMode="min". Without it, mu=0 combines to
    mu=0.4 against the mu=0.8 ground and stalls the chassis.

    All 4 corner casters (matching real MiR250 layout) must bind the same
    FrictionlessPhys material.
    """
    robot_path = build_mir_amr(stage)
    four_casters = ("caster_fl_link", "caster_fr_link",
                    "caster_rl_link", "caster_rr_link")
    for caster in four_casters:
        coll = stage.GetPrimAtPath(f"{robot_path}/{caster}/collision")
        assert coll.IsValid(), f"{caster}/collision not authored"
        binding = UsdShade.MaterialBindingAPI(coll).GetDirectBindingRel("physics")
        targets = binding.GetTargets()
        assert targets, f"{caster}/collision has no physics material binding"
        mat_path = str(targets[0])
        assert mat_path.endswith("FrictionlessPhys"), (
            f"{caster} should bind FrictionlessPhys, got {mat_path}"
        )

    mat = stage.GetPrimAtPath("/World/Materials/FrictionlessPhys")
    combine = mat.GetAttribute("physxMaterial:frictionCombineMode").Get()
    assert combine == "min", (
        f"FrictionlessPhys must have frictionCombineMode=min, got {combine!r}"
    )


def test_four_casters_at_chassis_corners(stage):
    """All 4 corner casters exist at symmetric chassis corners (+-x, +-y)."""
    robot_path = build_mir_amr(stage)

    def _translate(prim):
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op.Get()
        return None

    positions = {}
    for name in ("caster_fl_link", "caster_fr_link",
                 "caster_rl_link", "caster_rr_link"):
        prim = stage.GetPrimAtPath(f"{robot_path}/{name}")
        assert prim.IsValid(), f"{name} missing"
        positions[name] = _translate(prim)

    # All 4 at same z (symmetric support plane).
    zs = {p[2] for p in positions.values()}
    assert len(zs) == 1, f"caster z heights should match, got {zs}"

    # Signs: fl=(+,+), fr=(+,-), rl=(-,+), rr=(-,-)
    assert positions["caster_fl_link"][0] > 0 and positions["caster_fl_link"][1] > 0
    assert positions["caster_fr_link"][0] > 0 and positions["caster_fr_link"][1] < 0
    assert positions["caster_rl_link"][0] < 0 and positions["caster_rl_link"][1] > 0
    assert positions["caster_rr_link"][0] < 0 and positions["caster_rr_link"][1] < 0


def test_dual_diagonal_lidar_mount_points(stage):
    """Two lidar mount points must exist for the dual-diagonal layout, on
    opposite corners of the chassis top.
    """
    robot_path = build_mir_amr(stage)
    front = stage.GetPrimAtPath(f"{robot_path}/lidar_link_front")
    rear = stage.GetPrimAtPath(f"{robot_path}/lidar_link_rear")
    assert front.IsValid() and rear.IsValid()

    def _translate(prim):
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op.Get()
        return None

    f, r = _translate(front), _translate(rear)
    # Diagonally opposite: sign of x and y should flip.
    assert f[0] * r[0] < 0 and f[1] * r[1] < 0, (
        f"lidar mount points should be diagonal; got front={f} rear={r}"
    )


def test_wheel_drive_gains_configurable(stage):
    """MiR chassis is ~10x heavier than SlotCar; drive gains must scale up."""
    robot_path = build_mir_amr(stage, {
        "wheel_drive_damping": 800.0,
        "wheel_drive_max_force": 40.0,
    })
    for joint_name in ("drivewhl_l_joint", "drivewhl_r_joint"):
        joint = stage.GetPrimAtPath(f"{robot_path}/joints/{joint_name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        assert drive, f"{joint_name} missing angular DriveAPI"
        assert drive.GetDampingAttr().Get() == pytest.approx(800.0)
        assert drive.GetMaxForceAttr().Get() == pytest.approx(40.0)
        # Velocity mode requires stiffness=0.
        assert drive.GetStiffnessAttr().Get() == pytest.approx(0.0)


def test_mass_parameter_respected(stage):
    robot_path = build_mir_amr(stage, {"mass": 123.0})
    base_link = stage.GetPrimAtPath(f"{robot_path}/base_link")
    mass_api = UsdPhysics.MassAPI(base_link)
    assert mass_api.GetMassAttr().Get() == pytest.approx(123.0)


def test_load_plate_is_visual_only(stage):
    """Load plate must not carry a collider (RTX lidars sit above it; a
    collider would block the raycasts and also add a 4th contact point that
    competes with the diagonal casters)."""
    robot_path = build_mir_amr(stage)
    plate_visual = stage.GetPrimAtPath(f"{robot_path}/base_link/load_plate/visual")
    assert plate_visual.IsValid()
    assert not plate_visual.HasAPI(UsdPhysics.CollisionAPI), (
        "load_plate/visual must not be a collider"
    )
