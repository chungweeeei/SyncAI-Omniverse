"""Double-leaf automatic sliding door (kinematic-leaf design).

Authored as a USD scene fragment that plugs into the existing scene
(`scene.build_combined_scene`) and is driven at runtime by the ROS2
bridge in `syncai_omniverse.ros2.door_controller`. Pure `pxr` /
usd-core, no Isaac Sim dependency.

Scene shape:
    /World/Doors/<name>                 Xform (pose carrier)
    ├── leaf_left                       RigidBody (kinematic)
    │   ├── visual                      Cube (visible, no collider)
    │   └── collision                   Cube (invisible, CollisionAPI)
    ├── leaf_right                      RigidBody (kinematic, mirror)
    │   ├── visual
    │   └── collision
    └── frame                           Xform (static colliders + visuals)
        ├── jamb_left_{visual,collision}
        ├── jamb_right_{visual,collision}
        └── lintel_{visual,collision}

Why kinematic leaves (NOT an articulation):
    Three prior designs failed end-to-end diagnostic testing:
      1. `ArticulationRootAPI` on the door root Xform + joints with
         `body0=""` (world anchor) -> PhysX excludes these joints from
         the articulation's DOF list. IsaacArticulationController throws
         KeyError(`leaf_left_joint`).
      2. No articulation + direct DriveAPI:linear.targetPosition writes
         from a ScriptNode -> USD attrs update cleanly (readback matches)
         but PhysX silently ignores drive targets on non-articulation
         joints. Leaves never move.
      3. Dynamic `base_link` RigidBody + FixedJoint-to-world +
         ArticulationRootAPI on base_link + prismatic joints base<->leaf
         -> articulation registers but drive targets still don't cause
         motion. Root cause unclear (possibly the world FixedJoint
         interferes with articulation solver).
    Kinematic leaves sidestep the entire articulation stack. Kinematic
    RigidBodies are moved by writing their xformOp:translate each tick;
    PhysX picks up the new pose via the Fabric sync and teleports the
    collider accordingly. Doors don't need realistic dynamics -- a
    kinematic body with a collider blocks the robot just as well as a
    dynamic one. This is also how real automatic doors are typically
    simulated in robotics stacks.

Leaf motion:
    Each leaf's closed-position xform is authored here. The ROS2 bridge
    (`door_controller.py`) writes the open-position translate when the
    Bool is true, and the closed-position translate when false. A small
    per-tick step-size in the bridge's ScriptNode interpolates between
    the two so the motion isn't a jarring snap.
"""
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from syncai_omniverse.usd._amr_common import (
    _add_box,
    _apply_mass,
    _define_preview_material,
    _define_rigid_link,
)


def build_auto_door(stage: Usd.Stage, config: dict | None = None) -> str:
    """Author a double-leaf sliding door onto `stage`.

    Config keys (all optional, sensible defaults):
        name: prim name under /World/Doors/ (default "EastDoor")
        position: [x, y, z] world position of the opening centre at the
            floor (default [8.0, 0.0, 0.0] -- near the +X warehouse wall)
        rotation_z_deg: yaw of the door so it aligns with the host wall
            (default 0.0 -- leaves slide along world Y, suitable for a
            wall whose normal is ±X)
        opening_width: total clear width of the opening, metres (default 2.0)
        opening_height: clear height of the opening, metres (default 2.0)
        leaf_thickness: thickness of each leaf along the wall normal
            (default 0.05)
        leaf_mass: nominal per-leaf mass in kg (default 25.0). Kinematic
            bodies don't integrate mass but PhysX warns without it.
        open_target: per-leaf slide distance when fully open (default
            opening_width/2 - 0.05 so the leaves don't fully disappear
            into the jambs).
        topic: ROS2 topic the controller bridge subscribes to
            (default "/door/<name_lower>/cmd"). Stored as USD customData
            so `run_sim.py` can discover it via a stage walk -- keeps the
            USD scene self-describing.

    Returns the door root prim path.
    """
    config = config or {}
    name = config.get("name", "EastDoor")
    position = config.get("position", [8.0, 0.0, 0.0])
    rotation_z_deg = float(config.get("rotation_z_deg", 0.0))
    opening_width = float(config.get("opening_width", 2.0))
    opening_height = float(config.get("opening_height", 2.0))
    leaf_thickness = float(config.get("leaf_thickness", 0.05))
    leaf_mass = float(config.get("leaf_mass", 25.0))
    default_open = opening_width / 2.0 - 0.05
    open_target = float(config.get("open_target", default_open))
    topic = config.get("topic", f"/door/{name.lower()}/cmd")

    # Ensure the shared container Xform exists. Multiple doors can live
    # under /World/Doors; run_sim.py walks its direct children to attach
    # a ROS2 bridge per door.
    doors_root_path = "/World/Doors"
    if not stage.GetPrimAtPath(doors_root_path).IsValid():
        UsdGeom.Xform.Define(stage, doors_root_path)

    door_path = f"{doors_root_path}/{name}"

    # -- Door root: plain Xform carrying the shared pose.
    # TranslateOp before RotateZOp so the yaw spins around the opening's
    # ground-level centre, not around the origin (memory rule on op order).
    door_xform = UsdGeom.Xform.Define(stage, door_path)
    door_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotation_z_deg != 0.0:
        door_xform.AddRotateZOp().Set(rotation_z_deg)

    # Stash the ROS2 topic + open target + closed-position y on the root
    # as customData. The runtime bridge reads these back so the scene
    # file is self-describing.
    door_prim = door_xform.GetPrim()
    door_prim.SetCustomDataByKey("ros2_topic", topic)
    door_prim.SetCustomDataByKey("open_target", open_target)

    # -- Materials --
    leaf_vis_mat = _define_preview_material(
        stage, "/World/Materials/DoorLeafMat", Gf.Vec3f(0.75, 0.80, 0.85)
    )
    frame_vis_mat = _define_preview_material(
        stage, "/World/Materials/DoorFrameMat", Gf.Vec3f(0.25, 0.25, 0.28)
    )

    # -- Geometry constants (local to the door's rotated/translated frame) --
    leaf_width = opening_width / 2.0
    # Centre each leaf halfway into its half of the opening so the two
    # leaves meet in the middle at Y=0 when closed.
    leaf_center_y = leaf_width / 2.0
    leaf_center_z = opening_height / 2.0

    jamb_thickness = 0.10
    jamb_width = jamb_thickness          # along the wall (Y)
    jamb_depth = jamb_thickness * 2.0    # across the wall (X): thick enough
                                         # to read as a doorframe visually
    jamb_center_y_abs = opening_width / 2.0 + jamb_width / 2.0
    jamb_center_z = opening_height / 2.0

    lintel_height = jamb_thickness
    lintel_width = opening_width + 2.0 * jamb_width   # spans both jambs
    lintel_center_z = opening_height + lintel_height / 2.0

    # -- Left leaf (kinematic RigidBody; the bridge will write xformOp:
    # translate each tick to slide it). Closed position at y=+leaf_center_y.
    left_path = f"{door_path}/leaf_left"
    left_leaf = _define_rigid_link(
        stage, left_path,
        translate=(0.0, leaf_center_y, leaf_center_z),
    )
    UsdPhysics.RigidBodyAPI.Get(stage, left_path).CreateKinematicEnabledAttr(True)
    _add_box(stage, f"{left_path}/visual",
             size=(leaf_thickness, leaf_width, opening_height),
             material=leaf_vis_mat, collision=False)
    _add_box(stage, f"{left_path}/collision",
             size=(leaf_thickness, leaf_width, opening_height),
             material=None, collision=True)
    # Nominal mass; kinematic bodies don't integrate so this is cosmetic.
    _apply_mass(left_leaf.GetPrim(), mass=leaf_mass,
                inertia=(1.0, 1.0, 1.0))
    # Stash the closed-position and open-position leaf-local Y on each leaf
    # prim as customData. The bridge reads these to know where to slide.
    left_leaf.GetPrim().SetCustomDataByKey("closed_y", float(leaf_center_y))
    left_leaf.GetPrim().SetCustomDataByKey("open_y", float(leaf_center_y + open_target))
    left_leaf.GetPrim().SetCustomDataByKey("leaf_z", float(leaf_center_z))

    # -- Right leaf: mirror of left, closed at y=-leaf_center_y, opens -Y.
    right_path = f"{door_path}/leaf_right"
    right_leaf = _define_rigid_link(
        stage, right_path,
        translate=(0.0, -leaf_center_y, leaf_center_z),
    )
    UsdPhysics.RigidBodyAPI.Get(stage, right_path).CreateKinematicEnabledAttr(True)
    _add_box(stage, f"{right_path}/visual",
             size=(leaf_thickness, leaf_width, opening_height),
             material=leaf_vis_mat, collision=False)
    _add_box(stage, f"{right_path}/collision",
             size=(leaf_thickness, leaf_width, opening_height),
             material=None, collision=True)
    _apply_mass(right_leaf.GetPrim(), mass=leaf_mass,
                inertia=(1.0, 1.0, 1.0))
    right_leaf.GetPrim().SetCustomDataByKey("closed_y", float(-leaf_center_y))
    right_leaf.GetPrim().SetCustomDataByKey("open_y", float(-leaf_center_y - open_target))
    right_leaf.GetPrim().SetCustomDataByKey("leaf_z", float(leaf_center_z))

    # -- Frame (static colliders + visuals, no RigidBodyAPI).
    # Each element gets its own translated Xform so the visual + collision
    # children share a local origin at the element's centre.
    UsdGeom.Xform.Define(stage, f"{door_path}/frame")

    def _static_box(path: str, translate, size):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
        _add_box(stage, f"{path}/visual", size=size,
                 material=frame_vis_mat, collision=False)
        _add_box(stage, f"{path}/collision", size=size,
                 material=None, collision=True)

    _static_box(
        f"{door_path}/frame/jamb_left",
        translate=(0.0, jamb_center_y_abs, jamb_center_z),
        size=(jamb_depth, jamb_width, opening_height),
    )
    _static_box(
        f"{door_path}/frame/jamb_right",
        translate=(0.0, -jamb_center_y_abs, jamb_center_z),
        size=(jamb_depth, jamb_width, opening_height),
    )
    _static_box(
        f"{door_path}/frame/lintel",
        translate=(0.0, 0.0, lintel_center_z),
        size=(jamb_depth, lintel_width, lintel_height),
    )

    return door_path
