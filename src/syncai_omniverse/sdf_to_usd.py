"""Convert the SyncAI slotcar SDF model to a USD file for Isaac Sim."""

import math
import os
import xml.etree.ElementTree as ET

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf


def sdf_to_usd(sdf_path: str, output_path: str) -> str:
    """Parse the slotcar SDF and build a USD with physics."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    model_name = model.get("name", "robot")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        os.remove(output_path)

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    robot_path = f"/{model_name.replace('-', '_')}"
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    stage.SetDefaultPrim(robot_xform.GetPrim())

    # Apply ArticulationRootAPI to the robot root
    UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

    # Collect joint info first (for pose resolution)
    joint_data = {}
    for joint_el in model.findall("joint"):
        jname = joint_el.get("name")
        jtype = joint_el.get("type")
        parent_name = joint_el.findtext("parent")
        child_name = joint_el.findtext("child")
        pose_el = joint_el.find("pose")
        pose = _parse_pose(pose_el)
        axis_el = joint_el.find("axis/xyz")
        axis = _parse_vec3(axis_el.text) if axis_el is not None else (0, 0, 1)
        joint_data[jname] = {
            "type": jtype,
            "parent": parent_name,
            "child": child_name,
            "pose": pose,
            "axis": axis,
        }

    # Build links
    link_paths = {}
    for link_el in model.findall("link"):
        link_name = link_el.get("name")
        link_path = f"{robot_path}/{link_name}"
        link_paths[link_name] = link_path

        link_xform = UsdGeom.Xform.Define(stage, link_path)

        # Find pose from joint if this link is a child
        for jname, jd in joint_data.items():
            if jd["child"] == link_name:
                pos, rot = jd["pose"]
                link_xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
                if any(r != 0 for r in rot):
                    link_xform.AddRotateXYZOp().Set(
                        Gf.Vec3f(
                            math.degrees(rot[0]),
                            math.degrees(rot[1]),
                            math.degrees(rot[2]),
                        )
                    )
                break

        # Collision + RigidBody
        rb = UsdPhysics.RigidBodyAPI.Apply(link_xform.GetPrim())

        # Inertial — base_link in SDF has no mass, but it needs mass in USD
        inertial_el = link_el.find("inertial")
        mass_api = UsdPhysics.MassAPI.Apply(link_xform.GetPrim())
        if inertial_el is not None:
            mass_el = inertial_el.find("mass")
            if mass_el is not None:
                mass_api.CreateMassAttr().Set(float(mass_el.text))
        elif link_name == "base_link":
            # base_link has no inertial in SDF — give it the robot's mass
            mass_api.CreateMassAttr().Set(15.0)

        # Visuals
        for vi, vis_el in enumerate(link_el.findall("visual")):
            vis_name = vis_el.get("name", f"visual_{vi}")
            _add_geometry(
                stage, f"{link_path}/{vis_name}", vis_el, purpose="default"
            )

        # Collisions — use sphere for wheels (PhysX handles sphere rolling
        # much better than cylinder edge contact)
        is_wheel = "drivewhl" in link_name or "wheel" in link_name
        is_caster = "caster" in link_name

        if is_wheel:
            # Use sphere collision for drive wheels (radius = wheel radius)
            col_path = f"{link_path}/collision"
            sphere = UsdGeom.Sphere.Define(stage, col_path)
            sphere.CreateRadiusAttr(0.10)  # wheel radius from SDF
            col_prim = sphere.GetPrim()
            UsdPhysics.CollisionAPI.Apply(col_prim)

            mat_path = col_path + "_physics_mat"
            phys_mat = UsdShade.Material.Define(stage, mat_path)
            mat_api = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
            mat_api.CreateStaticFrictionAttr().Set(1.0)
            mat_api.CreateDynamicFrictionAttr().Set(0.8)
            mat_api.CreateRestitutionAttr().Set(0.0)
            UsdShade.MaterialBindingAPI(col_prim).Bind(
                phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
        elif is_caster:
            # Sphere collision with very low friction
            col_path = f"{link_path}/collision"
            sphere = UsdGeom.Sphere.Define(stage, col_path)
            sphere.CreateRadiusAttr(0.06)  # caster radius from SDF
            col_prim = sphere.GetPrim()
            UsdPhysics.CollisionAPI.Apply(col_prim)

            mat_path = col_path + "_physics_mat"
            phys_mat = UsdShade.Material.Define(stage, mat_path)
            mat_api = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
            mat_api.CreateStaticFrictionAttr().Set(0.001)
            mat_api.CreateDynamicFrictionAttr().Set(0.001)
            mat_api.CreateRestitutionAttr().Set(0.0)
            UsdShade.MaterialBindingAPI(col_prim).Bind(
                phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
        else:
            # Regular collision from SDF geometry
            for ci, col_el in enumerate(link_el.findall("collision")):
                col_name = col_el.get("name", f"collision_{ci}")
                col_path = f"{link_path}/{col_name}"
                _add_geometry(stage, col_path, col_el, purpose="default")
                col_prim = stage.GetPrimAtPath(col_path)
                UsdPhysics.CollisionAPI.Apply(col_prim)

                mat_path = col_path + "_physics_mat"
                phys_mat = UsdShade.Material.Define(stage, mat_path)
                mat_api = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
                mat_api.CreateStaticFrictionAttr().Set(0.5)
                mat_api.CreateDynamicFrictionAttr().Set(0.4)
                mat_api.CreateRestitutionAttr().Set(0.0)
                UsdShade.MaterialBindingAPI(col_prim).Bind(
                    phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
                )

    # Build joints
    for jname, jd in joint_data.items():
        parent_path = link_paths.get(jd["parent"])
        child_path = link_paths.get(jd["child"])
        if not parent_path or not child_path:
            continue

        joint_path = f"{robot_path}/joints/{jname}"

        if jd["type"] == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
            joint.CreateAxisAttr("Y")  # SDF axis is typically (0,1,0) for wheels

            ax = jd["axis"]
            if abs(ax[0]) > 0.5:
                joint.CreateAxisAttr("X")
            elif abs(ax[2]) > 0.5:
                joint.CreateAxisAttr("Z")

            # Unlimited rotation (no angle limits)
            joint.CreateLowerLimitAttr(-1e10)
            joint.CreateUpperLimitAttr(1e10)

            # Add drive for velocity control (high damping for enough torque)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateDampingAttr(1e5)
            drive.CreateMaxForceAttr(1e6)
            drive.CreateStiffnessAttr(0.0)

        elif jd["type"] == "fixed":
            joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        else:
            joint = UsdPhysics.Joint.Define(stage, joint_path)

        # Connect bodies
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([child_path])

        # Joint frame offset
        pos, rot = jd["pose"]
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    stage.GetRootLayer().Save()
    return output_path


def _parse_pose(pose_el):
    """Parse SDF <pose> → ((x,y,z), (roll,pitch,yaw))."""
    if pose_el is None or pose_el.text is None:
        return (0, 0, 0), (0, 0, 0)
    vals = [float(v) for v in pose_el.text.split()]
    pos = tuple(vals[:3])
    rot = tuple(vals[3:6]) if len(vals) >= 6 else (0, 0, 0)
    return pos, rot


def _parse_vec3(text):
    vals = [float(v) for v in text.strip().split()]
    return tuple(vals[:3])


def _add_geometry(stage, prim_path, element, purpose="default"):
    """Create a UsdGeom prim from an SDF visual/collision element."""
    geom_el = element.find("geometry")
    if geom_el is None:
        return

    pose_el = element.find("pose")
    pose_pos, pose_rot = _parse_pose(pose_el)

    box_el = geom_el.find("box/size")
    cyl_el = geom_el.find("cylinder")
    sphere_el = geom_el.find("sphere/radius")

    if box_el is not None:
        sx, sy, sz = [float(v) for v in box_el.text.split()]
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pose_pos))
        cube.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
        if purpose == "guide":
            cube.CreatePurposeAttr("guide")

    elif cyl_el is not None:
        radius = float(cyl_el.findtext("radius", "0.05"))
        length = float(cyl_el.findtext("length", "0.1"))
        cyl = UsdGeom.Cylinder.Define(stage, prim_path)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.CreateAxisAttr("Z")
        if pose_pos != (0, 0, 0) or pose_rot != (0, 0, 0):
            cyl.AddTranslateOp().Set(Gf.Vec3d(*pose_pos))
            if any(r != 0 for r in pose_rot):
                cyl.AddRotateXYZOp().Set(
                    Gf.Vec3f(
                        math.degrees(pose_rot[0]),
                        math.degrees(pose_rot[1]),
                        math.degrees(pose_rot[2]),
                    )
                )
        if purpose == "guide":
            cyl.CreatePurposeAttr("guide")

    elif sphere_el is not None:
        radius = float(sphere_el.text)
        sphere = UsdGeom.Sphere.Define(stage, prim_path)
        sphere.CreateRadiusAttr(radius)
        if pose_pos != (0, 0, 0):
            sphere.AddTranslateOp().Set(Gf.Vec3d(*pose_pos))
        if purpose == "guide":
            sphere.CreatePurposeAttr("guide")

    # Material (from visual only)
    if purpose == "default":
        mat_el = element.find("material")
        if mat_el is not None:
            diffuse_el = mat_el.find("diffuse")
            if diffuse_el is not None:
                rgba = [float(v) for v in diffuse_el.text.split()]
                mat_path = prim_path + "_mat"
                mat = UsdShade.Material.Define(stage, mat_path)
                shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
                shader.CreateIdAttr("UsdPreviewSurface")
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(rgba[0], rgba[1], rgba[2])
                )
                mat.CreateSurfaceOutput().ConnectToSource(
                    UsdShade.ConnectableAPI(shader), "surface"
                )
                prim = stage.GetPrimAtPath(prim_path)
                UsdShade.MaterialBindingAPI(prim).Bind(mat)
