"""Restore the imported gripper's native one-DOF linkage before PhysX parsing."""

from pxr import Usd, UsdPhysics


def configure_gripper_mimic_constraints(stage, robot_path='/World/Tinker'):
    """Keep authored gearing and references; remove compliance and extra motors.

    A zero natural frequency selects PhysX's hard mimic constraint. The source
    USD already authors the multiple-apply schema, even when an offline USD
    process without the PhysX plugin omits it from GetAppliedSchemas().
    """
    count=0
    root=stage.GetPrimAtPath(robot_path)
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        prefix='physxMimicJoint:rotX:'
        reference=prim.GetRelationship(prefix+'referenceJoint')
        if not reference or not reference.GetTargets():
            continue
        prim.AddAppliedSchema('PhysxMimicJointAPI:rotX')
        prim.GetAttribute(prefix+'naturalFrequency').Set(0.)
        prim.GetAttribute(prefix+'dampingRatio').Set(0.)
        drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(0.)
        count+=1
    return count
