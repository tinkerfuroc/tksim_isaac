from pxr import Sdf, Usd, UsdPhysics

from tinker_sim_isaac.gripper_mimic import configure_gripper_mimic_constraints


def test_authored_mimic_is_rigid_without_independent_follower_drives():
    stage=Usd.Stage.CreateInMemory()
    root=stage.DefinePrim('/World/Tinker','Xform')
    master=UsdPhysics.RevoluteJoint.Define(stage,'/World/Tinker/joints/drive_joint')
    follower=UsdPhysics.RevoluteJoint.Define(stage,'/World/Tinker/joints/left_finger_joint')
    prim=follower.GetPrim()
    prim.AddAppliedSchema('PhysxMimicJointAPI:rotX')
    prim.CreateRelationship('physxMimicJoint:rotX:referenceJoint').SetTargets([master.GetPath()])
    prim.CreateAttribute('physxMimicJoint:rotX:gearing',Sdf.ValueTypeNames.Float).Set(-1.)
    prim.CreateAttribute('physxMimicJoint:rotX:offset',Sdf.ValueTypeNames.Float).Set(0.)
    prim.CreateAttribute('physxMimicJoint:rotX:naturalFrequency',Sdf.ValueTypeNames.Float).Set(25.)
    prim.CreateAttribute('physxMimicJoint:rotX:dampingRatio',Sdf.ValueTypeNames.Float).Set(.005)
    drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
    drive.CreateStiffnessAttr(1500.)
    drive.CreateDampingAttr(55.)
    drive.CreateMaxForceAttr(2.5)
    assert configure_gripper_mimic_constraints(stage,root.GetPath())==1
    assert prim.GetAttribute('physxMimicJoint:rotX:naturalFrequency').Get()==0.
    assert prim.GetAttribute('physxMimicJoint:rotX:dampingRatio').Get()==0.
    assert prim.GetRelationship('physxMimicJoint:rotX:referenceJoint').GetTargets()==[master.GetPath()]
    assert prim.GetAttribute('physxMimicJoint:rotX:gearing').Get()==-1.
    assert drive.GetStiffnessAttr().Get()==0.
    assert drive.GetDampingAttr().Get()==0.
    assert drive.GetMaxForceAttr().Get()==2.5
