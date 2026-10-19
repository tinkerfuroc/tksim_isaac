"""World-space extent of Tinker posed at a map pose with the HRI navigation arm pose."""
import math, sys
from pathlib import Path
import yaml
from pxr import Usd, UsdGeom, Gf
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tinker_fk import link_transforms

robot_dir, poses_yaml, name = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
Z0 = 0.0775  # grounded_base_height(robot.urdf), computed earlier
p = yaml.safe_load(open(poses_yaml))["poses"][name] if len(sys.argv) < 5 else dict(zip(("x", "y", "yaw"), map(float, sys.argv[4].split(","))))
arm = {f"joint{i+1}": math.radians(v) for i, v in enumerate([-87.4, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2])}
fk = link_transforms(robot_dir / "robot.urdf", arm)
st = Usd.Stage.CreateInMemory()
root = UsdGeom.Xform.Define(st, "/R")
root.AddTranslateOp().Set(Gf.Vec3d(p["x"], p["y"], Z0))
root.AddRotateZOp().Set(math.degrees(p["yaw"]))
rb = st.DefinePrim("/R/T")
rb.GetReferences().AddReference(str(robot_dir / "robot.usd"))
for c in rb.GetChildren():
    T = fk.get(c.GetName())
    if T is None:
        continue
    m = Gf.Matrix4d(*T.T.flatten().tolist())
    ops = {o.GetOpName(): o for o in UsdGeom.Xformable(c).GetOrderedXformOps()}
    ops["xformOp:translate"].Set(Gf.Vec3d(m.ExtractTranslation()))
    o, q = ops["xformOp:orient"], m.ExtractRotationQuat()
    o.Set(Gf.Quatd(q) if o.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(q))
cache = UsdGeom.BBoxCache(0, ["default", "render"])
b = cache.ComputeWorldBound(rb).ComputeAlignedRange()
print("pose", p)
print("robot x %.3f..%.3f  y %.3f..%.3f  z %.2f..%.2f" % (b.GetMin()[0], b.GetMax()[0], b.GetMin()[1], b.GetMax()[1], b.GetMin()[2], b.GetMax()[2]))
rows = []
for c in rb.GetChildren():
    r = cache.ComputeWorldBound(c).ComputeAlignedRange()
    if not r.IsEmpty():
        rows.append((r.GetMax()[0], c.GetName(), r.GetMin()[2], r.GetMax()[2]))
for mx, n, z0, z1 in sorted(rows, reverse=True)[:5]:
    print(f"  {n:24} max x {mx:.3f}  z {z0:.2f}..{z1:.2f}")
