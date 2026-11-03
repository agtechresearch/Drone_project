"""Load STEP, list solids with bbox/volume, save a pickle of per-solid info."""
import sys, time, pickle
import cadquery as cq
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

src, out = sys.argv[1], sys.argv[2]
t=time.time()
r = STEPControl_Reader()
assert r.ReadFile(src) == IFSelect_RetDone, "read failed"
print("roots:", r.NbRootsForTransfer(), "read in %.1fs"%(time.time()-t), flush=True)
r.TransferRoots()
shape = r.OneShape()
print("transfer done %.1fs"%(time.time()-t), flush=True)

solids=[]
ex = TopExp_Explorer(shape, TopAbs_SOLID)
i=0
while ex.More():
    s = ex.Current()
    bb = Bnd_Box(); BRepBndLib.Add_s(s, bb)
    x0,y0,z0,x1,y1,z1 = bb.Get()
    gp = GProp_GProps(); BRepGProp.VolumeProperties_s(s, gp)
    v = gp.Mass(); c = gp.CentreOfMass()
    solids.append(dict(idx=i, bbox=(x0,y0,z0,x1,y1,z1), vol=v, com=(c.X(),c.Y(),c.Z())))
    i+=1; ex.Next()
print("solids:", len(solids), "%.1fs"%(time.time()-t), flush=True)

# overall bbox
bb = Bnd_Box(); BRepBndLib.Add_s(shape, bb)
print("overall bbox (mm):", ["%.1f"%v for v in bb.Get()])
pickle.dump(dict(solids=solids, overall=bb.Get()), open(out,'wb'))

# print largest 40 by bbox extent and by volume
def ext(s): x0,y0,z0,x1,y1,z1=s['bbox']; return (x1-x0,y1-y0,z1-z0)
print("\n--- top 40 by volume ---")
for s in sorted(solids, key=lambda s:-s['vol'])[:40]:
    e=ext(s); print("#%-5d vol=%9.0f mm3  ext=(%6.1f %6.1f %6.1f)  com=(%7.1f %7.1f %7.1f)"%(s['idx'],s['vol'],*e,*s['com']))
print("\n--- top 40 by max extent ---")
for s in sorted(solids, key=lambda s:-max(ext(s)))[:40]:
    e=ext(s); print("#%-5d vol=%9.0f mm3  ext=(%6.1f %6.1f %6.1f)  com=(%7.1f %7.1f %7.1f)"%(s['idx'],s['vol'],*e,*s['com']))
