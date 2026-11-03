"""Tessellate STEP; save per-solid triangle meshes (mm, CAD frame) to npz."""
import sys, time, numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

src, out, lin, ang = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
t=time.time()
r = STEPControl_Reader(); assert r.ReadFile(src) == IFSelect_RetDone
r.TransferRoots(); shape = r.OneShape()
print("loaded %.1fs"%(time.time()-t), flush=True)
BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
print("meshed %.1fs"%(time.time()-t), flush=True)

V=[]; F=[]; ntri=[]
ex = TopExp_Explorer(shape, TopAbs_SOLID); si=0
while ex.More():
    sol = ex.Current(); verts=[]; faces=[]; base=0
    fx = TopExp_Explorer(sol, TopAbs_FACE)
    while fx.More():
        face = TopoDS.Face_s(fx.Current()); loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            for i in range(1, n+1):
                p = tri.Node(i).Transformed(trsf); verts.append((p.X(),p.Y(),p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles()+1):
                a,b,c = tri.Triangle(i).Get()
                faces.append((base+a-1, base+c-1, base+b-1) if rev else (base+a-1, base+b-1, base+c-1))
            base += n
        fx.Next()
    V.append(np.array(verts, dtype=np.float32).reshape(-1,3)); F.append(np.array(faces, dtype=np.int32).reshape(-1,3))
    ntri.append(len(faces)); si+=1; ex.Next()
    if si%200==0: print("  solid", si, "%.0fs"%(time.time()-t), flush=True)
print("solids:", si, "total tris:", sum(ntri), "%.1fs"%(time.time()-t))
np.savez_compressed(out, **{f"v{i}":v for i,v in enumerate(V)}, **{f"f{i}":f for i,f in enumerate(F)}, n=si)
