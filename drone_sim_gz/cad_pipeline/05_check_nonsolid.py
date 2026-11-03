import time, numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer, TopExp
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_SHELL, TopAbs_COMPOUND
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
r=STEPControl_Reader(); r.ReadFile('step/starling2_c26_rev1.step'); r.TransferRoots(); shape=r.OneShape()
allf=TopTools_IndexedMapOfShape(); TopExp.MapShapes_s(shape, TopAbs_FACE, allf)
solf=TopTools_IndexedMapOfShape()
ex=TopExp_Explorer(shape, TopAbs_SOLID)
while ex.More(): TopExp.MapShapes_s(ex.Current(), TopAbs_FACE, solf); ex.Next()
print("faces total:", allf.Extent(), " faces inside solids:", solf.Extent())
# free shells (not in solids)
free=[]
for i in range(1, allf.Extent()+1):
    f=allf.FindKey(i)
    if not solf.Contains(f): free.append(f)
print("free faces (not in any solid):", len(free))
if free:
    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.35, True)
    V=[];F=[];base=0
    for f in free:
        face=TopoDS.Face_s(f); loc=TopLoc_Location(); tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None: continue
        t=loc.Transformation()
        for k in range(1,tri.NbNodes()+1):
            p=tri.Node(k).Transformed(t); V.append((p.X(),p.Y(),p.Z()))
        for k in range(1,tri.NbTriangles()+1):
            a,b,c=tri.Triangle(k).Get(); F.append((base+a-1,base+b-1,base+c-1))
        base+=tri.NbNodes()
    V=np.array(V);F=np.array(F); print("free-face tris:",len(F)); 
    if len(V): print("free-face bbox:",V.min(0).round(1),V.max(0).round(1))
    np.savez_compressed('out/free_faces.npz',v=V,f=F)
