"""Full STEP -> Gazebo mesh pipeline for one variant.  usage: 10_pipeline.py <variant> <step> [body_tris]
Outputs out/<variant>/: solids.pkl, tess.npz, free_faces.npz, starling2_body.stl, starling2_prop_{a,b}.stl, preview.png, report.txt
CAD frame: X=left, Y=up, Z=forward (camera side).  gz = (Z, X, Y) * 0.001
"""
import sys, os, time, math, pickle, numpy as np, trimesh
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer, TopExp
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

var, src = sys.argv[1], sys.argv[2]; target=int(sys.argv[3]) if len(sys.argv)>3 else 150000
od=f"out/{var}"; os.makedirs(od, exist_ok=True)
rep=open(f"{od}/report.txt","w")
def log(*a):
    s=" ".join(str(x) for x in a); print(s, flush=True); rep.write(s+"\n"); rep.flush()
def cad2gz(v): return np.stack([v[:,2], v[:,0], v[:,1]],axis=1)*0.001

t=time.time(); CACHED = (src=="--cached")
if not CACHED:
    r=STEPControl_Reader(); assert r.ReadFile(src)==IFSelect_RetDone; r.TransferRoots(); shape=r.OneShape()
    log(f"[{var}] loaded {src} in {time.time()-t:.0f}s")
    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.35, True); log(f"meshed {time.time()-t:.0f}s")

def tess_faces(face_iter):
    V=[];F=[];base=0
    for f in face_iter:
        face=TopoDS.Face_s(f); loc=TopLoc_Location(); tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None: continue
        tr=loc.Transformation(); n=tri.NbNodes()
        for k in range(1,n+1):
            p=tri.Node(k).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
        rev=face.Orientation()==TopAbs_REVERSED
        for k in range(1,tri.NbTriangles()+1):
            a,b,c=tri.Triangle(k).Get(); F.append((base+a-1,base+c-1,base+b-1) if rev else (base+a-1,base+b-1,base+c-1))
        base+=n
    return np.array(V,dtype=np.float64).reshape(-1,3), np.array(F,dtype=np.int32).reshape(-1,3)
def faces_of(s):
    ex=TopExp_Explorer(s, TopAbs_FACE)
    while ex.More(): yield ex.Current(); ex.Next()

# ---- solids ----
S=[]; meshes=[]
if CACHED:
    S=pickle.load(open(f"{od}/solids.pkl","rb")); zc=np.load(f"{od}/tess.npz"); meshes=[(zc[f"v{i}"].astype(np.float64), zc[f"f{i}"]) for i in range(int(zc["n"]))]
    fz=np.load(f"{od}/free_faces.npz"); fv,ff=fz["v"].astype(np.float64),fz["f"]; log(f"[{var}] cached: {len(S)} solids, sheet tris {len(ff)}")
ex=TopExp_Explorer(shape, TopAbs_SOLID) if not CACHED else None; i=0; solf=TopTools_IndexedMapOfShape()
while ex is not None and ex.More():
    s=ex.Current(); bb=Bnd_Box(); BRepBndLib.Add_s(s,bb); x0,y0,z0,x1,y1,z1=bb.Get()
    gp=GProp_GProps(); BRepGProp.VolumeProperties_s(s,gp); c=gp.CentreOfMass()
    S.append(dict(idx=i,bbox=(x0,y0,z0,x1,y1,z1),ext=(x1-x0,y1-y0,z1-z0),vol=gp.Mass(),com=np.array([c.X(),c.Y(),c.Z()])))
    meshes.append(tess_faces(faces_of(s))); TopExp.MapShapes_s(s, TopAbs_FACE, solf); i+=1; ex.Next()
if not CACHED:
    log(f"solids: {len(S)}, solid tris: {sum(len(f) for _,f in meshes)}")
    pickle.dump(S, open(f"{od}/solids.pkl","wb"))
    np.savez_compressed(f"{od}/tess.npz", **{f"v{i}":v.astype(np.float32) for i,(v,_) in enumerate(meshes)}, **{f"f{i}":f for i,(_,f) in enumerate(meshes)}, n=len(meshes))
    # ---- sheet bodies (faces not in any solid) ----
    allf=TopTools_IndexedMapOfShape(); TopExp.MapShapes_s(shape, TopAbs_FACE, allf)
    free=[allf.FindKey(k) for k in range(1,allf.Extent()+1) if not solf.Contains(allf.FindKey(k))]
    fv,ff=tess_faces(free); log(f"faces total {allf.Extent()}, free (sheet-body) faces {len(free)}, tris {len(ff)}")
    np.savez_compressed(f"{od}/free_faces.npz", v=fv, f=ff)
fin=[s for s in S if all(abs(v)<1e6 for v in s['bbox'])]
log("overall bbox (mm): x[%.1f %.1f] y[%.1f %.1f] z[%.1f %.1f]"%(min(s['bbox'][0] for s in fin),max(s['bbox'][3] for s in fin),min(s['bbox'][1] for s in fin),max(s['bbox'][4] for s in fin),min(s['bbox'][2] for s in fin),max(s['bbox'][5] for s in fin)))

# ---- feature detection ----
def near(a,b,tol): return all(abs(x-y)<=tol for x,y in zip(a,b))
bells=[s for s in S if near(s['ext'],(18,4.7,18),0.8) and 250<s['vol']<420]
hubs=[np.array([s['com'][0], 17.4, s['com'][2]]) for s in bells]
log("motor bells:", len(bells), [tuple(h.round(1)) for h in hubs])
blades=[s for s in S if 140<s['vol']<220 and 48<max(s['ext'])<62 and min(s['ext'])<7]
pins=[s for s in S if near(s['ext'],(3.9,7.4,3.9),0.5) and 20<s['vol']<50]
log("blades:", [s['idx'] for s in blades], " pins:", [s['idx'] for s in pins])
# pick the hub with 2 blades
hub=None
for h in hubs:
    bl=[s for s in blades if np.hypot(*(s['com'][[0,2]]-h[[0,2]]))<40]
    pn=[s for s in pins if np.hypot(*(s['com'][[0,2]]-h[[0,2]]))<8]
    if len(bl)==2 and len(pn)==2: hub, bl_sel, pn_sel = h, bl, pn; break
assert hub is not None, "no hub with 2 blades + 2 pins found"
log("prop built from hub", hub.round(1), "blades", [s['idx'] for s in bl_sel], "pins", [s['idx'] for s in pn_sel])
BL={s['idx'] for s in blades}; PN={s['idx'] for s in pins}

# ---- body ----
def tm(i): return trimesh.Trimesh(meshes[i][0], meshes[i][1], process=False)
def near_hub(s, r=35.0): return any(np.hypot(*(s['com'][[0,2]]-h[[0,2]]))<r for h in hubs)
cages=[s['idx'] for s in S if near_hub(s) and s['vol']>3000 and max(s['ext'])>45]
log("folded-blade retention cages (removed, not fitted in flight):", cages)
keep=[s['idx'] for s in S if s['idx'] not in BL and s['idx'] not in PN and s['idx'] not in cages and not (max(s['ext'])<6.0 and s['vol']<150)]
log(f"body: keep {len(keep)} solids, drop {len(S)-len(keep)-len(BL)-len(PN)} small solids")
parts=[tm(i) for i in keep]
fm=trimesh.Trimesh(fv.copy(), ff.copy(), process=True); hubs_gz=[cad2gz(h[None])[0] for h in hubs]; kept=0; dropped=0
for c in fm.split(only_watertight=False):
    if any(np.linalg.norm(cad2gz(c.centroid[None])[0][:2]-h[:2])<0.05 for h in hubs_gz): dropped+=len(c.faces); continue   # folded blade surfaces at the motors
    parts.append(c); kept+=len(c.faces)
log(f"sheet bodies: kept {kept} tris, dropped {dropped} tris of folded-blade surfaces near motors")
body=trimesh.util.concatenate(parts); body.merge_vertices(); body.update_faces(body.nondegenerate_faces())
log("body raw tris:", len(body.faces))
body_s=body.simplify_quadric_decimation(face_count=target); body_s.vertices=cad2gz(body_s.vertices)
body_s.export(f"{od}/starling2_body.stl"); log("body tris:", len(body_s.faces), "bounds (m):", body_s.bounds.round(4).tolist())

# ---- prop ----
pp=[]
for b in bl_sel:
    m=tm(b['idx']); v=m.vertices
    p=min(pn_sel, key=lambda p: np.min(np.linalg.norm(v[:,[0,2]]-p['com'][[0,2]],axis=1))); pc=p['com']
    cur=b['com'][[0,2]]-pc[[0,2]]; cur/=np.linalg.norm(cur); des=pc[[0,2]]-hub[[0,2]]; des/=np.linalg.norm(des)
    ang=math.atan2(cur[0]*des[1]-cur[1]*des[0], cur.dot(des)); c,s_=math.cos(ang),math.sin(ang)
    x=v[:,0]-pc[0]; zz=v[:,2]-pc[2]; v2=v.copy(); v2[:,0]=c*x-s_*zz+pc[0]; v2[:,2]=s_*x+c*zz+pc[2]; m.vertices=v2; pp.append(m)
for p in pn_sel: pp.append(tm(p['idx']))
prop=trimesh.util.concatenate(pp); prop.vertices-=hub
prop_s=prop.copy(); prop_s.merge_vertices(); prop_s.vertices=cad2gz(prop_s.vertices)
prop_s.export(f"{od}/starling2_prop_a.stl"); mir=prop_s.copy(); mir.vertices[:,1]*=-1; mir.invert(); mir.export(f"{od}/starling2_prop_b.stl")
log("prop tris:", len(prop_s.faces), "bounds (m):", prop_s.bounds.round(4).tolist())
rot=[cad2gz(h[None])[0] for h in hubs]; log("rotor hubs (gz m):", [tuple(r_.round(4)) for r_ in rot])

# ---- preview ----
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
light=np.array([0.4,0.3,0.85]); light/=np.linalg.norm(light)
views=[(28,-140,'iso (front-left)'),(89,-90,'top (x fwd = up)'),(5,-180,'FRONT (looking at -x face... camera side)'),(5,0,'REAR'),(-89,-90,'bottom'),(5,-90,'left side')]
fig=plt.figure(figsize=(24,15))
for k,(el,az,ttl) in enumerate(views):
    ax=fig.add_subplot(2,3,k+1,projection='3d'); sh=np.clip(np.abs(body_s.face_normals@light),0.2,1)
    ax.add_collection3d(Poly3DCollection(body_s.vertices[body_s.faces],facecolors=np.stack([sh*0.35,sh*0.35,sh*0.38,np.ones_like(sh)],1),linewidths=0))
    for j,h in enumerate(rot):
        m=(prop_s if j%2==0 else mir).copy(); m.vertices+=h; s2=np.clip(np.abs(m.face_normals@light),0.3,1)
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces],facecolors=np.stack([s2*0.9,s2*0.55,s2*0.1,np.ones_like(s2)],1),linewidths=0))
    ax.quiver(0,0,0.08,0.06,0,0,color='r',linewidth=2); ax.text(0.07,0,0.08,'x fwd',color='r')
    ax.set_xlim(-0.13,0.13); ax.set_ylim(-0.13,0.13); ax.set_zlim(-0.1,0.16); ax.view_init(elev=el,azim=az); ax.set_title(f"{var}: {ttl}"); ax.set_box_aspect((1,1,1))
plt.tight_layout(); plt.savefig(f"{od}/preview.png",dpi=70); log("preview saved", f"{od}/preview.png", f"total {time.time()-t:.0f}s")
