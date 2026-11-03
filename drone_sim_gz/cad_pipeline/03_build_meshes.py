"""Build Gazebo meshes (meters, x fwd / y left / z up) from tessellated solids.
CAD frame: X = left(+)/right, Y = up, Z = forward (camera side).  gz = (Z_cad, X_cad, Y_cad) * 0.001
"""
import numpy as np, trimesh, pickle, math, sys
z=np.load('out/tess_fine.npz'); n=int(z['n'])
S=pickle.load(open('out/solids.pkl','rb'))['solids']
def ext(i): b=S[i]['bbox']; return (b[3]-b[0],b[4]-b[1],b[5]-b[2])
BLADES={1515,1516,1519,1520}; PINS={1511,1512,1513,1514,1517,1518,1521,1522}
HUBS={(-88.0,17.4,66.0):(1515,1516,1514,1513)}  # hub center : (blade,blade,pin,pin)

def cad2gz(v):  # (N,3) mm CAD -> m gazebo
    return np.stack([v[:,2], v[:,0], v[:,1]],axis=1)*0.001
def mesh(i): return trimesh.Trimesh(z[f'v{i}'].astype(np.float64), z[f'f{i}'], process=False)

# ---------- body ----------
keep=[]; dropped=0; drop_tris=0
for i in range(n):
    if i in BLADES or i in PINS: continue
    e=ext(i)
    if max(e)<6.0 and S[i]['vol']<150:   # screws, nuts, washers, tiny standoffs
        dropped+=1; drop_tris+=len(z[f'f{i}']); continue
    keep.append(i)
print(f"body: keep {len(keep)} solids, dropped {dropped} small solids ({drop_tris} tris)")
parts_b=[mesh(i) for i in keep]
ff=np.load('out/free_faces.npz')            # surface bodies (carbon plates etc.) not inside any solid
free=trimesh.Trimesh(ff['v'].astype(np.float64), ff['f'], process=False)
print("free-face (sheet body) tris:", len(free.faces))
parts_b.append(free)
# CAD carries some arm sheet parts only on the (+x,+y)/(-x,-y) arms (like the blades): mirror them (CAD X -> -X == gz y) to the other two arms
fm=trimesh.Trimesh(free.vertices.copy(), free.faces.copy(), process=True)
mirrored=[]
for c in fm.split(only_watertight=False):
    cen=cad2gz(c.centroid[None])[0]
    if abs(cen[0])>0.025 and abs(cen[1])>0.025:
        c2=c.copy(); c2.vertices[:,0]*=-1; c2.invert(); mirrored.append(c2)
print("mirrored arm sheet components:", len(mirrored), "tris:", sum(len(c.faces) for c in mirrored))
parts_b += mirrored
body=trimesh.util.concatenate(parts_b)
print("body raw tris:", len(body.faces))
body.merge_vertices(); body.update_faces(body.nondegenerate_faces())
target=int(sys.argv[1]) if len(sys.argv)>1 else 120000
body_s=body.simplify_quadric_decimation(face_count=target)
print("body decimated tris:", len(body_s.faces))
body_s.vertices=cad2gz(body_s.vertices)
body_s.export('out/starling2_body.stl')
bb=body_s.bounds; print("body bounds (m):", bb.round(4).tolist())

# ---------- propeller (unfold the folded blades of one hub) ----------
hub=np.array([-88.0,17.4,66.0]); bl=(1515,1516); pins=(1514,1513)
pin_c={p:np.array(S[p]['com']) for p in pins}
parts=[]
for b in bl:
    m=mesh(b); v=m.vertices
    # which pin is this blade's root? nearest pin to any vertex
    p=min(pins,key=lambda p: np.min(np.linalg.norm(v[:,[0,2]]-pin_c[p][[0,2]],axis=1)))
    pc=pin_c[p]
    cur=np.array(S[b]['com'])[[0,2]]-pc[[0,2]]; cur/=np.linalg.norm(cur)          # folded blade direction (XZ); COM-pin-hub collinear = centrifugal equilibrium
    des=pc[[0,2]]-hub[[0,2]]; des/=np.linalg.norm(des)                             # radial outward
    ang=math.atan2(cur[0]*des[1]-cur[1]*des[0], cur.dot(des))                       # angle in (X,Z) plane
    # rotate about Y axis through pin: (x,z) plane rotation
    c,s=math.cos(ang),math.sin(ang)
    x=v[:,0]-pc[0]; zz=v[:,2]-pc[2]
    v2=v.copy(); v2[:,0]=c*x - s*zz + pc[0]; v2[:,2]=s*x + c*zz + pc[2]
    m.vertices=v2; parts.append(m)
    print(f"blade {b}: pin {p}, unfold {math.degrees(ang):.1f} deg")
for p in pins: parts.append(mesh(p))
prop=trimesh.util.concatenate(parts); prop.vertices-=hub
prop_s=prop.simplify_quadric_decimation(face_count=6000)
prop_s.vertices=cad2gz(prop_s.vertices); prop_s.fix_normals()
prop_s.export('out/starling2_prop_a.stl')
mir=prop_s.copy(); mir.vertices[:,1]*=-1; mir.invert(); mir.export('out/starling2_prop_b.stl')
print("prop tris:", len(prop_s.faces), "bounds (m):", prop_s.bounds.round(4).tolist())

# ---------- motor/hub positions ----------
for name,(x,zc) in {'front-left':(88,66),'front-right':(-88,66),'rear-left':(88,-66),'rear-right':(-88,-66)}.items():
    g=cad2gz(np.array([[x,17.4,zc]]))[0]; print(f"rotor {name}: gz xyz = {g.round(4).tolist()}")
