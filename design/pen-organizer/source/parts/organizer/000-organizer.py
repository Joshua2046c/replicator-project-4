# Reference user_style_reference defines appearance; dimensions come from product requirements.
w = param("organizer_width", 220.0)
d = param("organizer_depth", 120.0)
h = param("organizer_height", 145.0)
lh = param("organizer_left_height", 90.0)
t = param("organizer_wall", 2.4)
b = param("organizer_floor", 3.0)
f = param("organizer_left_fraction", 0.44)
td = param("organizer_tray_clear_depth", 30.0)
th = param("organizer_tray_height", 15.0)
ro = param("organizer_outer_radius", 5.0)
ri = param("organizer_inner_radius", 3.0)
rf = param("organizer_rim_radius", 0.8)
bc = param("organizer_bottom_chamfer", 0.4)
lf = param("organizer_left_front_row_fraction", 0.5)
lc = param("organizer_left_front_split", 0.5)
rff = param("organizer_right_front_row_fraction", 1.0/3.0)
rmf = param("organizer_right_middle_remaining_fraction", 0.5)
rmc = param("organizer_right_middle_split", 0.5)
rbc = param("organizer_right_back_split", 0.5)
sw = param("organizer_slot_width", 18.0)
assert 0 < b < th < lh < h
assert 0 < rf < t/2 and 0 < bc < b
assert all(0 < v < 1 for v in [f,lf,lc,rff,rmf,rmc,rbc])
lw = (w-3*t)*f
rw = (w-3*t)*(1-f)
xl = -w/2+t
split = xl+lw+t/2
xr = split+t/2
yt = -d/2+t
y0 = yt+td+t
y1 = d/2-t
ld1 = (y1-y0-t)*lf
ld2 = (y1-y0-t)*(1-lf)
rd1 = (y1-y0-2*t)*rff
rd2 = (y1-y0-2*t-rd1)*rmf
rd3 = y1-y0-2*t-rd1-rd2
assert lw >= 70 and ld2 >= 25

def prism(x0, x1, y0, y1, z0, z1, radius):
    return Pos((x0+x1)/2, (y0+y1)/2, z0) * extrude(RectangleRounded(x1-x0, y1-y0, radius), amount=z1-z0)

cavities = []
la = (lw-t)*lc
cavities += [("L_front_1",xl,y0,la,ld1,lh),("L_front_2",xl+la+t,y0,lw-t-la,ld1,lh),("L_back",xl,y0+ld1+t,lw,ld2,lh)]
for tag, yy, depth, fraction in [("middle",y0+rd1+t,rd2,rmc),("back",y0+rd1+t+rd2+t,rd3,rbc)]:
    a = (rw-t)*fraction
    cavities += [("R_"+tag+"_1",xr,yy,a,depth,h),("R_"+tag+"_2",xr+a+t,yy,rw-t-a,depth,h)]
a = (rw-3*t-sw)/3
for i in range(3):
    cavities.append(("R_front_"+str(i+1),xr+i*(a+t),y0,a,rd1,h))
cavities += [("R_front_slot",xr+3*(a+t),y0,sw,rd1,h),("tray_left",xl,yt,lw,td,th),("tray_right",xr,yt,rw,td,th)]
assert len(cavities) == 13
for tag,x,y,cw,cd,ch in cavities:
    assert cw >= 12 and min(cw,cd) > 2*ri
    print(tag,"clear_mm",round(cw,6),round(cd,6),"floor",b,"rim",ch)

# Complete the cutouts and rims of each integral height region, then fuse.
# This avoids a multi-height fillet chain running into the step junction.
def pocketed_region(xmin,xmax,ymin,ymax,height,entries):
    region = prism(xmin,xmax,ymin,ymax,0,height,ro)
    for tag,x,y,cw,cd,ch in entries:
        region -= prism(x,x+cw,y,y+cd,b,height+t,ri)
    top_edges = [e for e in region.edges() if abs(e.bounding_box().min.Z-height)<1e-5 and abs(e.bounding_box().max.Z-height)<1e-5]
    print("Rounding region",height,"edges",len(top_edges))
    return fillet(top_edges,radius=rf)

base = pocketed_region(-w/2,w/2,-d/2,d/2,th,cavities)
low = pocketed_region(-w/2,w/2,y0-t,d/2,lh,[v for v in cavities if not v[0].startswith("tray")])
high = pocketed_region(split-t/2,w/2,y0-t,d/2,h,[v for v in cavities if v[0].startswith("R_")])
body = base + low + high
bottom_edges = [e for e in body.edges() if abs(e.bounding_box().min.Z)<1e-5 and abs(e.bounding_box().max.Z)<1e-5]
body = chamfer(bottom_edges,length=bc)
assert body.is_valid
assert len(body.solids()) == 1
bb = body.bounding_box()
assert abs(bb.size.X-w)<1e-4 and abs(bb.size.Y-d)<1e-4 and abs(bb.size.Z-h)<1e-4
print("PASS one valid solid; envelope",bb.size,"volume_mm3",body.volume)
publish("organizer",body,"阶梯分格笔筒")