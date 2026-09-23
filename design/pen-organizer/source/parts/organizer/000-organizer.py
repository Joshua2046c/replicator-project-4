from math import radians, sin, cos, tan, sqrt
w=param('organizer_width',220.0)
d=param('organizer_depth',120.0)
h=param('organizer_height',145.0)
lh=param('organizer_left_height',90.0)
t=param('organizer_wall',3.0)
b=param('organizer_floor',3.0)
f=param('organizer_left_fraction',0.44)
td=param('organizer_tray_clear_depth',30.0)
th=param('organizer_tray_height',15.0)
ro=param('organizer_outer_radius',5.0)
ri=param('organizer_inner_radius',3.0)
rf=param('organizer_rim_radius',0.8)
bc=param('organizer_bottom_chamfer',0.4)
angle=param('organizer_forward_slope_deg',15.0)
exposed=param('organizer_grip_exposure',12.0)
gap=param('organizer_side_clearance',0.5)
midgap=param('organizer_insert_gap',0.8)
fg=param('organizer_front_clearance',0.5)
run=param('organizer_transition_width',55.0)
r=param('organizer_transition_radius',10.0)
a=radians(angle);s=tan(a);crown=rf*(s+1/cos(a)-1)
wy=-d/2+t+td; iy=wy+t
splitx=-w/2+t+gap+(w-2*t-2*gap-midgap)*f+midgap/2
zlo=lh-exposed+crown; zhi=h-exposed+crown; rise=zhi-zlo
assert 0<rf<t/2 and b<th<lh-exposed and run>2*r and splitx+run<w/2-ro
# Solve tangent circular blends joining a straight inclined segment.
lo=0.0; hi=radians(85)
for _ in range(60):
    q=(lo+hi)/2
    delta=(run-2*r*sin(q))*tan(q)+2*r*(1-cos(q))
    if delta<rise: lo=q
    else: hi=q
q=(lo+hi)/2
x0=splitx; x1=x0+run
p0=(x0,d/2,zlo)
p1=(x0+r*sin(q),d/2,zlo+r*(1-cos(q)))
p2=(x1-r*sin(q),d/2,zhi-r*(1-cos(q)))
p3=(x1,d/2,zhi)
m1=(x0+r*sin(q/2),d/2,zlo+r*(1-cos(q/2)))
m2=(x1-r*sin(q/2),d/2,zhi-r*(1-cos(q/2)))
edges=[Edge.make_line((-w,d/2,-h),(w,d/2,-h)),Edge.make_line((w,d/2,-h),(w,d/2,zhi)),Edge.make_line((w,d/2,zhi),p3),ThreePointArc(p3,m2,p2),Edge.make_line(p2,p1),ThreePointArc(p1,m1,p0),Edge.make_line(p0,(-w,d/2,zlo)),Edge.make_line((-w,d/2,zlo),(-w,d/2,-h))]
profile=Face(Wire(edges))
limit=extrude(profile,amount=(d+2*t)/cos(a),dir=(0,-cos(a),-sin(a)))
def rr(width,depth,height,radius):
    return extrude(RectangleRounded(width,depth,radius),amount=height)
top=h+t
rear=Pos(0,(d/2+wy)/2,0)*rr(w,d/2-wy,top,ro)-Pos(0,(d/2-t+iy)/2,b)*rr(w-2*t,d/2-t-iy,top+t,ri)
rear=rear & limit
# Only the upper perimeter, including its blended ramp, is rounded.
def roof(x):
    if x<=x0: return zlo
    if x>=x1: return zhi
    if x<p1[0]: return zlo+r-sqrt(max(0,r*r-(x-x0)**2))
    if x>p2[0]: return zhi-r+sqrt(max(0,r*r-(x-x1)**2))
    return p1[2]+(x-p1[0])*tan(q)
rim=[e for e in rear.edges() if all(abs(p.Z-roof(p.X)-s*(p.Y-d/2))<1e-4 for p in [e.position_at(u) for u in (0,.25,.5,.75,1)])]
assert len(rim)>0
rear=fillet(rim,radius=rf)
tray=rr(w,d,th,ro)-Pos(0,0,b)*rr(w-2*t,d-2*t,th+t,ri)
tray=fillet([e for e in tray.edges() if abs(e.bounding_box().min.Z-th)<1e-5 and abs(e.bounding_box().max.Z-th)<1e-5],radius=rf)
base=rear+tray
base=chamfer([e for e in base.edges() if abs(e.bounding_box().min.Z)<1e-5 and abs(e.bounding_box().max.Z)<1e-5],length=bc)
assert base.is_valid and len(base.solids())==1
assert abs(base.bounding_box().size.Z-(h-exposed))<1e-4
print('Transition width',run,'radius',r,'rise',rise,'slope_deg',q*180/3.141592653589793)
publish('organizer',base,'圆弧斜坡外托座')