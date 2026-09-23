from math import radians, sin, cos, tan
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
a=radians(angle);s=tan(a);crown=rf*(s+1/cos(a)-1)
# Preserve the user's 30 mm clear tray; the rear pocket takes the wall allowance.
wall_outer_y=-d/2+t+td
wall_inner_y=wall_outer_y+t
insert_front_y=wall_inner_y+fg
splitx=-w/2+t+gap+(w-2*t-2*gap-midgap)*f+midgap/2
assert 0<rf<t/2 and b<th<lh-exposed<h-exposed

def rr(width,depth,height,r):
    return extrude(RectangleRounded(width,depth,r),amount=height)

def shell_region(height,sloped=False,right_only=False):
    top=height+crown+t if sloped else height
    if sloped:
        # One rear pocket encloses both inserts; no left/right partition.
        od=d/2-wall_outer_y
        idp=d/2-t-wall_inner_y
        outside=Pos(0,(d/2+wall_outer_y)/2,0)*rr(w,od,top,ro)
        inside=Pos(0,(d/2-t+wall_inner_y)/2,b)*rr(w-2*t,idp,top+t,ri)
        shell=outside-inside
        if right_only:
            clip=Pos((splitx+w/2)/2,0,0)*Box(w/2-splitx,d+2*t,top+t,align=(Align.CENTER,Align.CENTER,Align.MIN))
            shell=shell & clip
        plane=Plane(origin=(0,d/2,height+crown),x_dir=(1,0,0),z_dir=(0,-sin(a),cos(a)))
        shell=split(shell,bisect_by=plane,keep=Keep.BOTTOM)
        edges=[e for e in shell.edges() if all(abs(p.Z-height-crown-s*(p.Y-d/2))<1e-5 for p in [e.position_at(u) for u in (0,0.5,1)])]
    else:
        shell=rr(w,d,top,ro)-Pos(0,0,b)*rr(w-2*t,d-2*t,top+t,ri)
        edges=[e for e in shell.edges() if abs(e.bounding_box().min.Z-height)<1e-5 and abs(e.bounding_box().max.Z-height)<1e-5]
    return fillet(edges,radius=rf)
base=shell_region(th)+shell_region(lh-exposed,True)+shell_region(h-exposed,True,True)
edge0=[e for e in base.edges() if abs(e.bounding_box().min.Z)<1e-5 and abs(e.bounding_box().max.Z)<1e-5]
base=chamfer(edge0,length=bc)
assert base.is_valid and len(base.solids())==1
assert abs(base.bounding_box().size.Z-(h-exposed))<1e-4
publish('organizer',base,'阶梯包围外托座')