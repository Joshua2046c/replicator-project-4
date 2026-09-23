from math import radians,sin,cos,tan
w=param('right_insert_width',118.832)
d=param('right_insert_depth',83.5)
h=param('right_insert_height',142.0)
t=param('right_insert_wall',3.0)
b=param('right_insert_floor',3.0)
ro=param('right_insert_outer_radius',5.0)
ri=param('right_insert_inner_radius',3.0)
rf=param('right_insert_rim_radius',0.8)
bc=param('right_insert_bottom_chamfer',0.4)
angle=param('right_insert_forward_slope_deg',15.0)
frontf=param('organizer_right_front_row_fraction',1/3)
midf=param('organizer_right_middle_remaining_fraction',0.5)
midcol=param('organizer_right_middle_split',0.5)
backcol=param('organizer_right_back_split',0.5)
slot=param('organizer_slot_width',18.0)
a=radians(angle);s=tan(a);crown=rf*(s+1/cos(a)-1)
assert 0<rf<t/2 and 0<bc<b
body=extrude(RectangleRounded(w,d,ro),amount=h+crown+t)
x=-w/2+t;y=-d/2+t;iw=w-2*t;idepth=d-2*t
fd=(idepth-2*t)*frontf;md=(idepth-2*t-fd)*midf;bd=idepth-2*t-fd-md
cw=(iw-3*t-slot)/3
cavities=[(x+i*(cw+t),y,cw,fd) for i in range(3)]
cavities.append((x+3*(cw+t),y,slot,fd))
for yy,cd,col in [(y+fd+t,md,midcol),(y+fd+t+md+t,bd,backcol)]:
    ca=(iw-t)*col
    cavities.extend([(x,yy,ca,cd),(x+ca+t,yy,iw-t-ca,cd)])
assert len(cavities)==8
for xx,yy,cw,cd in cavities:
    assert min(cw,cd)>2*ri and cw>=12
    body-=Pos(xx+cw/2,yy+cd/2,b)*extrude(RectangleRounded(cw,cd,ri),amount=h+t)
plane=Plane(origin=(0,d/2,h+crown),x_dir=(1,0,0),z_dir=(0,-sin(a),cos(a)))
body=split(body,bisect_by=plane,keep=Keep.BOTTOM)
rim=[e for e in body.edges() if all(abs(p.Z-h-crown-s*(p.Y-d/2))<1e-5 for p in [e.position_at(u) for u in (0,.5,1)])]
body=fillet(rim,radius=rf)
bottom=[e for e in body.edges() if abs(e.bounding_box().min.Z)<1e-5 and abs(e.bounding_box().max.Z)<1e-5]
body=chamfer(bottom,length=bc)
assert body.is_valid and len(body.solids())==1
assert abs(body.bounding_box().size.Z-h)<1e-4
publish('right_insert',body,'右侧八格内胆')