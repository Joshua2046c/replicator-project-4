from math import radians,sin,cos,tan
w=param('left_insert_width',93.368)
d=param('left_insert_depth',83.5)
h=param('left_insert_height',87.0)
t=param('left_insert_wall',3.0)
b=param('left_insert_floor',3.0)
ro=param('left_insert_outer_radius',5.0)
ri=param('left_insert_inner_radius',3.0)
rf=param('left_insert_rim_radius',0.8)
bc=param('left_insert_bottom_chamfer',0.4)
angle=param('left_insert_forward_slope_deg',15.0)
rowf=param('organizer_left_front_row_fraction',0.5)
colf=param('organizer_left_front_split',0.5)
a=radians(angle);s=tan(a);crown=rf*(s+1/cos(a)-1)
assert 0<rf<t/2 and 0<bc<b
body=extrude(RectangleRounded(w,d,ro),amount=h+crown+t)
x=-w/2+t;y=-d/2+t;iw=w-2*t;idepth=d-2*t
fd=(idepth-t)*rowf;bd=idepth-t-fd;fw=(iw-t)*colf
cavities=[(x,y,fw,fd),(x+fw+t,y,iw-t-fw,fd),(x,y+fd+t,iw,bd)]
assert iw>=70 and bd>=25
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
publish('left_insert',body,'左侧三格内胆')