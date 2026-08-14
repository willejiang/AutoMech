"""Parametric open-frame pumpjack benchmark mechanism (millimetres)."""
from __future__ import annotations
from pathlib import Path
import json, math
from build123d import Box, Cylinder, Compound, Pos, Rot, export_step, export_stl

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "meshes"

# Assembly convention: X horizontal motion plane, Y crankshaft axis/depth, Z up.
P = {
    "base_x": 360.0, "base_y": 150.0, "base_z": 12.0,
    "shaft_x": -90.0, "shaft_z": 110.0,
    "pivot_x": -100.0, "pivot_z": 220.0,
    "crank_r": 40.0, "pitman_len": 140.0,
    "beam_left": 100.0, "beam_right": 180.0,
    "output_x": 80.0, "beam_depth": 18.0,
}

MECHANISM = {
    "name": "open_hand_cranked_pumpjack",
    "units": "mm",
    "coordinate_system": {"motion_plane": "XZ", "shaft_axis": [0, 1, 0], "up": [0, 0, 1]},
    "links": [
        {"name":"base","dof":"fixed"},
        {"name":"crankshaft_input","dof":"spin","axis":[0,1,0]},
        {"name":"hand_crank","dof":"rigid_mount","parent":"crankshaft_input"},
        {"name":"crank_disk","dof":"rigid_mount","parent":"crankshaft_input"},
        {"name":"crank_pin","dof":"rigid_mount","parent":"crankshaft_input"},
        {"name":"pitman_rod","dof":"closure_link"},
        {"name":"walking_beam","dof":"spin","axis":[0,1,0]},
        {"name":"beam_pivot","dof":"fixed","axis":[0,1,0]},
        {"name":"polished_rod_output","dof":"slide","axis":[0,0,1]},
        {"name":"vertical_guide","dof":"fixed","axis":[0,0,1]},
        {"name":"left_support","dof":"fixed"}, {"name":"right_support","dof":"fixed"}
    ],
    "ports": {
        "crankshaft_input.axis":{"link":"crankshaft_input","point":[-90,0,110],"axis":[0,1,0]},
        "crank_pin.center":{"link":"crank_pin","point":[-90,0,110],"radius":40},
        "pitman_rod.lower":{"link":"pitman_rod","end":"lower"},
        "pitman_rod.upper":{"link":"pitman_rod","end":"upper"},
        "walking_beam.left_pin":{"link":"walking_beam","point":[-100,0,220],"radius":100},
        "walking_beam.right_pin":{"link":"walking_beam","point":[-100,0,220],"radius":180},
        "beam_pivot.axis":{"link":"walking_beam","point":[-100,0,220],"axis":[0,1,0]},
        "polished_rod_output.top":{"link":"polished_rod_output"},
        "vertical_guide.axis":{"link":"vertical_guide","axis":[0,0,1]}
    },
    "relations": [
        {"name":"base_to_crank_bearing","kind":"running_bearing","a":"base","b":"crankshaft_input","axis":[0,1,0]},
        {"name":"disk_press_fit","kind":"press_fit","a":"crankshaft_input","b":"crank_disk"},
        {"name":"handle_press_fit","kind":"press_fit","a":"crankshaft_input","b":"hand_crank"},
        {"name":"dedicated_crank_pin_fit","kind":"pin_fit","a":"crank_disk","b":"crank_pin"},
        {"name":"lower_pin_closure","kind":"pin_closure","a":"crank_pin","b":"pitman_rod"},
        {"name":"upper_pin_closure","kind":"pin_closure","a":"pitman_rod","b":"walking_beam"},
        {"name":"beam_running_bearing","kind":"running_bearing","a":"beam_pivot","b":"walking_beam","axis":[0,1,0]},
        {"name":"output_pin_closure","kind":"pin_closure","a":"walking_beam","b":"polished_rod_output"},
        {"name":"vertical_guided_output","kind":"linear_guide","a":"vertical_guide","b":"polished_rod_output","axis":[0,0,1]}
    ],
    "motion_joints": [
        {"name":"crankshaft_hinge","kind":"hinge","parent":"base","child":"crankshaft_input","axis":[0,1,0]},
        {"name":"walking_beam_hinge","kind":"hinge","parent":"base","child":"walking_beam","axis":[0,1,0]},
        {"name":"polished_rod_slide","kind":"slide","parent":"base","child":"polished_rod_output","axis":[0,0,1]}
    ],
    "transmissions": [], "planetary_stages": [], "mesh_pairs": [],
    "driver":{"joint":"crankshaft_hinge","mode":"finite_effort","actuator":"hand_crank_motor","max_effort_Nm":35.0},
    "output":{"joint":"polished_rod_slide","link":"polished_rod_output","coordinate":"z"},
    "watch_links":["crank_pin","pitman_rod","walking_beam","polished_rod_output"]
}

def centered_box(x, y, z, cx, cy, cz):
    return Pos(cx-x/2, cy-y/2, cz-z/2) * Box(x,y,z)

def cyl_y(radius, length, cx, cy, cz):
    return Pos(cx, cy+length/2, cz) * Rot(90,0,0) * Cylinder(radius, length)

def bar_between(name, a, b, width, depth):
    dx, dz = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx,dz)
    ang = -math.degrees(math.atan2(dz,dx))
    s = centered_box(length,width,depth,0,0,0)
    s = Pos((a[0]+b[0])/2,0,(a[1]+b[1])/2) * Rot(0,ang,0) * s
    s.label=name
    return s

def initial_geometry():
    shaft=(-90.0,110.0); pivot=(-100.0,220.0)
    crank_pin=(-50.0,110.0)  # crank angle zero
    # Circle intersection selects right-facing beam attachment.
    vx,vz=crank_pin[0]-pivot[0],crank_pin[1]-pivot[1]; d=math.hypot(vx,vz)
    R=P["beam_left"]; L=P["pitman_len"]
    a=(R*R-L*L+d*d)/(2*d); h=math.sqrt(max(0,R*R-a*a))
    x=pivot[0]+a*vx/d-h*vz/d; z=pivot[1]+a*vz/d+h*vx/d
    beam_left=(x,z)
    ux=(beam_left[0]-pivot[0])/R; uz=(beam_left[1]-pivot[1])/R
    beam_right=(pivot[0]-P["beam_right"]*ux,pivot[1]-P["beam_right"]*uz)
    return crank_pin,beam_left,beam_right

def named_parts():
    cp,bl,br=initial_geometry(); parts={}
    parts["base"]=centered_box(360,150,12,0,0,6)
    parts["left_support"]=centered_box(18,18,210,-100,-50,111)
    parts["right_support"]=centered_box(18,18,210,-100,50,111)
    parts["beam_pivot"]=cyl_y(13,120,-100,-60,220)
    parts["crankshaft_input"]=cyl_y(10,130,-90,-65,110)
    parts["crank_disk"]=cyl_y(52,12,-90,-20,110)
    # L-shaped visible crank at camera side: arm plus grip, represented as one rigid compound.
    arm=centered_box(75,10,10,-52.5,58,110); grip=cyl_y(8,42,-15,58,110)
    parts["hand_crank"]=Compound(children=[arm,grip])
    parts["crank_pin"]=cyl_y(8,34,cp[0],-26,cp[1])
    parts["pitman_rod"]=bar_between("pitman_rod",cp,bl,14,12)
    parts["walking_beam"]=bar_between("walking_beam",bl,br,20,18)
    # Vertical polished rod hangs from the beam's right pin.
    parts["polished_rod_output"]=Pos(br[0],0,br[1]-150) * Cylinder(7,150)
    guide1=centered_box(16,24,18,br[0]-18,0,105); guide2=centered_box(16,24,18,br[0]+18,0,105)
    parts["vertical_guide"]=Compound(children=[guide1,guide2])
    for n,s in parts.items(): s.label=n
    return parts

def build_machine():
    return Compound(label="open_hand_cranked_pumpjack",children=list(named_parts().values()))

def gen_step():
    return build_machine()

def export_named_meshes():
    MESH_DIR.mkdir(parents=True,exist_ok=True)
    for name,shape in named_parts().items():
        export_stl(shape, str(MESH_DIR/f"{name}.stl"), tolerance=0.25, angular_tolerance=0.2)

def main():
    export_named_meshes()
    export_step(build_machine(), str(ROOT/"open_hand_cranked_pumpjack.step"))
    print(json.dumps({"status":"ok","parts":sorted(named_parts()),"step":str(ROOT/"open_hand_cranked_pumpjack.step")},indent=2))

if __name__ == "__main__": main()
