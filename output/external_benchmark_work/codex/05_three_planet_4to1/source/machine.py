"""Open three-planet, ring-fixed 4:1 reducer. CAD units millimetres."""
from __future__ import annotations
import math
from pathlib import Path
from build123d import Align,Axis,Box,Compound,Cylinder,Location,Plane,Polygon,export_stl,extrude
C=(0.,0.,105.); ORBIT=48.; ANGLES=(90.,210.,330.)
POSES={"base":(0,0,4),"left_support":(-90,0,56),"right_support":(90,0,56),"fixed_ring":C,"sun_input":C,"carrier_output":C,"input_shaft":C,"hand_crank":C}
for i,a in enumerate(ANGLES,1):POSES[f"planet_{i}"]=(ORBIT*math.cos(math.radians(a)),0,C[2]+ORBIT*math.sin(math.radians(a)));POSES[f"planet_pin_{i}"]=POSES[f"planet_{i}"]
MECHANISM={"name":"open_three_planet_4to1","links":[{"name":n} for n in POSES],"ports":{"sun_axis":{"link":"sun_input","axis":[0,1,0]},"carrier_axis":{"link":"carrier_output","axis":[0,1,0]},**{f"planet_pin_{i}_axis":{"link":f"planet_pin_{i}","axis":[0,1,0]} for i in range(1,4)}},"relations":[{"type":"rigid_mount","parent":"base","child":"fixed_ring"},{"type":"press_fit","outer":"sun_input","inner":"input_shaft"},{"type":"press_fit","outer":"hand_crank","inner":"input_shaft"}]+[{"type":"dedicated_pin_fit","pin":f"planet_pin_{i}","gear":f"planet_{i}","radial_clearance_mm":.8} for i in range(1,4)],"motion_joints":[{"name":"sun_input_hinge","child":"sun_input","axis":[0,1,0],"driver":True},{"name":"carrier_output_hinge","child":"carrier_output","axis":[0,1,0],"driver":False}]+[{"name":f"planet_pin_{i}_hinge","parent":"carrier_output","child":f"planet_{i}","axis":[0,1,0],"driver":False} for i in range(1,4)],"transmissions":[],"planetary_stages":[{"name":"ring_fixed_4to1","sun_joint":"sun_input_hinge","carrier_joint":"carrier_output_hinge","ring_link":"fixed_ring","sun_teeth":24,"planet_teeth":24,"ring_teeth":72,"carrier_over_sun_ratio":.25,"planets":[f"planet_{i}" for i in range(1,4)]}],"mesh_pairs":[*[{"type":"ideal_external","a":"sun_input","b":f"planet_{i}"} for i in range(1,4)],*[{"type":"ideal_internal","a":f"planet_{i}","b":"fixed_ring"} for i in range(1,4)]],"driver":{"joint":"sun_input_hinge","source":"hand_crank"},"output":{"joint":"carrier_output_hinge","link":"carrier_output"},"watch_links":["carrier_output","planet_1","planet_2","planet_3"]}
def cy(r,l):return Cylinder(r,l,align=(Align.CENTER,Align.CENTER,Align.CENTER)).rotate(Axis.X,90)
def gear(teeth,module=2,thickness=10,phase=0):
 p=module*teeth/2;root=p-1.25*module;outer=p+module;body=cy(root,thickness);tw=.32*math.pi*module;rw=tw*root/outer;ts=[]
 for i in range(teeth):
  pr=Polygon((root-.4,-rw/2),(outer,-tw/2),(outer,tw/2),(root-.4,rw/2),align=None);ts.append(extrude(Plane.XZ*pr,amount=thickness/2,both=True).rotate(Axis.Y,phase+360*i/teeth))
 return body.fuse(*ts)
def ring():return cy(82,12).cut(cy(74,14))
def carrier():
 hub=cy(13,4).translate((0,12,0));arms=[]
 for a in ANGLES:
  arms.append(Box(ORBIT,4,8,align=(Align.MIN,Align.CENTER,Align.CENTER)).translate((0,12,0)).rotate(Axis.Y,-a))
 return hub.fuse(*arms)
def support(x):return Box(12,70,105,align=(Align.CENTER,Align.CENTER,Align.MIN)).translate((0,0,-52.5))
def crank():return cy(10,8).translate((0,-52,0)).fuse(Box(8,6,34,align=(Align.CENTER,Align.CENTER,Align.MIN)).translate((0,-56,0)),cy(6,20).translate((0,-66,34)))
def build_local_parts():
 p={"base":Box(220,110,8,align=(Align.CENTER,Align.CENTER,Align.CENTER)),"left_support":support(-90),"right_support":support(90),"fixed_ring":ring(),"sun_input":gear(24),"carrier_output":carrier(),"input_shaft":cy(5,100),"hand_crank":crank()}
 for i,a in enumerate(ANGLES,1):p[f"planet_{i}"]=gear(24,phase=i*5).cut(cy(5,12));p[f"planet_pin_{i}"]=cy(4,26).translate((0,8,0))
 for n,s in p.items():s.label=n
 return p
def build_machine():
 ch=[]
 for n,s in build_local_parts().items():q=s.moved(Location(POSES[n]));q.label=n;ch.append(q)
 r=Compound(children=ch);r.label="open_three_planet_4to1";return r
def gen_step():return build_machine()
def export_named_stls(d):
 d=Path(d);d.mkdir(parents=True,exist_ok=True)
 for n,s in build_local_parts().items():export_stl(s,d/f"{n}.stl",tolerance=.12,angular_tolerance=.12)
if __name__=="__main__":
 import argparse;p=argparse.ArgumentParser();p.add_argument("--export-stls");a=p.parse_args();assert build_machine().solids();export_named_stls(a.export_stls) if a.export_stls else None
