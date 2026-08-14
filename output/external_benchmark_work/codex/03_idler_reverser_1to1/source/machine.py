"""Open three-shaft 1:1 reversing train, CAD in millimetres."""
from __future__ import annotations
import math
from pathlib import Path
from build123d import Align,Axis,Box,Compound,Cylinder,Location,Plane,Polygon,export_stl,extrude
XS={"input_shaft":-72.0,"idler_shaft":0.0,"output_shaft":72.0}
POSES={"base":(0,0,4),**{f"{s}_{side}_bearing":(x,y,90) for s,x in (("input",-72),("idler",0),("output",72)) for side,y in (("front",-30),("rear",30))},"input_shaft":(-72,0,90),"idler_shaft":(0,0,90),"output_shaft":(72,0,90),"input_gear":(-72,0,90),"idler_gear":(0,0,90),"output_gear":(72,0,90),"hand_crank":(-72,0,90)}
MECHANISM={"name":"open_idler_reverser_1to1","links":[{"name":n} for n in POSES],"ports":{n+"_axis":{"link":n,"axis":[0,1,0]} for n in XS},"relations":[{"type":"press_fit","outer":g,"inner":s} for g,s in (("input_gear","input_shaft"),("idler_gear","idler_shaft"),("output_gear","output_shaft"),("hand_crank","input_shaft"))]+[{"type":"ideal_external_gear_mesh","driving":a,"driven":b,"driving_teeth":24,"driven_teeth":24} for a,b in (("input_gear","idler_gear"),("idler_gear","output_gear"))],"motion_joints":[{"name":n+"_hinge","child":n,"axis":[0,1,0],"driver":n=="input_shaft"} for n in XS],"transmissions":[{"name":"input_to_idler","driving_joint":"input_shaft_hinge","driven_joint":"idler_shaft_hinge","ratio":-1.0},{"name":"idler_to_output","driving_joint":"idler_shaft_hinge","driven_joint":"output_shaft_hinge","ratio":-1.0}],"driver":{"joint":"input_shaft_hinge","source":"hand_crank"},"output":{"joint":"output_shaft_hinge"},"watch_links":["input_gear","idler_gear","output_gear"]}
def cy(r,l):return Cylinder(r,l,align=(Align.CENTER,Align.CENTER,Align.CENTER)).rotate(Axis.X,90)
def gear(teeth=24,module=3,thickness=12,phase=0):
 p=module*teeth/2;root=p-1.25*module;outer=p+module;body=cy(root,thickness);tw=.36*math.pi*module;rw=tw*root/outer;ts=[]
 for i in range(teeth):
  pr=Polygon((root-.7,-rw/2),(outer,-tw/2),(outer,tw/2),(root-.7,rw/2),align=None);ts.append(extrude(Plane.XZ*pr,amount=thickness/2,both=True).rotate(Axis.Y,phase+360*i/teeth))
 return body.fuse(*ts)
def bearing():return cy(14,10).cut(cy(6,12)).fuse(Box(20,10,69,align=(Align.CENTER,Align.CENTER,Align.CENTER)).translate((0,0,-47.5)))
def crank():return cy(10,8).translate((0,-48,0)).fuse(Box(9,6,36,align=(Align.CENTER,Align.CENTER,Align.CENTER)).translate((0,-52,18)),cy(6,20).translate((0,-62,36)))
def build_local_parts():
 p={"base":Box(250,100,8,align=(Align.CENTER,Align.CENTER,Align.CENTER)),"input_front_bearing":bearing(),"input_rear_bearing":bearing(),"idler_front_bearing":bearing(),"idler_rear_bearing":bearing(),"output_front_bearing":bearing(),"output_rear_bearing":bearing(),"input_shaft":cy(5,100),"idler_shaft":cy(5,80),"output_shaft":cy(5,80),"input_gear":gear(phase=0),"idler_gear":gear(phase=7.5),"output_gear":gear(phase=0),"hand_crank":crank()}
 for n,s in p.items():s.label=n
 return p
def build_machine():
 ch=[]
 for n,s in build_local_parts().items():q=s.moved(Location(POSES[n]));q.label=n;ch.append(q)
 r=Compound(children=ch);r.label="open_idler_reverser_1to1";return r
def gen_step():return build_machine()
def export_named_stls(d):
 d=Path(d);d.mkdir(parents=True,exist_ok=True)
 for n,s in build_local_parts().items():export_stl(s,d/f"{n}.stl",tolerance=.12,angular_tolerance=.12)
if __name__=="__main__":
 import argparse;p=argparse.ArgumentParser();p.add_argument("--export-stls");a=p.parse_args();assert build_machine().solids();export_named_stls(a.export_stls) if a.export_stls else None
