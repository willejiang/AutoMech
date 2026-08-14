"""Openwork two-hand clock train, CAD units millimetres."""
from __future__ import annotations
import math
from pathlib import Path
from build123d import Align,Axis,Box,Compound,Cylinder,Location,Plane,Polygon,export_stl,extrude
CENTER=(45.0,0.0,100.0); INTER=(-27.0,0.0,100.0)
POSES={"base":(0,0,4),"central_front_bearing":(45,-28,100),"central_rear_bearing":(45,28,100),"intermediate_front_bearing":(-27,-28,100),"intermediate_rear_bearing":(-27,28,100),"minute_input_shaft":CENTER,"hour_output_sleeve":CENTER,"intermediate_shaft":INTER,"minute_input_pinion":CENTER,"stage1_intermediate_gear":INTER,"stage2_intermediate_pinion":INTER,"hour_output_gear":CENTER,"minute_hand":CENTER,"hour_hand":CENTER}
MECHANISM={"name":"openwork_clock_12to1","links":[{"name":n} for n in POSES],"ports":{"coaxial_minute_axis":{"link":"minute_input_shaft","axis":[0,1,0]},"coaxial_hour_axis":{"link":"hour_output_sleeve","axis":[0,1,0]},"intermediate_axis":{"link":"intermediate_shaft","axis":[0,1,0]}},"relations":[{"type":"running_bearing","inner":"minute_input_shaft","outer":"hour_output_sleeve","radial_clearance_mm":1.5},{"type":"press_fit","outer":"minute_input_pinion","inner":"minute_input_shaft"},{"type":"press_fit","outer":"minute_hand","inner":"minute_input_shaft"},{"type":"press_fit","outer":"stage1_intermediate_gear","inner":"intermediate_shaft"},{"type":"press_fit","outer":"stage2_intermediate_pinion","inner":"intermediate_shaft"},{"type":"press_fit","outer":"hour_output_gear","inner":"hour_output_sleeve"},{"type":"press_fit","outer":"hour_hand","inner":"hour_output_sleeve"},{"type":"ideal_external_gear_mesh","driving":"minute_input_pinion","driven":"stage1_intermediate_gear","driving_teeth":15,"driven_teeth":45},{"type":"ideal_external_gear_mesh","driving":"stage2_intermediate_pinion","driven":"hour_output_gear","driving_teeth":12,"driven_teeth":48}],"motion_joints":[{"name":"minute_input_hinge","child":"minute_input_shaft","axis":[0,1,0],"driver":True},{"name":"intermediate_shaft_hinge","child":"intermediate_shaft","axis":[0,1,0],"driver":False},{"name":"hour_output_hinge","child":"hour_output_sleeve","axis":[0,1,0],"driver":False}],"transmissions":[{"name":"minute_to_intermediate_3to1","driving_joint":"minute_input_hinge","driven_joint":"intermediate_shaft_hinge","ratio":-0.3333333333333333},{"name":"intermediate_to_hour_4to1","driving_joint":"intermediate_shaft_hinge","driven_joint":"hour_output_hinge","ratio":-0.25}],"driver":{"joint":"minute_input_hinge"},"output":{"joint":"hour_output_hinge","link":"hour_output_sleeve"},"watch_links":["minute_hand","hour_hand","hour_output_sleeve"]}
def cy(r,l):return Cylinder(r,l,align=(Align.CENTER,Align.CENTER,Align.CENTER)).rotate(Axis.X,90)
def gear(teeth,module=2.4,thickness=8,y=0,phase=0):
 p=module*teeth/2;root=p-1.25*module;outer=p+module;body=cy(root,thickness);tw=.28*math.pi*module;rw=tw*root/outer;ts=[]
 for i in range(teeth):
  pr=Polygon((root-.5,-rw/2),(outer,-tw/2),(outer,tw/2),(root-.5,rw/2),align=None);ts.append(extrude(Plane.XZ*pr,amount=thickness/2,both=True).rotate(Axis.Y,phase+360*i/teeth))
 return body.fuse(*ts).translate((0,y,0))
def bearing():return cy(15,10).cut(cy(8.5,12)).fuse(Box(22,10,77,align=(Align.CENTER,Align.CENTER,Align.CENTER)).translate((0,0,-53.5)))
def hand(length,width,y):return cy(width*.9,4).translate((0,y,0)).fuse(Box(width,4,length,align=(Align.CENTER,Align.CENTER,Align.MIN)).translate((0,y,0)))
def build_local_parts():
 p={"base":Box(220,100,8,align=(Align.CENTER,Align.CENTER,Align.CENTER)),"central_front_bearing":bearing(),"central_rear_bearing":bearing(),"intermediate_front_bearing":bearing(),"intermediate_rear_bearing":bearing(),"minute_input_shaft":cy(4,86),"hour_output_sleeve":cy(7,62).cut(cy(5.5,64)),"intermediate_shaft":cy(5,80),"minute_input_pinion":gear(15,y=-14,phase=0).cut(cy(7.5,10).translate((0,-14,0))),"stage1_intermediate_gear":gear(45,y=-14,phase=4),"stage2_intermediate_pinion":gear(12,y=14,phase=0),"hour_output_gear":gear(48,y=14,phase=3.75).cut(cy(8.0,10).translate((0,14,0))),"minute_hand":hand(68,5,-48),"hour_hand":hand(48,8,-40).cut(cy(5.25,6).translate((0,-40,0)))}
 for n,s in p.items():s.label=n
 return p
def build_machine():
 ch=[]
 for n,s in build_local_parts().items():q=s.moved(Location(POSES[n]));q.label=n;ch.append(q)
 r=Compound(children=ch);r.label="openwork_clock_12to1";return r
def gen_step():return build_machine()
def export_named_stls(d):
 d=Path(d);d.mkdir(parents=True,exist_ok=True)
 for n,s in build_local_parts().items():export_stl(s,d/f"{n}.stl",tolerance=.12,angular_tolerance=.12)
if __name__=="__main__":
 import argparse;p=argparse.ArgumentParser();p.add_argument("--export-stls");a=p.parse_args();assert build_machine().solids();export_named_stls(a.export_stls) if a.export_stls else None
