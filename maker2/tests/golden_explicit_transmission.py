"""Golden explicit compound transmission: equality + contact exclusion are atomic.

Run: python -m maker2.tests.golden_explicit_transmission
"""
from __future__ import annotations
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from build123d import Align,Cylinder,export_stl
from maker2.mjcf_builder import _add_explicit_transmissions
from maker2.model import KinematicModel,LinkSpec,MateSpec,PortSpec,TransmissionSpec

def main():
 d=Path(tempfile.mkdtemp());
 shaft=Cylinder(3,10,align=(Align.CENTER,Align.CENTER,Align.MIN)); wheel=Cylinder(10,5,align=(Align.CENTER,Align.CENTER,Align.MIN))-Cylinder(2.995,7,align=(Align.CENTER,Align.CENTER,Align.MIN))
 export_stl(shaft,d/'shaft.stl');export_stl(wheel,d/'wheel.stl')
 links=[LinkSpec('shaft','shaft',dof='spin'),LinkSpec('wheel','wheel',dof='spin',mount='shaft')]
 m=KinematicModel('x','shaft',links,ports_by_link={'shaft':[PortSpec('seat','shaft',diameter_mm=6,depth_mm=5)],'wheel':[PortSpec('bore','bore',diameter_mm=5.99,depth_mm=5)]},relations=[MateSpec('fit','press_fit','shaft','seat','wheel','bore')],transmissions=[TransmissionSpec('lock','compound_1to1','shaft','wheel',1)])
 root=ET.Element('mujoco'); world=ET.SubElement(root,'worldbody')
 for name in ('shaft','wheel'):
  body=ET.SubElement(world,'body',{'name':name});ET.SubElement(body,'joint',{'name':f'{name}_spin','type':'hinge'})
 metrics={};pairs=_add_explicit_transmissions(root,m,str(d),{x.name:x for x in links},metrics=metrics,log_fn=lambda *_:None)
 assert frozenset(('shaft','wheel')) in pairs
 j=root.find('./equality/joint');e=root.find('./contact/exclude')
 assert j is not None and j.get('polycoef')=='0 1 0 0 0'
 assert e is not None and {e.get('body1'),e.get('body2')}=={'shaft','wheel'}
 # A dof=fixed pressed-on hand has no authored coordinate; the builder must emit a
 # real follower hinge before referencing it from equality.
 fixed=LinkSpec('hand','hand',dof='fixed',mount='shaft')
 m2=KinematicModel('y','shaft',[links[0],fixed],ports_by_link={
  'shaft':[PortSpec('seat','shaft',diameter_mm=6,depth_mm=5)],
  'hand':[PortSpec('bore','bore',diameter_mm=5.99,depth_mm=5)]},
  relations=[MateSpec('fit2','press_fit','shaft','seat','hand','bore')],
  transmissions=[TransmissionSpec('lock2','compound_1to1','shaft','hand',1)])
 export_stl(wheel,d/'hand.stl')
 root2=ET.Element('mujoco');world2=ET.SubElement(root2,'worldbody')
 shaft_body=ET.SubElement(world2,'body',{'name':'shaft'});ET.SubElement(shaft_body,'joint',{'name':'shaft_spin','type':'hinge'})
 ET.SubElement(world2,'body',{'name':'hand'})
 pairs2=_add_explicit_transmissions(root2,m2,str(d),{'shaft':links[0],'hand':fixed},metrics={},log_fn=lambda *_:None)
 assert frozenset(('shaft','hand')) in pairs2
 assert root2.find("./worldbody/body[@name='hand']/joint[@name='hand_spin']") is not None
 assert root2.find('./equality/joint').get('joint1')=='hand_spin'
 print('golden explicit transmission: PASS')
if __name__=='__main__':main()
