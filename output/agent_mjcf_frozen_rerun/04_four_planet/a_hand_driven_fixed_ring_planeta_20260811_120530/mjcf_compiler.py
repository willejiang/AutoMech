def compile_mjcf(facts,out):
 links=['baseplate','input_thrust_pad','lower_input_bearing','output_bearing_pedestal','output_bearing','ring_support','fixed_ring_gear','top_post_1','top_post_2','top_post_3','top_bridge','upper_input_bearing','input_shaft','sun_gear','crank_arm','hand_grip','output_carrier']+['planet_pin_'+str(i) for i in range(1,5)]+['planet_gear_'+str(i) for i in range(1,5)]
 par={'baseplate':'','input_thrust_pad':'baseplate','lower_input_bearing':'baseplate','output_bearing_pedestal':'baseplate','output_bearing':'output_bearing_pedestal','ring_support':'baseplate','fixed_ring_gear':'ring_support','top_post_1':'baseplate','top_post_2':'baseplate','top_post_3':'baseplate','top_bridge':'top_post_1','upper_input_bearing':'top_bridge','input_shaft':'baseplate','sun_gear':'input_shaft','crank_arm':'input_shaft','hand_grip':'crank_arm','output_carrier':'baseplate'}
 for i in range(1,5): par['planet_pin_'+str(i)]='output_carrier';par['planet_gear_'+str(i)]='planet_pin_'+str(i)
 mov=['input_shaft','sun_gear','crank_arm','hand_grip','output_carrier']+['planet_pin_'+str(i) for i in range(1,5)]+['planet_gear_'+str(i) for i in range(1,5)]
 cd={}
 for a in mov:
  for b in links:
   if a!=b:
    x,y=(a,b) if a<b else (b,a);k=x+'|'+y
    if k not in cd: cd[k]={'pair':[x,y],'action':'keep','reason':'Potentially active pair kept because measurement is uncertain or real contact may be required.','source_entity_ids':['link/'+x,'link/'+y],'fact_ids':['pair/'+x+'/'+y]}
 ex=[('input_shaft','lower_input_bearing','Running journal hinge',['relation/lower_input_journal'],['fit/lower_input_bearing/bore/input_shaft/lower_journal']),('input_shaft','upper_input_bearing','Running journal hinge',['relation/upper_input_journal'],['fit/upper_input_bearing/bore/input_shaft/upper_journal']),('output_carrier','output_bearing','Running journal hinge',['relation/carrier_output_journal'],['fit/output_bearing/bore/output_carrier/sleeve_journal']),('input_shaft','sun_gear','Interference press fit rigid carrying',['relation/sun_on_input_shaft'],['fit/sun_gear/bore/input_shaft/sun_seat']),('input_shaft','crank_arm','Interference press fit rigid carrying',['relation/crank_on_input_shaft'],['fit/crank_arm/hub_bore/input_shaft/crank_seat']),('crank_arm','hand_grip','Rigid face mount',['relation/grip_on_crank'],['fit/crank_arm/grip_face/hand_grip/bottom_face']),('input_shaft','output_carrier','Measured running clearance between independent coaxial members',['motion_joint/input_rotation','motion_joint/carrier_output_rotation'],['fit/output_carrier/central_running_bore/input_shaft/shaft_outer']),('crank_arm','top_bridge','Runtime high-impulse false contact at zero-thickness tangency stalls driver',['link/crank_arm','link/top_bridge'],['pair/crank_arm/top_bridge']),('crank_arm','upper_input_bearing','Runtime high-impulse false contact at zero-thickness tangency stalls driver',['link/crank_arm','link/upper_input_bearing'],['pair/crank_arm/upper_input_bearing'])]
 for i in range(1,5):
  s=str(i);ex += [('planet_gear_'+s,'planet_pin_'+s,'Clearance planet journal hinge',['relation/planet_'+s+'_journal'],['fit/planet_gear_'+s+'/bore/planet_pin_'+s+'/journal']),('sun_gear','planet_gear_'+s,'Ideal external gear mesh equality',['transmission/sun_to_planet_'+s],['pair/planet_gear_'+s+'/sun_gear']),('fixed_ring_gear','planet_gear_'+s,'Ideal internal gear mesh equality',['transmission/planet_'+s+'_to_fixed_ring'],['pair/fixed_ring_gear/planet_gear_'+s]),('output_carrier','planet_pin_'+s,'Rigidly carried dedicated pin',['link/planet_pin_'+s],['pair/output_carrier/planet_pin_'+s])]
 for a,b,r,s,f in ex:
  x,y=(a,b) if a<b else (b,a);cd[x+'|'+y]={'pair':[x,y],'action':'exclude','reason':r,'source_entity_ids':s,'fact_ids':f}
 cm={n:None for n in links};cm.update({'input_shaft':'input_rotation','sun_gear':'input_rotation','crank_arm':'input_rotation','hand_grip':'input_rotation','output_carrier':'carrier_output_rotation'})
 for i in range(1,5): cm['planet_pin_'+str(i)]='carrier_output_rotation';cm['planet_gear_'+str(i)]='planet_'+str(i)+'_spin_on_carrier'
 out.topology_plan({'coordinate_map':cm,'tree_edges':[[par[n],n] for n in links if par[n]],'closure_edges':[],'rigid_carried':[['input_shaft','sun_gear'],['input_shaft','crank_arm'],['crank_arm','hand_grip']]+[['output_carrier','planet_pin_'+str(i)] for i in range(1,5)],'independent_coaxial':[['input_shaft','output_carrier']],'transmissions':[{'name':'fixed_ring_stage','ratio':0.25},{'name':'planet_spins','ratio':-0.75}],'contact_decisions':[cd[k] for k in cd],'support_ground':['baseplate'],'support_strategy':'Fixed baseplate with global ground retained; no support patch.'})
 for n in links: out.body(n,par[n])
 out.joint('input_shaft','input_rotation','hinge',(0,0,1),(0,0,0),'world');out.joint('output_carrier','carrier_output_rotation','hinge',(0,0,1),(0,0,0),'world')
 c=[(30,0,28),(0,30,28),(-30,0,28),(0,-30,28)]
 for i in range(1,5): out.joint('planet_gear_'+str(i),'planet_'+str(i)+'_spin_on_carrier','hinge',(0,0,1),c[i-1],'world')
 out.joint_equality('fixed_ring_carrier_ratio','input_rotation','carrier_output_rotation',.25,0,'Fixed-ring 20/60 planetary ratio',['planetary_stage/fixed_ring_stage'],['pair/fixed_ring_gear/planet_gear_1'])
 for i in range(1,5): out.joint_equality('planet_'+str(i)+'_mesh','input_rotation','planet_'+str(i)+'_spin_on_carrier',-.75,0,'Planet spin in fixed-ring stage',['transmission/sun_to_planet_'+str(i),'transmission/planet_'+str(i)+'_to_fixed_ring'],['pair/planet_gear_'+str(i)+'/sun_gear','pair/fixed_ring_gear/planet_gear_'+str(i)])
 for d in cd.values():
  if d['action']=='exclude': out.exclude(d['pair'][0],d['pair'][1],d['reason'],d['source_entity_ids'],d['fact_ids'])
 for e in facts['entity_ids']:
  n='baseplate'
  if e.startswith('link/'): n=e[5:]
  elif e.startswith('pose/place_'): n=e[11:]
  elif e.startswith('motion_joint/'): n=e[13:]
  elif e.startswith('transmission/sun_to_planet_'): n='planet_'+e[-1]+'_mesh'
  elif e.startswith('transmission/planet_'): n='planet_'+e[20]+'_mesh'
  elif e=='planetary_stage/fixed_ring_stage': n='fixed_ring_carrier_ratio'
  elif e.startswith('role/driver') or e=='role/watch/input_shaft' or e=='role/watch/sun_gear': n='input_rotation'
  elif e.startswith('role/output') or e=='role/watch/output_carrier': n='carrier_output_rotation'
  elif e.startswith('role/watch/planet_gear_'): n='planet_'+e[-1]+'_spin_on_carrier'
  out.decision(e,'represented_by',[n],'Mapped to emitted topology.',[e])
