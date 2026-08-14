def compile_mjcf(facts,out):
 L=['base','lower_input_bearing','upper_input_bearing','carrier_bearing','input_shaft','ring_post_1','ring_post_2','ring_post_3','carrier','planet_pin_1','planet_pin_2','planet_pin_3','fixed_ring','sun_gear','planet_1','planet_2','planet_3','input_crank','crank_grip']
 R=[('shaft_in_lower_bearing','lower_input_bearing','journal','input_shaft','shaft_axis'),('shaft_in_upper_bearing','upper_input_bearing','journal','input_shaft','shaft_axis'),('sun_press_fit','input_shaft','sun_seat','sun_gear','shaft_bore'),('crank_press_fit','input_shaft','crank_seat','input_crank','hub_bore'),('carrier_on_support_bearing','carrier_bearing','carrier_journal','carrier','output_axis')]
 for i in [1,2,3]:
  q=str(i);R.append(('carrier_pin_press_fit_'+q,'carrier','pin_seat_'+q,'planet_pin_'+q,'pin_axis'));R.append(('planet_pin_journal_'+q,'planet_pin_'+q,'pin_axis','planet_'+q,'pin_bore'));R.append(('sun_planet_mesh_'+q,'sun_gear','sun_mesh','planet_'+q,'planet_mesh'));R.append(('ring_planet_mesh_'+q,'fixed_ring','internal_mesh','planet_'+q,'planet_mesh'))
 N={'input_shaft':['base','lower_input_bearing','upper_input_bearing','carrier_bearing','carrier','fixed_ring','sun_gear','planet_2','planet_3','input_crank','planet_1','crank_grip','planet_pin_2','planet_pin_3','planet_pin_1'],'sun_gear':['input_shaft','fixed_ring','planet_1','planet_2','planet_3','carrier','input_crank','planet_pin_3','planet_pin_2','carrier_bearing','upper_input_bearing','crank_grip','planet_pin_1','lower_input_bearing','base'],'carrier':['carrier_bearing','input_shaft','planet_pin_1','planet_pin_2','planet_pin_3','fixed_ring','sun_gear','planet_1','planet_2','planet_3','upper_input_bearing','lower_input_bearing','ring_post_1','ring_post_3','base','input_crank','crank_grip','ring_post_2'],'planet_1':['planet_pin_1','fixed_ring','sun_gear','carrier','planet_2','input_crank','planet_3','carrier_bearing','input_shaft','upper_input_bearing','crank_grip','lower_input_bearing','base','planet_pin_2','planet_pin_3','ring_post_1','ring_post_3'],'planet_2':['input_shaft','planet_pin_2','fixed_ring','sun_gear','carrier','planet_1','input_crank','carrier_bearing','upper_input_bearing','planet_3','ring_post_2','lower_input_bearing','ring_post_1','crank_grip','base','planet_pin_1'],'planet_3':['input_shaft','planet_pin_3','fixed_ring','sun_gear','carrier','input_crank','planet_1','carrier_bearing','upper_input_bearing','planet_2','ring_post_2','lower_input_bearing','crank_grip','ring_post_3','base','planet_pin_1'],'input_crank':['input_shaft','crank_grip','planet_pin_1','planet_1','planet_2','planet_3','fixed_ring','sun_gear','carrier','planet_pin_3','planet_pin_2','carrier_bearing','upper_input_bearing']}
 rp={}
 for r in R:
  p=sorted([r[1],r[3]]);rp[p[0]+'|'+p[1]]=r
 C=[];seen=[]
 for a in N:
  for b in N[a]:
   p=sorted([a,b]);k=p[0]+'|'+p[1]
   if k not in seen:
    seen.append(k);s=['link/'+p[0],'link/'+p[1]];f=['nearby/'+p[0]+'/'+p[1],'pair/'+p[0]+'/'+p[1]]
    if k in rp:
     r=rp[k];s.append('relation/'+r[0]);f.append('fit/'+r[1]+'/'+r[2]+'/'+r[3]+'/'+r[4]);act='exclude';why='Authored fit, bearing, or gear interface is idealized; exact-pair contact would fight its coordinate or equality.'
    elif k in ['base|input_shaft','crank_grip|input_crank']:act='keep';why='Touching geometry has unavailable overlap; preserve uncertain real contact.'
    else:act='exclude';why='Positive mesh separation or disjoint AABB and no contact function identifies exact proxy false-contact risk.'
    C.append({'pair':p,'action':act,'reason':why,'source_entity_ids':s,'fact_ids':f})
 cm={}
 for n in L:cm[n]=None
 for n in ['input_shaft','sun_gear','input_crank','crank_grip']:cm[n]='sun_input_hinge'
 for n in ['carrier','planet_pin_1','planet_pin_2','planet_pin_3']:cm[n]='carrier_output_hinge'
 for i in [1,2,3]:cm['planet_'+str(i)]='planet_'+str(i)+'_carrier_hinge'
 tree=[['base','lower_input_bearing'],['base','upper_input_bearing'],['base','carrier_bearing'],['base','ring_post_1'],['base','ring_post_2'],['base','ring_post_3'],['base','fixed_ring'],['base','input_shaft'],['input_shaft','sun_gear'],['input_shaft','input_crank'],['input_crank','crank_grip'],['base','carrier'],['carrier','planet_pin_1'],['carrier','planet_pin_2'],['carrier','planet_pin_3'],['carrier','planet_1'],['carrier','planet_2'],['carrier','planet_3']]
 T=[{'name':'planetary_carrier_ratio','driving_joint':'sun_input_hinge','driven_joint':'carrier_output_hinge','ratio':.25}]
 for i in [1,2,3]:
  q=str(i);T.append({'name':'planetary_planet_ratio_'+q,'driving_joint':'sun_input_hinge','driven_joint':'planet_'+q+'_carrier_hinge','ratio':-.75})
 out.topology_plan({'coordinate_map':cm,'tree_edges':tree,'closure_edges':[['sun_gear','carrier'],['sun_gear','planet_1'],['fixed_ring','planet_1'],['sun_gear','planet_2'],['fixed_ring','planet_2'],['sun_gear','planet_3'],['fixed_ring','planet_3']],'rigid_carried':[['input_shaft','sun_gear'],['input_shaft','input_crank'],['input_crank','crank_grip'],['carrier','planet_pin_1'],['carrier','planet_pin_2'],['carrier','planet_pin_3']],'independent_coaxial':[['input_shaft','carrier'],['sun_gear','carrier'],['sun_gear','fixed_ring']],'transmissions':T,'contact_decisions':C,'support_ground':['base'],'support_strategy':'Fixed base and supports remain ground-collidable; no support patch.'})
 out.body('base')
 for n in ['lower_input_bearing','upper_input_bearing','carrier_bearing','ring_post_1','ring_post_2','ring_post_3','fixed_ring']:out.body(n,'base')
 out.body('input_shaft','base');out.joint('input_shaft','sun_input_hinge',axis=(0,0,1),pos_mm=(0,0,18.2),frame='world');out.body('sun_gear','input_shaft');out.body('input_crank','input_shaft');out.body('crank_grip','input_crank');out.body('carrier','base');out.joint('carrier','carrier_output_hinge',axis=(0,0,1),pos_mm=(0,0,13.5),frame='world')
 for i in [1,2,3]:out.body('planet_pin_'+str(i),'carrier')
 A=[(18,0,18.2),(-9,15.588457268119896,18.2),(-9,-15.588457268119893,18.2)]
 for i in [1,2,3]:
  q=str(i);out.body('planet_'+q,'carrier');out.joint('planet_'+q,'planet_'+q+'_carrier_hinge',axis=(0,0,1),pos_mm=A[i-1],frame='world')
 st=['planetary_stage/four_to_one_planetary_stage'];out.joint_equality('planetary_carrier_ratio','sun_input_hinge','carrier_output_hinge',.25,reason='Fixed 54 tooth ring and 18 tooth sun give four to one reduction.',sources=st,fact_ids=st)
 for i in [1,2,3]:
  q=str(i);s=['transmission/sun_to_planet_'+q,'transmission/planet_to_internal_ring_'+q,'planetary_stage/four_to_one_planetary_stage'];out.joint_equality('planetary_planet_ratio_'+q,'sun_input_hinge','planet_'+q+'_carrier_hinge',-.75,reason='Fixed-ring planetary local planet spin.',sources=s,fact_ids=s)
 for c in C:
  if c['action']=='exclude':out.exclude(c['pair'][0],c['pair'][1],c['reason'],c['source_entity_ids'],c['fact_ids'])
 P=[('base','top'),('lower_input_bearing','journal'),('upper_input_bearing','journal'),('carrier_bearing','carrier_journal'),('input_shaft','shaft_axis'),('input_shaft','sun_seat'),('input_shaft','crank_seat'),('sun_gear','shaft_bore'),('sun_gear','sun_mesh'),('carrier','output_axis'),('carrier','pin_seat_1'),('carrier','pin_seat_2'),('carrier','pin_seat_3'),('fixed_ring','internal_mesh'),('input_crank','hub_bore'),('crank_grip','grip_axis'),('planet_1','pin_bore'),('planet_1','planet_mesh'),('planet_pin_1','pin_axis'),('planet_2','pin_bore'),('planet_2','planet_mesh'),('planet_pin_2','pin_axis'),('planet_3','pin_bore'),('planet_3','planet_mesh'),('planet_pin_3','pin_axis'),('ring_post_1','support_axis'),('ring_post_2','support_axis'),('ring_post_3','support_axis')]
 D=[]
 for n in L:D.append(('link/'+n,n,'emitted'));D.append(('pose/place_'+n,n,'represented_by'))
 for p in P:D.append(('port/'+p[0]+'/'+p[1],p[0],'represented_by'))
 for r in R:
  n='sun_input_hinge'
  if r[0]=='carrier_on_support_bearing' or r[0].startswith('carrier_pin_press_fit_'):n='carrier_output_hinge'
  if r[0].startswith('planet_pin_journal_') or r[0].startswith('sun_planet_mesh_') or r[0].startswith('ring_planet_mesh_'):n='planetary_planet_ratio_'+r[0][-1]
  D.append(('relation/'+r[0],n,'represented_by'))
 for j in ['sun_input_hinge','carrier_output_hinge','planet_1_carrier_hinge','planet_2_carrier_hinge','planet_3_carrier_hinge']:D.append(('motion_joint/'+j,j,'emitted'))
 for i in [1,2,3]:
  q=str(i);D.append(('transmission/sun_to_planet_'+q,'planetary_planet_ratio_'+q,'represented_by'));D.append(('transmission/planet_to_internal_ring_'+q,'planetary_planet_ratio_'+q,'represented_by'))
 D.append(('planetary_stage/four_to_one_planetary_stage','planetary_carrier_ratio','represented_by'));D.append(('role/driver/input_shaft','sun_input_hinge','represented_by'));D.append(('role/output/carrier','carrier_output_hinge','represented_by'))
 for n in ['sun_gear','planet_1','planet_2','planet_3','carrier','input_crank']:
  j='sun_input_hinge'
  if n=='carrier':j='carrier_output_hinge'
  if n.startswith('planet_'):j=n+'_carrier_hinge'
  D.append(('role/watch/'+n,j,'represented_by'))
 for d in D:out.decision(d[0],action=d[2],generated_nodes=[d[1]],reason='Represented by emitted topology, ideal coordinate, transmission, or contact classification.',fact_ids=[d[0]])
