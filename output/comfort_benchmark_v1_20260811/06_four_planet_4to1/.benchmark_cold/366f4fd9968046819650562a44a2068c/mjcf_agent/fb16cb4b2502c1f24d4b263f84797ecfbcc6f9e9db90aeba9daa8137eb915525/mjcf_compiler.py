def compile_mjcf(facts,out):
 links=[]
 for n in facts['links']:
  links.append(n)
 ideal={}
 ideal['input_bearing|input_shaft']=['running journal represented by hinge',['relation/input_shaft_in_input_bearing'],['fit/input_bearing/input_bore/input_shaft/shaft_axis','pair/input_bearing/input_shaft']]
 ideal['output_bearing|output_sleeve']=['running journal represented by hinge',['relation/output_sleeve_in_output_bearing'],['fit/output_bearing/output_bore/output_sleeve/outer_journal','pair/output_bearing/output_sleeve']]
 ideal['input_shaft|sun_gear']=['declared press fit rigidly carried',['relation/sun_press_fit','transmission/input_shaft_to_sun'],['fit/input_shaft/sun_seat/sun_gear/shaft_bore','pair/input_shaft/sun_gear']]
 ideal['output_sleeve|carrier']=['declared press fit rigidly carried',['relation/carrier_output_press_fit','transmission/carrier_to_output_sleeve'],['fit/output_sleeve/carrier_seat/carrier/output_bore','pair/carrier/output_sleeve']]
 ideal['input_shaft|crank_arm']=['fixed aligned crank seat rigidly carried',['link/input_shaft','link/crank_arm'],['fit/input_shaft/crank_seat/crank_arm/drive_bore','pair/crank_arm/input_shaft']]
 ideal['crank_arm|crank_handle']=['fixed accessory rigidly carried',['link/crank_arm','link/crank_handle'],['pair/crank_arm/crank_handle']]
 for i in range(1,5):
  s=str(i)
  ideal['planet_pin_'+s+'|planet_'+s]=['running journal represented by planet hinge',['relation/planet_'+s+'_journal'],['fit/planet_pin_'+s+'/pin_shaft/planet_'+s+'/pin_bore','pair/planet_'+s+'/planet_pin_'+s]]
  ideal['carrier|planet_pin_'+s]=['measured interference pin seat rigidly carried',['link/carrier','link/planet_pin_'+s],['fit/carrier/pin_seat_'+s+'/planet_pin_'+s+'/pin_shaft','pair/carrier/planet_pin_'+s]]
  ideal['sun_gear|planet_'+s]=['ideal external tooth mesh replaces contact',['planetary_stage/four_planet_4to1_stage'],['fit/sun_gear/pitch_axis/planet_'+s+'/pitch_axis','pair/planet_'+s+'/sun_gear']]
  ideal['fixed_ring_gear|planet_'+s]=['ideal internal tooth mesh replaces contact',['planetary_stage/four_planet_4to1_stage'],['fit/fixed_ring_gear/ring_pitch_axis/planet_'+s+'/pitch_axis','pair/fixed_ring_gear/planet_'+s]]
 contacts=[]
 for i in range(0,len(links)):
  for j in range(i+1,len(links)):
   a=links[i];b=links[j];k=a+'|'+b;r=b+'|'+a
   if k in ideal:q=ideal[k]
   elif r in ideal:q=ideal[r]
   else:q=['contact retained because solid intersection is unavailable or this pair may require ordinary collision response',['link/'+a,'link/'+b],['pair/'+a+'/'+b]]
   contacts.append({'pair':[a,b],'action':'exclude' if k in ideal or r in ideal else 'keep','reason':q[0],'source_entity_ids':q[1],'fact_ids':q[2]})
 cmap={}
 for n in links:cmap[n]=None
 for n in ['input_shaft','sun_gear','crank_arm','crank_handle']:cmap[n]='input_shaft_hinge'
 for n in ['output_sleeve','carrier','planet_pin_1','planet_pin_2','planet_pin_3','planet_pin_4']:cmap[n]='output_sleeve_hinge'
 for i in range(1,5):cmap['planet_'+str(i)]='planet_'+str(i)+'_carrier_hinge'
 out.topology_plan({'coordinate_map':cmap,'tree_edges':[['base','input_bearing'],['base','output_bearing'],['input_bearing','input_shaft'],['output_bearing','output_sleeve'],['output_sleeve','carrier'],['carrier','planet pins'],['planet pins','planets']],'closure_edges':[['input_shaft_hinge','output_sleeve_hinge'],['input_shaft_hinge','planet hinges']],'rigid_carried':['sun by input','carrier by sleeve','pins by carrier','crank by input'],'independent_coaxial':['input and output before planetary equality','planet spin relative to carrier'],'transmissions':[{'driving':'input_shaft_hinge','driven':'output_sleeve_hinge','ratio':0.25},{'driving':'input_shaft_hinge','driven':'planet hinges','ratio':-0.75}],'contact_decisions':contacts,'support_ground':{'body':'base','ground_contact':'keep'},'support_strategy':{'normal':'fixed base','patches':[]}})
 out.body('base')
 for b in ['input_bearing','output_bearing','ring_post_1','ring_post_2','ring_post_3','ring_post_4']:out.body(b,'base')
 out.body('fixed_ring_gear','ring_post_1')
 out.body('input_shaft','input_bearing');out.joint('input_shaft','input_shaft_hinge',axis=(0,0,1),pos_mm=(0,0,0),frame='world')
 out.body('sun_gear','input_shaft');out.body('crank_arm','input_shaft');out.body('crank_handle','crank_arm')
 out.body('output_sleeve','output_bearing');out.joint('output_sleeve','output_sleeve_hinge',axis=(0,0,1),pos_mm=(0,0,0),frame='world');out.body('carrier','output_sleeve')
 pp=[(16,0,0),(0,16,0),(-16,0,0),(0,-16,0)]
 for i in range(1,5):
  s=str(i);out.body('planet_pin_'+s,'carrier');out.body('planet_'+s,'planet_pin_'+s);out.joint('planet_'+s,'planet_'+s+'_carrier_hinge',axis=(0,0,1),pos_mm=pp[i-1],frame='world')
 out.joint_equality('planetary_carrier_ratio','input_shaft_hinge','output_sleeve_hinge',.25,reason='fixed 48 tooth ring and 16 tooth sun',sources=['planetary_stage/four_planet_4to1_stage'],fact_ids=['planetary_stage/four_planet_4to1_stage'])
 for i in range(1,5):
  s=str(i);out.joint_equality('planet_'+s+'_mesh','input_shaft_hinge','planet_'+s+'_carrier_hinge',-.75,reason='sun and fixed ring planetary mesh kinematics',sources=['planetary_stage/four_planet_4to1_stage'],fact_ids=['planetary_stage/four_planet_4to1_stage'])
 for k in ideal:
  a='';b='';bar=False
  for ch in k:
   if ch=='|':bar=True
   elif bar:b=b+ch
   else:a=a+ch
  q=ideal[k];out.exclude(a,b,q[0],q[1],q[2])
 nodes={'motion_joint/input_shaft_hinge':['input_shaft_hinge'],'motion_joint/sun_hinge':['input_shaft_hinge'],'motion_joint/carrier_hinge':['output_sleeve_hinge'],'motion_joint/output_sleeve_hinge':['output_sleeve_hinge'],'transmission/input_shaft_to_sun':['input_shaft_hinge'],'transmission/carrier_to_output_sleeve':['output_sleeve_hinge'],'planetary_stage/four_planet_4to1_stage':['planetary_carrier_ratio']}
 for i in range(1,5):nodes['motion_joint/planet_'+str(i)+'_carrier_hinge']=['planet_'+str(i)+'_carrier_hinge']
 for eid in facts['entity_ids']:
  if eid in nodes:g=nodes[eid]
  elif eid[:5]=='link/':g=[eid[5:]]
  else:g=['base']
  out.decision(eid,action='emitted' if eid[:5]=='link/' or eid in nodes else 'represented_by',generated_nodes=g,reason='represented by emitted topology node',fact_ids=[eid])
