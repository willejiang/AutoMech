def compile_mjcf(facts, out):
    links=['baseplate','output_lower_bearing','output_bearing_spacer','output_upper_bearing','output_shaft','carrier_plate','planet_pin_1','planet_thrust_washer_1','planet_pin_2','planet_thrust_washer_2','planet_pin_3','planet_thrust_washer_3','input_shaft','sun_gear','planet_1','planet_2','planet_3','ring_support_post_1','ring_support_post_2','ring_support_post_3','fixed_ring_gear','input_support_post_1','input_support_post_2','input_support_post_3','input_lower_bridge','input_lower_bearing','upper_bridge_spacer_1','upper_bridge_spacer_2','upper_bridge_spacer_3','input_upper_bridge','input_upper_bearing','hand_crank_arm','hand_crank_grip']
    ni={12:[13,20,24,25,29,30,31,4,5,16,15,14,3,2,9,11,8,10,7,32,6],13:[12,14,15,16,20,4,5,11,9,24,25,10,8,7,3,6,2,29,30,17,19],5:[4,6,7,8,9,10,11,12,13,14,15,16,20,3,17,19,2,24,25,21,23,1,18,0,26,28,29,30,22,31,27,32],4:[0,1,2,3,5,12,13,20,16,15,14,24,25,9,11],14:[6,7,13,20,5,24,25,4,12,15,16,3,2,29,30,17,19,9,11,8,10],15:[8,9,13,20,5,4,12,24,25,14,3,18,16,2,17,29,30,7,21,22,6],16:[10,11,13,20,5,4,12,24,25,14,3,18,15,2,19,29,30,7,22,23,6]}
    pairs={}
    for ai in ni:
        for bi in ni[ai]:
            a=links[ai];b=links[bi];p=(a,b) if a<b else (b,a);pairs[p]=1
    keep={('baseplate','output_shaft'):1,('carrier_plate','planet_thrust_washer_1'):1,('carrier_plate','planet_thrust_washer_2'):1,('carrier_plate','planet_thrust_washer_3'):1,('planet_1','planet_thrust_washer_1'):1,('planet_2','planet_thrust_washer_2'):1,('planet_3','planet_thrust_washer_3'):1}
    rels=[x for x in facts['entity_ids'] if x[:9]=='relation/']
    contact=[]
    for p in pairs:
        action='keep' if p in keep else 'exclude'
        reason='Measured touching interface retained for real support/thrust contact' if action=='keep' else 'Measured separation, running/press fit, or idealized gear mesh makes proxy contact nonphysical'
        contact.append({'pair':[p[0],p[1]],'action':action,'reason':reason,'source_entity_ids':['link/'+p[0],'link/'+p[1]]+rels,'fact_ids':['pair/'+p[0]+'/'+p[1]]})
    cmap={}
    for n in links:cmap[n]=None
    for n in ['input_shaft','sun_gear','hand_crank_arm','hand_crank_grip']:cmap[n]='sun_world_hinge'
    for n in ['carrier_plate','output_shaft','planet_pin_1','planet_thrust_washer_1','planet_pin_2','planet_thrust_washer_2','planet_pin_3','planet_thrust_washer_3']:cmap[n]='carrier_world_hinge'
    for i in [1,2,3]:cmap['planet_'+str(i)]='planet_'+str(i)+'_carrier_hinge'
    tree=[['','baseplate'],['','sun_gear'],['sun_gear','input_shaft'],['input_shaft','hand_crank_arm'],['hand_crank_arm','hand_crank_grip'],['','carrier_plate'],['carrier_plate','output_shaft']]
    for i in [1,2,3]:tree += [['carrier_plate','planet_pin_'+str(i)],['carrier_plate','planet_thrust_washer_'+str(i)],['carrier_plate','planet_'+str(i)]]
    mobile=['sun_gear','input_shaft','hand_crank_arm','hand_crank_grip','carrier_plate','output_shaft','planet_1','planet_2','planet_3','planet_pin_1','planet_pin_2','planet_pin_3','planet_thrust_washer_1','planet_thrust_washer_2','planet_thrust_washer_3','baseplate']
    fixed=[n for n in links if n not in mobile]
    for n in fixed:tree.append(['baseplate',n])
    tx=[{'name':'fixed_ring_reduction_stage','driving_joint':'sun_world_hinge','driven_joint':'carrier_world_hinge','ratio':0.25}]
    for i in [1,2,3]:tx.append({'name':'planet_spin_'+str(i),'driving_joint':'carrier_world_hinge','driven_joint':'planet_'+str(i)+'_carrier_hinge','ratio':-3.0})
    out.topology_plan({'coordinate_map':cmap,'tree_edges':tree,'closure_edges':[],'rigid_carried':[['input_shaft','sun_gear'],['carrier_plate','output_shaft'],['input_shaft','hand_crank_arm'],['hand_crank_arm','hand_crank_grip']], 'independent_coaxial':[['sun_gear','carrier_plate'],['planet_1','planet_2'],['planet_2','planet_3']], 'transmissions':tx,'contact_decisions':contact,'support_ground':'baseplate','support_strategy':'normal fixed base; no support patch required'})
    parent={}
    for e in tree:parent[e[1]]=e[0]
    order=['baseplate','sun_gear','input_shaft','hand_crank_arm','hand_crank_grip','carrier_plate','output_shaft','planet_pin_1','planet_thrust_washer_1','planet_1','planet_pin_2','planet_thrust_washer_2','planet_2','planet_pin_3','planet_thrust_washer_3','planet_3']+fixed
    for n in order:out.body(n,parent.get(n,''))
    out.joint('sun_gear','sun_world_hinge','hinge',(0,0,1),(0,0,35))
    out.joint('carrier_plate','carrier_world_hinge','hinge',(0,0,1),(0,0,28))
    pos=[(27,0,35),(-13.5,23.382685902179844,35),(-13.5,-23.38268590217984,35)]
    for i in [1,2,3]:out.joint('planet_'+str(i),'planet_'+str(i)+'_carrier_hinge','hinge',(0,0,1),pos[i-1])
    stage=['planetary_stage/fixed_ring_reduction_stage']
    out.joint_equality('fixed_ring_reduction','sun_world_hinge','carrier_world_hinge',0.25,0.0,'Fixed 54-tooth ring and 18-tooth sun give carrier/sun=1/4',stage,['pair/fixed_ring_gear/planet_1','pair/planet_1/sun_gear'])
    for i in [1,2,3]:out.joint_equality('planet_spin_'+str(i),'carrier_world_hinge','planet_'+str(i)+'_carrier_hinge',-3.0,0.0,'Planet local spin from fixed-ring kinematics',stage,['pair/fixed_ring_gear/planet_'+str(i),'pair/planet_'+str(i)+'/sun_gear'])
    for p in pairs:
        if p not in keep:out.exclude(p[0],p[1],'Exact measured separated pair or idealized press/running/gear interface',rels+['link/'+p[0],'link/'+p[1]],['pair/'+p[0]+'/'+p[1]])
    nodes=['body/'+n for n in links]+['joint/sun_world_hinge','joint/carrier_world_hinge','joint/planet_1_carrier_hinge','joint/planet_2_carrier_hinge','joint/planet_3_carrier_hinge','equality/fixed_ring_reduction','equality/planet_spin_1','equality/planet_spin_2','equality/planet_spin_3']
    for eid in facts['entity_ids']:out.decision(eid,'represented_by',nodes,'Authored entity represented by emitted topology, coordinate, or ideal relation',[eid])
