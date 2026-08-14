def compile_mjcf(facts, out):
    z=(0.0,0.0,1.0)
    tree=[('baseplate',''),('lower_bearing_housing','baseplate'),('lower_input_bearing','lower_bearing_housing'),('ring_seat','baseplate'),('support_post_1','baseplate'),('support_post_2','baseplate'),('support_post_3','baseplate'),('fixed_ring_gear','ring_seat'),('upper_bearing_bridge','support_post_1'),('upper_output_bearing','upper_bearing_bridge'),('input_shaft','baseplate'),('sun_gear','input_shaft'),('hand_crank_arm','input_shaft'),('hand_grip','hand_crank_arm'),('carrier','baseplate'),('planet_pin_1','carrier'),('planet_thrust_washer_1','planet_pin_1'),('planet_gear_1','carrier'),('planet_pin_2','carrier'),('planet_thrust_washer_2','planet_pin_2'),('planet_gear_2','carrier'),('planet_pin_3','carrier'),('planet_thrust_washer_3','planet_pin_3'),('planet_gear_3','carrier'),('output_shaft','carrier'),('output_sleeve_bearing','output_shaft'),('output_pointer','output_shaft')]
    cmap={'baseplate':None,'lower_bearing_housing':None,'lower_input_bearing':None,'ring_seat':None,'support_post_1':None,'support_post_2':None,'support_post_3':None,'fixed_ring_gear':None,'upper_bearing_bridge':None,'upper_output_bearing':None,'input_shaft':'input_hinge','sun_gear':'input_hinge','hand_crank_arm':'input_hinge','hand_grip':'input_hinge','carrier':'carrier_hinge','planet_pin_1':'carrier_hinge','planet_thrust_washer_1':'carrier_hinge','planet_gear_1':'planet_1_carrier_hinge','planet_pin_2':'carrier_hinge','planet_thrust_washer_2':'carrier_hinge','planet_gear_2':'planet_2_carrier_hinge','planet_pin_3':'carrier_hinge','planet_thrust_washer_3':'carrier_hinge','planet_gear_3':'planet_3_carrier_hinge','output_shaft':'carrier_hinge','output_sleeve_bearing':'carrier_hinge','output_pointer':'carrier_hinge'}
    ex=[]
    ex.append(('input_shaft','sun_gear','press fit and compound 1:1 replace contact',['relation/sun_press_fit','transmission/input_shaft_to_sun'],['fit/input_shaft/sun_seat/sun_gear/bore','pair/input_shaft/sun_gear']))
    ex.append(('lower_input_bearing','input_shaft','clearance journal is represented by input hinge',['relation/input_lower_journal'],['fit/lower_input_bearing/journal/input_shaft/shaft_axis','pair/input_shaft/lower_input_bearing']))
    ex.append(('output_sleeve_bearing','input_shaft','clearance sleeve journal permits independent coaxial rotation',['relation/input_upper_sleeve_journal'],['fit/output_sleeve_bearing/input_journal/input_shaft/shaft_axis','pair/input_shaft/output_sleeve_bearing']))
    ex.append(('carrier','output_shaft','interference press fit rigidly carries output shaft',['relation/carrier_output_press_fit','transmission/carrier_to_output_shaft'],['fit/carrier/output_bore/output_shaft/outer','pair/carrier/output_shaft']))
    ex.append(('upper_output_bearing','output_shaft','clearance journal is represented by carrier hinge',['relation/output_upper_journal'],['fit/upper_output_bearing/journal/output_shaft/outer','pair/output_shaft/upper_output_bearing']))
    for i in (1,2,3):
        s=str(i)
        ex.append(('planet_gear_'+s,'planet_pin_'+s,'clearance pin journal is represented by local planet hinge',['relation/planet_'+s+'_pin_journal'],['fit/planet_gear_'+s+'/bore/planet_pin_'+s+'/pin_axis','pair/planet_gear_'+s+'/planet_pin_'+s]))
        ex.append(('sun_gear','planet_gear_'+s,'ideal external gear equality replaces tooth contact',['relation/sun_planet_mesh_'+s],['fit/sun_gear/pitch/planet_gear_'+s+'/pitch','pair/planet_gear_'+s+'/sun_gear']))
        ex.append(('fixed_ring_gear','planet_gear_'+s,'fixed-ring equality replaces internal tooth contact',['planetary_stage/fixed_ring_stage'],['fit/fixed_ring_gear/pitch/planet_gear_'+s+'/pitch','pair/fixed_ring_gear/planet_gear_'+s]))
        ex.append(('carrier','planet_pin_'+s,'measured interference fit rigidly carries dedicated pin',['port/carrier/pin_bore_'+s,'port/planet_pin_'+s+'/pin_axis'],['fit/carrier/pin_bore_'+s+'/planet_pin_'+s+'/pin_axis','pair/carrier/planet_pin_'+s]))
    cd=[]
    for v in ex: cd.append({'pair':[v[0],v[1]],'action':'exclude','reason':v[2],'source_entity_ids':v[3],'fact_ids':v[4]})
    plan={'coordinate_map':cmap,'tree_edges':tree,'closure_edges':[],'rigid_carried':[('input_shaft','sun_gear'),('carrier','output_shaft'),('carrier','planet_pin_1'),('carrier','planet_pin_2'),('carrier','planet_pin_3'),('output_shaft','output_sleeve_bearing'),('output_shaft','output_pointer'),('input_shaft','hand_crank_arm'),('hand_crank_arm','hand_grip')],'independent_coaxial':[('input_shaft','carrier'),('input_shaft','output_shaft'),('input_shaft','output_sleeve_bearing'),('sun_gear','fixed_ring_gear')],'transmissions':[{'name':'input_shaft_to_sun','ratio':1.0},{'name':'fixed_ring_stage','sun_to_carrier':0.25,'planet_local_to_sun':-0.75},{'name':'carrier_to_output_shaft','ratio':1.0}],'contact_decisions':cd,'support_ground':'baseplate rests on global plane','support_strategy':'normal base fixed; support probe releases baseplate onto plane'}
    out.topology_plan(plan)
    for b,p in tree: out.body(b,p)
    out.joint('input_shaft','input_hinge','hinge',z,(0.0,0.0,0.0),'world')
    out.joint('carrier','carrier_hinge','hinge',z,(0.0,0.0,0.0),'world')
    pp=((21.6,0.0,0.0),(-10.8,18.706148721743876,0.0),(-10.8,-18.70614872174387,0.0))
    for i in (1,2,3): out.joint('planet_gear_'+str(i),'planet_'+str(i)+'_carrier_hinge','hinge',z,pp[i-1],'world')
    out.joint_equality('fixed_ring_sun_carrier','input_hinge','carrier_hinge',0.25,0.0,'fixed 54-tooth ring and 18-tooth sun impose Willis carrier ratio',['planetary_stage/fixed_ring_stage'],['fit/fixed_ring_gear/pitch/planet_gear_1/pitch','fit/sun_gear/pitch/planet_gear_1/pitch'])
    for i in (1,2,3):
        s=str(i); out.joint_equality('planet_'+s+'_spin','input_hinge','planet_'+s+'_carrier_hinge',-0.75,0.0,'external sun mesh and fixed ring impose local planet spin',['relation/sun_planet_mesh_'+s,'planetary_stage/fixed_ring_stage'],['fit/sun_gear/pitch/planet_gear_'+s+'/pitch','fit/fixed_ring_gear/pitch/planet_gear_'+s+'/pitch'])
    for v in ex: out.exclude(v[0],v[1],v[2],v[3],v[4])
    out.support_patch('free_body','baseplate','release normally fixed base only for gravity support probe')
    rmap={'sun_press_fit':'input_hinge','input_lower_journal':'input_hinge','input_upper_sleeve_journal':'input_hinge','carrier_output_press_fit':'carrier_hinge','output_upper_journal':'carrier_hinge','planet_1_pin_journal':'planet_1_carrier_hinge','planet_2_pin_journal':'planet_2_carrier_hinge','planet_3_pin_journal':'planet_3_carrier_hinge','sun_planet_mesh_1':'planet_1_spin','sun_planet_mesh_2':'planet_2_spin','sun_planet_mesh_3':'planet_3_spin'}
    for e in facts['entity_ids']:
        action='represented_by'; nodes=[]
        if e[:5]=='link/': action='emitted'; nodes=[e[5:]]
        elif e[:13]=='motion_joint/': action='emitted'; nodes=[e[13:]]
        elif e[:11]=='pose/place_': nodes=[e[11:]]
        elif e[:5]=='port/':
            n=''
            for c in e[5:]:
                if c=='/': break
                n=n+c
            nodes=[n]
        elif e[:9]=='relation/': nodes=[rmap[e[9:]]]
        elif e=='transmission/input_shaft_to_sun': nodes=['input_hinge']
        elif e=='transmission/carrier_to_output_shaft': nodes=['carrier_hinge']
        elif e=='planetary_stage/fixed_ring_stage': nodes=['fixed_ring_sun_carrier']
        elif e[:11]=='role/watch/': nodes=[cmap[e[11:]]]
        elif e=='role/driver/input_shaft': nodes=['input_hinge']
        elif e=='role/output/carrier': nodes=['carrier_hinge']
        out.decision(e,action,nodes,'authored entity emitted or represented by selected topology, measured fit, contact decision, or transmission',[e])
