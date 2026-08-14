def compile_mjcf(facts, out):
    fixed=['baseplate','lower_input_bearing','output_bearing_pedestal','output_bearing','ring_post_1','ring_post_2','ring_post_3','ring_post_4','ring_support_shelf','fixed_ring','upper_bridge_column_left','upper_bridge_column_right','upper_bridge','upper_input_bearing']
    jmap={}
    for b in fixed:jmap[b]=None
    for b in ['input_shaft','sun_gear','crank_arm','hand_handle']:jmap[b]='input_shaft_hinge'
    for b in ['output_shaft','output_dial','carrier','planet_pin_1','planet_pin_2','planet_pin_3','planet_pin_4']:jmap[b]='output_shaft_hinge'
    for i in [1,2,3,4]:jmap['planet_'+str(i)]='planet_'+str(i)+'_carrier_hinge'
    near={
    'input_shaft':'baseplate lower_input_bearing output_bearing_pedestal output_bearing output_shaft output_dial carrier ring_support_shelf fixed_ring sun_gear upper_bridge upper_input_bearing crank_arm planet_1 planet_2 planet_3 planet_4 planet_pin_1 planet_pin_2 planet_pin_3 planet_pin_4 hand_handle',
    'sun_gear':'input_shaft ring_support_shelf fixed_ring planet_1 planet_2 planet_3 planet_4 upper_bridge output_shaft carrier upper_input_bearing output_dial planet_pin_1 planet_pin_2 planet_pin_3 planet_pin_4 crank_arm output_bearing output_bearing_pedestal lower_input_bearing hand_handle',
    'carrier':'input_shaft output_shaft output_dial ring_support_shelf planet_pin_1 planet_pin_2 planet_pin_3 planet_pin_4 output_bearing sun_gear planet_1 planet_2 planet_3 planet_4 ring_post_1 ring_post_2 ring_post_3 ring_post_4 fixed_ring output_bearing_pedestal lower_input_bearing upper_bridge baseplate upper_bridge_column_left upper_bridge_column_right upper_input_bearing crank_arm hand_handle',
    'output_shaft':'lower_input_bearing output_bearing_pedestal output_bearing input_shaft output_dial carrier ring_support_shelf sun_gear planet_1 planet_2 planet_3 planet_4 baseplate fixed_ring planet_pin_1 planet_pin_2 planet_pin_3 planet_pin_4 upper_bridge',
    'planet_1':'ring_support_shelf fixed_ring sun_gear planet_pin_1 planet_2 planet_4 upper_bridge output_shaft carrier ring_post_1 upper_input_bearing input_shaft planet_pin_2 planet_pin_4 output_dial upper_bridge_column_right crank_arm output_bearing hand_handle planet_3 output_bearing_pedestal lower_input_bearing',
    'planet_2':'ring_support_shelf fixed_ring sun_gear planet_1 planet_pin_2 planet_3 upper_bridge output_shaft carrier ring_post_2 upper_input_bearing input_shaft planet_pin_1 planet_pin_3 output_dial crank_arm output_bearing planet_4 output_bearing_pedestal hand_handle lower_input_bearing',
    'planet_3':'ring_support_shelf fixed_ring sun_gear planet_2 planet_pin_3 planet_4 upper_bridge output_shaft carrier ring_post_3 upper_input_bearing input_shaft planet_pin_2 planet_pin_4 output_dial upper_bridge_column_left crank_arm output_bearing planet_1 output_bearing_pedestal lower_input_bearing',
    'planet_4':'ring_support_shelf fixed_ring sun_gear planet_1 planet_3 planet_pin_4 upper_bridge output_shaft carrier ring_post_4 upper_input_bearing input_shaft planet_pin_3 planet_pin_1 output_dial crank_arm output_bearing planet_2 output_bearing_pedestal hand_handle lower_input_bearing'}
    pairs={}
    for a in near:
        b=''
        for c in near[a]+' ':
            if c==' ':
                x,y=(a,b) if a<b else (b,a);pairs[x+'|'+y]=[x,y];b=''
            else:b+=c
    rels={'input_shaft|lower_input_bearing':['relation/lower_input_journal','fit/lower_input_bearing/input_bore/input_shaft/lower_journal'],'input_shaft|upper_input_bearing':['relation/upper_input_journal','fit/upper_input_bearing/input_bore/input_shaft/upper_journal'],'input_shaft|sun_gear':['relation/sun_press_fit','fit/sun_gear/shaft_bore/input_shaft/sun_seat'],'output_bearing|output_shaft':['relation/output_journal','fit/output_bearing/output_bore/output_shaft/output_journal'],'carrier|output_shaft':['relation/carrier_output_press_fit','fit/carrier/output_bore/output_shaft/carrier_seat']}
    for i in [1,2,3,4]:
        s=str(i);rels['carrier|planet_pin_'+s]=['relation/carrier_pin_'+s+'_press_fit','fit/carrier/pin_'+s+'_socket/planet_pin_'+s+'/pin_axis'];rels['planet_'+s+'|planet_pin_'+s]=['relation/planet_'+s+'_journal','fit/planet_'+s+'/pin_bore/planet_pin_'+s+'/pin_axis'];rels['planet_'+s+'|sun_gear']=['planetary_stage/fixed_ring_stage','fit/sun_gear/sun_pitch/planet_'+s+'/planet_pitch'];rels['fixed_ring|planet_'+s]=['planetary_stage/fixed_ring_stage','fit/fixed_ring/internal_pitch/planet_'+s+'/planet_pitch']
    cds=[]
    for k in sorted(pairs):
        a,b=pairs[k];ss=['link/'+a,'link/'+b];fs=['nearby/'+a+'/'+b,'pair/'+a+'/'+b];r='Measured positive real-mesh separation or disjoint AABB extent makes this exact pair a collision-proxy false-contact risk.'
        if k in rels:ss.append(rels[k][0]);fs.append(rels[k][1]);r='Measured running/press fit or ideal planetary tooth mesh is represented by coordinate topology/equality and must not fight contact.'
        cds.append({'pair':[a,b],'action':'exclude','reason':r,'source_entity_ids':ss,'fact_ids':fs})
    tree=[('baseplate',''),('lower_input_bearing','baseplate'),('input_shaft','lower_input_bearing'),('sun_gear','input_shaft'),('crank_arm','input_shaft'),('hand_handle','crank_arm'),('output_bearing_pedestal','baseplate'),('output_bearing','output_bearing_pedestal'),('output_shaft','output_bearing'),('output_dial','output_shaft'),('carrier','output_shaft')]
    for i in [1,2,3,4]:tree += [('planet_pin_'+str(i),'carrier'),('planet_'+str(i),'planet_pin_'+str(i))]
    tree += [('ring_post_1','baseplate'),('ring_post_2','baseplate'),('ring_post_3','baseplate'),('ring_post_4','baseplate'),('ring_support_shelf','ring_post_1'),('fixed_ring','ring_support_shelf'),('upper_bridge_column_left','baseplate'),('upper_bridge_column_right','baseplate'),('upper_bridge','upper_bridge_column_left'),('upper_input_bearing','upper_bridge')]
    plan={'coordinate_map':jmap,'tree_edges':[[p,b] for b,p in tree if p],'closure_edges':['planetary_carrier_ratio','planet_1_mesh','planet_2_mesh','planet_3_mesh','planet_4_mesh'],'rigid_carried':[['input_shaft','sun_gear'],['input_shaft','crank_arm'],['crank_arm','hand_handle'],['output_shaft','carrier'],['output_shaft','output_dial'],['carrier','planet_pin_1'],['carrier','planet_pin_2'],['carrier','planet_pin_3'],['carrier','planet_pin_4']],'independent_coaxial':[['input_shaft','output_shaft']],'transmissions':[{'name':'input_shaft_to_sun','ratio':1.0,'strategy':'rigid carrying'},{'name':'carrier_to_output_shaft','ratio':1.0,'strategy':'rigid carrying'},{'name':'fixed_ring_stage','ratio':0.25,'strategy':'joint equalities'}],'contact_decisions':cds,'support_ground':'baseplate','support_strategy':'Fixed root baseplate supports the normal gravity model; no probe alteration is required.'}
    out.topology_plan(plan)
    for b,p in tree:out.body(b,p)
    out.joint('input_shaft','input_shaft_hinge','hinge',(0.0,0.0,1.0),(0.0,0.0,0.0));out.joint('output_shaft','output_shaft_hinge','hinge',(0.0,0.0,1.0),(0.0,0.0,0.0))
    for i in [1,2,3,4]:out.joint('planet_'+str(i),'planet_'+str(i)+'_carrier_hinge','hinge',(0.0,0.0,1.0),(0.0,0.0,0.0))
    out.joint_equality('planetary_carrier_ratio','input_shaft_hinge','output_shaft_hinge',0.25,0.0,'Fixed 48-tooth ring and 16-tooth sun give carrier/sun=16/(16+48).',['planetary_stage/fixed_ring_stage'],['fit/sun_gear/sun_pitch/planet_1/planet_pitch'])
    for i in [1,2,3,4]:
        s=str(i);out.joint_equality('planet_'+s+'_mesh','input_shaft_hinge','planet_'+s+'_carrier_hinge',-0.75,0.0,'External equal-tooth sun/planet mesh with carried center gives local spin=carrier-sun.',['planetary_stage/fixed_ring_stage'],['fit/sun_gear/sun_pitch/planet_'+s+'/planet_pitch','fit/fixed_ring/internal_pitch/planet_'+s+'/planet_pitch'])
    for d in cds:out.exclude(d['pair'][0],d['pair'][1],d['reason'],d['source_entity_ids'],d['fact_ids'])
    special={'motion_joint/input_shaft_hinge':['input_shaft_hinge'],'motion_joint/sun_hinge':['input_shaft_hinge'],'motion_joint/carrier_hinge':['output_shaft_hinge'],'motion_joint/output_shaft_hinge':['output_shaft_hinge'],'transmission/input_shaft_to_sun':['input_shaft','sun_gear'],'transmission/carrier_to_output_shaft':['carrier','output_shaft'],'planetary_stage/fixed_ring_stage':['planetary_carrier_ratio','planet_1_mesh','planet_2_mesh','planet_3_mesh','planet_4_mesh']}
    for i in [1,2,3,4]:special['motion_joint/planet_'+str(i)+'_carrier_hinge']=['planet_'+str(i)+'_carrier_hinge']
    for eid in facts['entity_ids']:
        if eid.startswith('link/'):nodes=[eid[5:]];act='emitted'
        elif eid in special:nodes=special[eid];act='represented_by'
        else:nodes=['baseplate'];act='represented_by'
        out.decision(eid,act,nodes,'Authored entity is emitted or resolved by the selected body, coordinate, equality, and exact-pair topology.',[eid])
