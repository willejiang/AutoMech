def compile_mjcf(facts, out):
    links = ['baseplate','central_lower_bearing','intermediate_lower_bearing','bridge_post_low','bridge_post_high','upper_bridge','central_upper_bearing','intermediate_upper_bearing','minute_shaft','intermediate_arbor','minute_pinion','intermediate_wheel','hour_pipe','intermediate_pinion','hour_wheel','dial_post_1','dial_post_2','dial_post_3','dial_post_4','chapter_ring'] + ['hour_marker_'+str(i) for i in range(1,13)] + ['hour_hand','minute_hand']
    fixed = ['central_lower_bearing','intermediate_lower_bearing','bridge_post_low','bridge_post_high','upper_bridge','central_upper_bearing','intermediate_upper_bearing','dial_post_1','dial_post_2','dial_post_3','dial_post_4','chapter_ring'] + ['hour_marker_'+str(i) for i in range(1,13)]
    carried = {'minute_pinion':'minute_shaft','minute_hand':'minute_shaft','intermediate_wheel':'intermediate_arbor','intermediate_pinion':'intermediate_arbor','hour_wheel':'hour_pipe','hour_hand':'hour_pipe'}
    cmap = {}
    for x in links: cmap[x] = None
    for x in ['minute_shaft','minute_pinion','minute_hand']: cmap[x]='minute_shaft_spin'
    for x in ['intermediate_arbor','intermediate_wheel','intermediate_pinion']: cmap[x]='intermediate_arbor_spin'
    for x in ['hour_pipe','hour_wheel','hour_hand']: cmap[x]='hour_pipe_spin'
    rel = [
      ('central_lower_bearing','minute_shaft','minute_shaft_in_lower_bearing','fit/central_lower_bearing/bore/minute_shaft/outer','pair/central_lower_bearing/minute_shaft','running journal represented by minute shaft hinge'),
      ('central_upper_bearing','hour_pipe','hour_pipe_in_upper_bearing','fit/central_upper_bearing/bore/hour_pipe/outer','pair/central_upper_bearing/hour_pipe','running journal represented by hour pipe hinge'),
      ('hour_pipe','minute_shaft','minute_shaft_inside_hour_pipe','fit/hour_pipe/inner_bore/minute_shaft/outer','pair/hour_pipe/minute_shaft','running coaxial clearance fit; coordinates remain independent'),
      ('intermediate_lower_bearing','intermediate_arbor','intermediate_arbor_in_lower_bearing','fit/intermediate_lower_bearing/bore/intermediate_arbor/outer','pair/intermediate_arbor/intermediate_lower_bearing','running journal represented by arbor hinge'),
      ('intermediate_upper_bearing','intermediate_arbor','intermediate_arbor_in_upper_bearing','fit/intermediate_upper_bearing/bore/intermediate_arbor/outer','pair/intermediate_arbor/intermediate_upper_bearing','running journal represented by arbor hinge'),
      ('minute_shaft','minute_pinion','minute_pinion_press_fit','fit/minute_shaft/minute_pinion_seat/minute_pinion/bore','pair/minute_pinion/minute_shaft','declared press fit rigidly carried'),
      ('intermediate_arbor','intermediate_wheel','intermediate_wheel_press_fit','fit/intermediate_arbor/wheel_seat/intermediate_wheel/bore','pair/intermediate_arbor/intermediate_wheel','declared press fit rigidly carried'),
      ('intermediate_arbor','intermediate_pinion','intermediate_pinion_press_fit','fit/intermediate_arbor/pinion_seat/intermediate_pinion/bore','pair/intermediate_arbor/intermediate_pinion','declared press fit rigidly carried'),
      ('hour_pipe','hour_wheel','hour_wheel_press_fit','fit/hour_pipe/hour_wheel_seat/hour_wheel/bore','pair/hour_pipe/hour_wheel','declared press fit rigidly carried'),
      ('hour_pipe','hour_hand','hour_hand_press_fit','fit/hour_pipe/hour_hand_seat/hour_hand/hub_bore','pair/hour_hand/hour_pipe','declared press fit rigidly carried'),
      ('minute_shaft','minute_hand','minute_hand_press_fit','fit/minute_shaft/minute_hand_seat/minute_hand/hub_bore','pair/minute_hand/minute_shaft','declared press fit rigidly carried'),
      ('minute_pinion','intermediate_wheel','minute_stage_mesh','fit/minute_pinion/mesh/intermediate_wheel/mesh','pair/intermediate_wheel/minute_pinion','ideal external gear equality replaces tooth contact'),
      ('hour_wheel','intermediate_pinion','hour_stage_mesh','fit/hour_wheel/mesh/intermediate_pinion/mesh','pair/hour_wheel/intermediate_pinion','ideal external gear equality replaces tooth contact')]
    contacts=[]
    for a,b,n,f,p,r in rel: contacts.append({'pair':[a,b],'action':'exclude','reason':r,'source_entity_ids':['relation/'+n],'fact_ids':[f,p]})
    out.topology_plan({'coordinate_map':cmap,'tree_edges':[['','baseplate']]+[['baseplate',x] for x in fixed]+[['baseplate','minute_shaft'],['baseplate','intermediate_arbor'],['baseplate','hour_pipe']]+[[v,k] for k,v in carried.items()],'closure_edges':[],'rigid_carried':carried,'independent_coaxial':[['minute_shaft','hour_pipe']],'transmissions':[{'driving_joint':'minute_shaft_spin','driven_joint':'intermediate_arbor_spin','ratio':-0.3333333333333333},{'driving_joint':'intermediate_arbor_spin','driven_joint':'hour_pipe_spin','ratio':-0.25}],'contact_decisions':contacts,'support_ground':'baseplate rests on global plane','support_strategy':'fixed baseplate is the structural gravity support; no support patch required'})
    out.body('baseplate')
    for x in fixed: out.body(x,'baseplate')
    out.body('minute_shaft','baseplate'); out.joint('minute_shaft','minute_shaft_spin',axis=(0,0,1),pos_mm=(0,0,2),frame='world')
    out.body('minute_pinion','minute_shaft'); out.body('minute_hand','minute_shaft')
    out.body('intermediate_arbor','baseplate'); out.joint('intermediate_arbor','intermediate_arbor_spin',axis=(0,0,1),pos_mm=(12,0,2),frame='world')
    out.body('intermediate_wheel','intermediate_arbor'); out.body('intermediate_pinion','intermediate_arbor')
    out.body('hour_pipe','baseplate'); out.joint('hour_pipe','hour_pipe_spin',axis=(0,0,1),pos_mm=(0,0,6.5),frame='world')
    out.body('hour_wheel','hour_pipe'); out.body('hour_hand','hour_pipe')
    out.joint_equality('minute_to_intermediate_eq','minute_shaft_spin','intermediate_arbor_spin',-0.3333333333333333,reason='external 3 mm to 9 mm pitch-radius mesh',sources=['relation/minute_stage_mesh','transmission/minute_to_intermediate'],fact_ids=['fit/minute_pinion/mesh/intermediate_wheel/mesh'])
    out.joint_equality('intermediate_to_hour_eq','intermediate_arbor_spin','hour_pipe_spin',-0.25,reason='rigid compound arbor drives external 2.4 mm to 9.6 mm mesh',sources=['relation/hour_stage_mesh','transmission/compound_intermediate_arbor','transmission/intermediate_to_hour'],fact_ids=['fit/hour_wheel/mesh/intermediate_pinion/mesh'])
    for a,b,n,f,p,r in rel: out.exclude(a,b,r,['relation/'+n],[f,p])
    generated={}
    for x in links: generated['link/'+x]=[x]
    for x in links: generated['pose/place_'+x]=[x]
    port_owners={'central_lower_bearing':['bore'],'central_upper_bearing':['bore'],'intermediate_lower_bearing':['bore'],'intermediate_upper_bearing':['bore'],'minute_shaft':['outer','minute_pinion_seat','minute_hand_seat'],'hour_pipe':['inner_bore','outer','hour_wheel_seat','hour_hand_seat'],'intermediate_arbor':['outer','wheel_seat','pinion_seat'],'minute_pinion':['bore','mesh'],'intermediate_wheel':['bore','mesh'],'intermediate_pinion':['bore','mesh'],'hour_wheel':['bore','mesh'],'hour_hand':['hub_bore'],'minute_hand':['hub_bore']}
    for x in port_owners:
        for p in port_owners[x]: generated['port/'+x+'/'+p]=[x]
    for a,b,n,f,p,r in rel: generated['relation/'+n]=[b]
    generated['relation/minute_stage_mesh']=['minute_to_intermediate_eq']; generated['relation/hour_stage_mesh']=['intermediate_to_hour_eq']
    generated['transmission/minute_to_intermediate']=['minute_to_intermediate_eq']; generated['transmission/compound_intermediate_arbor']=['intermediate_arbor_spin']; generated['transmission/intermediate_to_hour']=['intermediate_to_hour_eq']
    generated['role/driver/minute_shaft']=['minute_shaft_spin']; generated['role/output/hour_hand']=['hour_pipe_spin']
    for x in ['minute_hand','hour_hand','minute_pinion','intermediate_wheel','intermediate_pinion','hour_wheel']: generated['role/watch/'+x]=[cmap[x]]
    for eid in facts['entity_ids']: out.decision(eid,'represented_by',generated[eid],'represented in emitted body tree, rigid carrying, bearing coordinate, or ideal transmission',[eid])
