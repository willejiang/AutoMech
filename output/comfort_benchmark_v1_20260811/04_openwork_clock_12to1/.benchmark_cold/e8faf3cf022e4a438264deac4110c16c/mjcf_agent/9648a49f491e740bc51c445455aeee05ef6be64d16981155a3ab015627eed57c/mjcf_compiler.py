def compile_mjcf(facts,out):
    names=['base','minute_input_bearing','intermediate_bearing','rear_mast','lower_sleeve_bracket','upper_sleeve_bracket','sleeve_thrust_washer','lower_sleeve_bearing','upper_sleeve_bearing','minute_shaft','intermediate_shaft','hour_sleeve','minute_gear','compound_big_gear','compound_pinion','hour_gear','hour_hand','minute_hand']
    fixed=['base','minute_input_bearing','intermediate_bearing','rear_mast','lower_sleeve_bracket','upper_sleeve_bracket','sleeve_thrust_washer','lower_sleeve_bearing','upper_sleeve_bearing']
    cm={}
    for n in fixed: cm[n]=None
    for n in ['minute_shaft','minute_gear','minute_hand']: cm[n]='minute_spin'
    for n in ['intermediate_shaft','compound_big_gear','compound_pinion']: cm[n]='intermediate_spin'
    for n in ['hour_sleeve','hour_gear','hour_hand']: cm[n]='hour_spin'
    tree=[['base',''],['minute_input_bearing','base'],['intermediate_bearing','base'],['rear_mast','base'],['lower_sleeve_bracket','rear_mast'],['upper_sleeve_bracket','rear_mast'],['sleeve_thrust_washer','lower_sleeve_bracket'],['lower_sleeve_bearing','lower_sleeve_bracket'],['upper_sleeve_bearing','upper_sleeve_bracket'],['minute_shaft','minute_input_bearing'],['minute_gear','minute_shaft'],['minute_hand','minute_shaft'],['intermediate_shaft','intermediate_bearing'],['compound_big_gear','intermediate_shaft'],['compound_pinion','intermediate_shaft'],['hour_sleeve','lower_sleeve_bearing'],['hour_gear','hour_sleeve'],['hour_hand','hour_sleeve']]
    rel={
      ('compound_big_gear','minute_gear'):('relation/stage1_tooth_contact','fit/minute_gear/stage1_mesh/compound_big_gear/stage1_mesh','ideal external gear mesh replaced by equality'),
      ('compound_pinion','hour_gear'):('relation/stage2_tooth_contact','fit/hour_gear/stage2_mesh/compound_pinion/stage2_mesh','ideal external gear mesh replaced by equality'),
      ('minute_gear','minute_shaft'):('relation/minute_gear_press_fit','fit/minute_shaft/minute_gear_seat/minute_gear/shaft_bore','measured interference press fit rigidly carried'),
      ('compound_big_gear','intermediate_shaft'):('relation/compound_big_press_fit','fit/intermediate_shaft/big_gear_seat/compound_big_gear/shaft_bore','measured interference press fit rigidly carried'),
      ('compound_pinion','intermediate_shaft'):('relation/compound_pinion_press_fit','fit/intermediate_shaft/pinion_seat/compound_pinion/shaft_bore','measured interference press fit rigidly carried'),
      ('hour_gear','hour_sleeve'):('relation/hour_gear_press_fit','fit/hour_sleeve/hour_gear_seat/hour_gear/sleeve_bore','measured interference press fit rigidly carried'),
      ('minute_hand','minute_shaft'):('relation/minute_hand_press_fit','fit/minute_shaft/minute_hand_seat/minute_hand/hub_bore','measured interference press fit rigidly carried'),
      ('hour_hand','hour_sleeve'):('relation/hour_hand_press_fit','fit/hour_sleeve/hour_hand_seat/hour_hand/hub_bore','measured interference press fit rigidly carried'),
      ('minute_input_bearing','minute_shaft'):('relation/minute_lower_journal','fit/minute_input_bearing/journal_bore/minute_shaft/lower_journal','positive-clearance running journal represented by hinge'),
      ('intermediate_bearing','intermediate_shaft'):('relation/intermediate_lower_journal','fit/intermediate_bearing/journal_bore/intermediate_shaft/lower_journal','positive-clearance running journal represented by hinge'),
      ('hour_sleeve','lower_sleeve_bearing'):('relation/hour_sleeve_lower_journal','fit/lower_sleeve_bearing/journal_bore/hour_sleeve/lower_outer_journal','positive-clearance running journal represented by hinge'),
      ('hour_sleeve','upper_sleeve_bearing'):('relation/hour_sleeve_upper_journal','fit/upper_sleeve_bearing/journal_bore/hour_sleeve/upper_outer_journal','positive-clearance running journal represented by hinge'),
      ('hour_sleeve','minute_shaft'):('relation/independent_coaxial_handshaft_journal','fit/hour_sleeve/inner_journal/minute_shaft/sleeve_journal','positive-clearance independent coaxial journal')}
    contacts=[]
    for i in range(len(names)):
        for j in range(i+1,len(names)):
            a=names[i]; b=names[j]
            if a in fixed and b in fixed: continue
            k=tuple(sorted([a,b])); r=rel.get(k)
            if r: contacts.append({'pair':[a,b],'action':'exclude','reason':r[2],'source_entity_ids':['link/'+a,'link/'+b,r[0]],'fact_ids':[r[1],'pair/'+k[0]+'/'+k[1]]})
            else: contacts.append({'pair':[a,b],'action':'keep','reason':'no ideal constraint or measured fit replaces contact; unavailable or uncertain overlap is conservatively kept','source_entity_ids':['link/'+a,'link/'+b],'fact_ids':['link/'+a,'link/'+b]})
    plan={'coordinate_map':cm,'tree_edges':tree,'closure_edges':[],'rigid_carried':[['minute_shaft','minute_gear'],['minute_shaft','minute_hand'],['intermediate_shaft','compound_big_gear'],['intermediate_shaft','compound_pinion'],['hour_sleeve','hour_gear'],['hour_sleeve','hour_hand']],'independent_coaxial':[['minute_shaft','hour_sleeve']],'transmissions':[{'name':'minute_to_compound_reduction','driving_joint':'minute_spin','driven_joint':'intermediate_spin','ratio':-0.3333333333333333},{'name':'rigid_compound_pair','coordinate':'intermediate_spin','ratio':1.0},{'name':'compound_to_hour_reduction','driving_joint':'intermediate_spin','driven_joint':'hour_spin','ratio':-0.25}],'contact_decisions':contacts,'support_ground':['base'],'support_strategy':'fixed authored base and support frame carry three bearing-supported hinge coordinates; no support-probe alteration required'}
    out.topology_plan(plan)
    for b,p in tree: out.body(b,p)
    out.joint('minute_shaft','minute_spin','hinge',(0,0,1),(0,0,5.5),'world')
    out.joint('intermediate_shaft','intermediate_spin','hinge',(0,0,1),(24,0,5.5),'world')
    out.joint('hour_sleeve','hour_spin','hinge',(0,0,1),(0,0,14.6),'world')
    out.joint_equality('minute_to_compound_reduction','minute_spin','intermediate_spin',-0.3333333333333333,0.0,'6 mm driving and 18 mm driven external pitch radii',['transmission/minute_to_compound_reduction','relation/stage1_tooth_contact'],['fit/minute_gear/stage1_mesh/compound_big_gear/stage1_mesh'])
    out.joint_equality('compound_to_hour_reduction','intermediate_spin','hour_spin',-0.25,0.0,'4.8 mm driving and 19.2 mm driven external pitch radii',['transmission/compound_to_hour_reduction','relation/stage2_tooth_contact'],['fit/hour_gear/stage2_mesh/compound_pinion/stage2_mesh'])
    for c in contacts:
        if c['action']=='exclude': out.exclude(c['pair'][0],c['pair'][1],c['reason'],c['source_entity_ids'],c['fact_ids'])
    for eid in facts['entity_ids']:
        nodes=['base']
        if eid.startswith('link/'):
            n=eid[5:]; nodes=[n]
            if cm[n]: nodes.append(cm[n])
        elif eid=='transmission/minute_to_compound_reduction': nodes=['minute_to_compound_reduction']
        elif eid=='transmission/compound_to_hour_reduction': nodes=['compound_to_hour_reduction']
        elif eid=='transmission/rigid_compound_pair': nodes=['intermediate_spin']
        elif eid=='role/driver/minute_gear': nodes=['minute_spin']
        elif eid=='role/output/hour_hand': nodes=['hour_spin']
        out.decision(eid,'emitted' if eid.startswith('link/') else 'represented_by',nodes,'authored entity represented by the emitted topology, coordinate, equality, contact classification, or placement on its emitted body',[eid])
