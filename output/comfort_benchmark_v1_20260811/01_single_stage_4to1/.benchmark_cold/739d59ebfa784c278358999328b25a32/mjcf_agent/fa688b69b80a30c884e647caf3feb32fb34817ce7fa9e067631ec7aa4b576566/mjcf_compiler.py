def compile_mjcf(facts, out):
    coord={'base':None,'bench_bolt_1':None,'bench_bolt_2':None,'bench_bolt_3':None,'bench_bolt_4':None,'input_thrust_washer':None,'output_thrust_washer':None,'input_bearing_pedestal':None,'output_bearing_pedestal':None,'input_lower_bearing':None,'input_upper_bearing':None,'output_lower_bearing':None,'output_upper_bearing':None,'input_shaft':'input_shaft_hinge','input_pinion':'input_shaft_hinge','hand_crank':'input_shaft_hinge','output_shaft':'output_shaft_hinge','output_gear':'output_shaft_hinge','crank_grip':'crank_grip_hinge'}
    rr=[
      ('input_lower_bearing','input_shaft','input_lower_running_fit','input_lower_bearing/journal/input_shaft/lower_journal','running journal fit is represented by the input shaft hinge'),
      ('input_upper_bearing','input_shaft','input_upper_running_fit','input_upper_bearing/journal/input_shaft/upper_journal','running journal fit is represented by the input shaft hinge'),
      ('output_lower_bearing','output_shaft','output_lower_running_fit','output_lower_bearing/journal/output_shaft/lower_journal','running journal fit is represented by the output shaft hinge'),
      ('output_upper_bearing','output_shaft','output_upper_running_fit','output_upper_bearing/journal/output_shaft/upper_journal','running journal fit is represented by the output shaft hinge'),
      ('input_shaft','input_pinion','pinion_press_fit','input_shaft/pinion_seat/input_pinion/shaft_bore','measured interference press fit is rigidly carried'),
      ('output_shaft','output_gear','output_gear_press_fit','output_shaft/gear_seat/output_gear/shaft_bore','measured interference press fit is rigidly carried'),
      ('input_shaft','hand_crank','crank_press_fit','input_shaft/crank_seat/hand_crank/shaft_bore','measured interference press fit is rigidly carried'),
      ('hand_crank','crank_grip','crank_grip_running_fit','hand_crank/grip_pin/crank_grip/pin_bore','running grip journal is represented by its hinge'),
      ('input_pinion','output_gear','stage1_external_spur_mesh','input_pinion/mesh/output_gear/mesh','ideal joint equality replaces the external spur tooth mesh')]
    near=[
      ('input_shaft','input_thrust_washer','keep'),('input_shaft','input_bearing_pedestal','exclude'),('input_shaft','output_gear','exclude'),('input_shaft','base','exclude'),('output_shaft','output_thrust_washer','keep'),('output_shaft','output_bearing_pedestal','exclude'),('output_shaft','base','exclude'),
      ('input_pinion','input_upper_bearing','exclude'),('input_pinion','hand_crank','exclude'),('input_pinion','input_lower_bearing','exclude'),('input_pinion','input_bearing_pedestal','exclude'),('input_pinion','input_thrust_washer','exclude'),('input_pinion','base','exclude'),('input_pinion','crank_grip','exclude'),
      ('output_gear','input_upper_bearing','exclude'),('output_gear','output_upper_bearing','exclude'),('output_gear','hand_crank','exclude'),('output_gear','input_lower_bearing','exclude'),('output_gear','output_lower_bearing','exclude'),('output_gear','crank_grip','exclude'),('output_gear','bench_bolt_3','exclude'),('output_gear','bench_bolt_4','exclude'),('output_gear','input_bearing_pedestal','exclude'),('output_gear','output_bearing_pedestal','exclude'),('output_gear','input_thrust_washer','exclude'),('output_gear','output_thrust_washer','exclude'),('output_gear','base','exclude'),('output_gear','bench_bolt_1','exclude'),('output_gear','bench_bolt_2','exclude'),
      ('hand_crank','input_upper_bearing','exclude'),('hand_crank','input_lower_bearing','exclude'),('hand_crank','input_bearing_pedestal','exclude'),('hand_crank','input_thrust_washer','exclude'),('hand_crank','base','exclude')]
    contacts=[]
    for a,b,r,f,why in rr:
        lo=a if a<b else b; hi=b if a<b else a
        contacts.append({'pair':[a,b],'action':'exclude','reason':why,'source_entity_ids':['relation/'+r,'link/'+a,'link/'+b],'fact_ids':['fit/'+f,'pair/'+lo+'/'+hi,'nearby/'+lo+'/'+hi]})
    for a,b,act in near:
        lo=a if a<b else b; hi=b if a<b else a
        why='real thrust-face contact is mechanically required and geometry overlap is uncertain' if act=='keep' else 'positive surface separation or disjoint AABB identifies collision-proxy false-contact risk'
        contacts.append({'pair':[a,b],'action':act,'reason':why,'source_entity_ids':['link/'+a,'link/'+b],'fact_ids':['pair/'+lo+'/'+hi,'nearby/'+lo+'/'+hi]})
    edges=[['base',x] for x in ['bench_bolt_1','bench_bolt_2','bench_bolt_3','bench_bolt_4','input_thrust_washer','output_thrust_washer','input_bearing_pedestal','output_bearing_pedestal','input_shaft','output_shaft']]
    edges += [['input_bearing_pedestal','input_lower_bearing'],['input_lower_bearing','input_upper_bearing'],['output_bearing_pedestal','output_lower_bearing'],['output_lower_bearing','output_upper_bearing'],['input_shaft','input_pinion'],['input_shaft','hand_crank'],['output_shaft','output_gear'],['hand_crank','crank_grip']]
    out.topology_plan({'coordinate_map':coord,'tree_edges':edges,'closure_edges':[['input_pinion','output_gear','spur_reduction_eq']],'rigid_carried':[['input_shaft','input_pinion'],['input_shaft','hand_crank'],['output_shaft','output_gear']],'independent_coaxial':[['input_shaft','input_lower_bearing'],['input_shaft','input_upper_bearing'],['output_shaft','output_lower_bearing'],['output_shaft','output_upper_bearing'],['hand_crank','crank_grip']],'transmissions':[{'name':'spur_reduction_stage','driving_joint':'input_shaft_hinge','driven_joint':'output_shaft_hinge','ratio':-0.25}],'contact_decisions':contacts,'support_ground':['base'],'support_strategy':'fixed base supports gravity and remains ground-collidable; no patch required'})
    out.body('base')
    for x in ['bench_bolt_1','bench_bolt_2','bench_bolt_3','bench_bolt_4','input_thrust_washer','output_thrust_washer','input_bearing_pedestal','output_bearing_pedestal']: out.body(x,'base')
    out.body('input_lower_bearing','input_bearing_pedestal'); out.body('input_upper_bearing','input_lower_bearing'); out.body('output_lower_bearing','output_bearing_pedestal'); out.body('output_upper_bearing','output_lower_bearing')
    out.body('input_shaft','base'); out.joint('input_shaft','input_shaft_hinge','hinge',(0,0,1),(0,0,0),'world'); out.body('input_pinion','input_shaft'); out.body('hand_crank','input_shaft'); out.body('crank_grip','hand_crank'); out.joint('crank_grip','crank_grip_hinge','hinge',(0,0,1),(0,0,0),'world')
    out.body('output_shaft','base'); out.joint('output_shaft','output_shaft_hinge','hinge',(0,0,1),(0,0,0),'world'); out.body('output_gear','output_shaft')
    out.joint_equality('spur_reduction_eq','input_shaft_hinge','output_shaft_hinge',-0.25,0.0,'external 9 mm to 36 mm pitch-radius spur reduction',['relation/stage1_external_spur_mesh','transmission/spur_reduction_stage'],['fit/input_pinion/mesh/output_gear/mesh','pair/input_pinion/output_gear'])
    for c in contacts:
        if c['action']=='exclude': out.exclude(c['pair'][0],c['pair'][1],c['reason'],c['source_entity_ids'],c['fact_ids'])
    for e in facts['entity_ids']:
        n=['base']; act='represented_by'
        if e[:5]=='link/': n=[e[5:]]; act='emitted'
        if e[:13]=='motion_joint/': n=[e[13:]]; act='emitted'
        if e=='transmission/spur_reduction_stage' or e=='relation/stage1_external_spur_mesh': n=['spur_reduction_eq']
        out.decision(e,act,n,'accounted for by an emitted body, joint, fit exclusion, rigid carrying, role mapping, or transmission equality',[e])
