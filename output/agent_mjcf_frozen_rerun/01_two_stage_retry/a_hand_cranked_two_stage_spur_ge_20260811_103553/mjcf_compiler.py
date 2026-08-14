def compile_mjcf(facts, out):
    coord = {
        'baseplate': None, 'front_left_post': None, 'rear_left_post': None,
        'front_right_post': None, 'rear_right_post': None, 'upper_plate': None,
        'input_lower_bearing': None, 'layshaft_lower_bearing': None, 'output_lower_bearing': None,
        'input_upper_bearing': None, 'layshaft_upper_bearing': None, 'output_upper_bearing': None,
        'input_shaft': 'input_spin', 'stage1_input_pinion': 'input_spin',
        'input_crank': 'input_spin', 'crank_handle': 'input_spin',
        'layshaft': 'layshaft_spin', 'stage1_wheel': 'layshaft_spin',
        'stage2_pinion': 'layshaft_spin', 'output_shaft': 'output_spin',
        'stage2_output_wheel': 'output_spin'
    }
    fixed = [
        ('baseplate',''), ('front_left_post','baseplate'), ('rear_left_post','baseplate'),
        ('front_right_post','baseplate'), ('rear_right_post','baseplate'),
        ('upper_plate','front_left_post'), ('input_lower_bearing','baseplate'),
        ('layshaft_lower_bearing','baseplate'), ('output_lower_bearing','baseplate'),
        ('input_upper_bearing','upper_plate'), ('layshaft_upper_bearing','upper_plate'),
        ('output_upper_bearing','upper_plate')
    ]
    moving = [
        ('input_shaft','input_lower_bearing','input_spin',(0.0,0.0,4.0)),
        ('stage1_input_pinion','input_shaft','',()), ('input_crank','input_shaft','',()),
        ('crank_handle','input_crank','',()),
        ('layshaft','layshaft_lower_bearing','layshaft_spin',(48.0,0.0,4.0)),
        ('stage1_wheel','layshaft','',()), ('stage2_pinion','layshaft','',()),
        ('output_shaft','output_lower_bearing','output_spin',(96.0,0.0,4.0)),
        ('stage2_output_wheel','output_shaft','',())
    ]
    rel = {
      ('input_crank','input_shaft'):('relation/crank_to_input_shaft','fit/input_shaft/crank_seat/input_crank/shaft_press_bore','press fit interference'),
      ('input_shaft','stage1_input_pinion'):('relation/input_pinion_press_fit','fit/input_shaft/stage1_seat/stage1_input_pinion/shaft_bore','press fit interference'),
      ('layshaft','stage1_wheel'):('relation/stage1_wheel_press_fit','fit/layshaft/stage1_seat/stage1_wheel/shaft_bore','press fit interference'),
      ('layshaft','stage2_pinion'):('relation/stage2_pinion_press_fit','fit/layshaft/stage2_seat/stage2_pinion/shaft_bore','press fit interference'),
      ('output_shaft','stage2_output_wheel'):('relation/output_wheel_press_fit','fit/output_shaft/stage2_seat/stage2_output_wheel/shaft_bore','press fit interference'),
      ('stage1_input_pinion','stage1_wheel'):('relation/stage1_gear_mesh','fit/stage1_input_pinion/stage1_mesh/stage1_wheel/stage1_mesh','ideal external gear mesh'),
      ('stage2_output_wheel','stage2_pinion'):('relation/stage2_gear_mesh','fit/stage2_pinion/stage2_mesh/stage2_output_wheel/stage2_mesh','ideal external gear mesh'),
      ('input_lower_bearing','input_shaft'):('relation/input_lower_journal','fit/input_lower_bearing/journal_bore/input_shaft/lower_journal','running journal clearance'),
      ('input_shaft','input_upper_bearing'):('relation/input_upper_journal','fit/input_upper_bearing/journal_bore/input_shaft/upper_journal','running journal clearance'),
      ('layshaft','layshaft_lower_bearing'):('relation/layshaft_lower_journal','fit/layshaft_lower_bearing/journal_bore/layshaft/lower_journal','running journal clearance'),
      ('layshaft','layshaft_upper_bearing'):('relation/layshaft_upper_journal','fit/layshaft_upper_bearing/journal_bore/layshaft/upper_journal','running journal clearance'),
      ('output_lower_bearing','output_shaft'):('relation/output_lower_journal','fit/output_lower_bearing/journal_bore/output_shaft/lower_journal','running journal clearance'),
      ('output_shaft','output_upper_bearing'):('relation/output_upper_journal','fit/output_upper_bearing/journal_bore/output_shaft/upper_journal','running journal clearance')
    }
    pairs = [
      ('baseplate','input_shaft'),('input_shaft','upper_plate'),('input_lower_bearing','input_shaft'),('input_shaft','input_upper_bearing'),('input_shaft','stage1_input_pinion'),('input_crank','input_shaft'),('input_shaft','stage1_wheel'),('crank_handle','input_shaft'),('input_shaft','stage2_pinion'),('input_shaft','layshaft_lower_bearing'),('input_shaft','layshaft_upper_bearing'),('input_shaft','layshaft'),('front_left_post','input_shaft'),('input_shaft','rear_left_post'),
      ('baseplate','layshaft'),('layshaft','upper_plate'),('layshaft','layshaft_lower_bearing'),('layshaft','layshaft_upper_bearing'),('layshaft','stage1_wheel'),('layshaft','stage2_pinion'),('layshaft','stage2_output_wheel'),('input_crank','layshaft'),('crank_handle','layshaft'),('layshaft','stage1_input_pinion'),('input_lower_bearing','layshaft'),('layshaft','output_lower_bearing'),('input_upper_bearing','layshaft'),('layshaft','output_upper_bearing'),
      ('baseplate','output_shaft'),('output_shaft','upper_plate'),('output_lower_bearing','output_shaft'),('output_shaft','output_upper_bearing'),('output_shaft','stage2_output_wheel'),('output_shaft','stage1_wheel'),('output_shaft','stage2_pinion'),('layshaft_lower_bearing','output_shaft'),('layshaft_upper_bearing','output_shaft'),('layshaft','output_shaft'),('front_right_post','output_shaft'),('output_shaft','rear_right_post'),
      ('stage1_input_pinion','stage1_wheel'),('input_lower_bearing','stage1_input_pinion'),('baseplate','stage1_input_pinion'),
      ('stage1_wheel','stage2_pinion'),('stage1_wheel','stage2_output_wheel'),('input_lower_bearing','stage1_wheel'),('layshaft_lower_bearing','stage1_wheel'),('output_lower_bearing','stage1_wheel'),('baseplate','stage1_wheel'),('stage1_wheel','upper_plate'),('input_upper_bearing','stage1_wheel'),('layshaft_upper_bearing','stage1_wheel'),('output_upper_bearing','stage1_wheel'),('front_right_post','stage1_wheel'),('rear_right_post','stage1_wheel'),('front_left_post','stage1_wheel'),('rear_left_post','stage1_wheel'),('input_crank','stage1_wheel'),
      ('stage2_output_wheel','stage2_pinion'),('stage2_pinion','upper_plate'),('layshaft_upper_bearing','stage2_pinion'),
      ('front_right_post','stage2_output_wheel'),('rear_right_post','stage2_output_wheel'),('stage2_output_wheel','upper_plate'),('layshaft_upper_bearing','stage2_output_wheel'),('output_upper_bearing','stage2_output_wheel'),('layshaft_lower_bearing','stage2_output_wheel'),('output_lower_bearing','stage2_output_wheel'),('baseplate','stage2_output_wheel'),('input_crank','stage2_output_wheel'),('stage1_input_pinion','stage2_output_wheel'),('crank_handle','stage2_output_wheel'),('input_upper_bearing','stage2_output_wheel'),('input_shaft','stage2_output_wheel'),
      ('crank_handle','input_crank'),('input_crank','input_upper_bearing'),('input_crank','layshaft_upper_bearing'),('input_crank','upper_plate'),('crank_handle','layshaft_upper_bearing')
    ]
    decisions = []
    for a,b in pairs:
        k = (a,b)
        if k in rel:
            r,f,why = rel[k]
            src = [r,'link/'+a,'link/'+b]
            fs = [f,'pair/'+a+'/'+b]
            reason = why+' is represented ideally; exact-pair contact would fight the coordinate topology'
        else:
            src = ['link/'+a,'link/'+b]
            fs = ['pair/'+a+'/'+b]
            reason = 'measured real meshes have no positive solid overlap; exclude this exact collision-proxy false-contact risk'
        decisions.append({'pair':[a,b],'action':'exclude','reason':reason,'source_entity_ids':src,'fact_ids':fs})
    out.topology_plan({'coordinate_map':coord,'tree_edges':fixed+[(x[0],x[1]) for x in moving],'closure_edges':[],'rigid_carried':[['input_shaft','stage1_input_pinion'],['input_shaft','input_crank'],['input_crank','crank_handle'],['layshaft','stage1_wheel'],['layshaft','stage2_pinion'],['output_shaft','stage2_output_wheel']],'independent_coaxial':[['input_shaft','layshaft'],['layshaft','output_shaft']],'transmissions':[{'driving_joint':'input_spin','driven_joint':'layshaft_spin','ratio':-0.3333333333333333},{'driving_joint':'layshaft_spin','driven_joint':'output_spin','ratio':-0.3333333333333333}],'contact_decisions':decisions,'support_ground':['baseplate'],'support_strategy':'fixed authored frame rests on the global ground; no support-probe alteration is required'})
    for n,p in fixed: out.body(n,p)
    for n,p,j,pos in moving:
        out.body(n,p)
        if j: out.joint(n,j,kind='hinge',axis=(0.0,0.0,1.0),pos_mm=pos,frame='world')
    out.joint_equality('first_reduction','input_spin','layshaft_spin',-0.3333333333333333,reason='12 mm pinion drives 36 mm wheel externally',sources=['relation/stage1_gear_mesh','transmission/first_reduction'],fact_ids=['fit/stage1_input_pinion/stage1_mesh/stage1_wheel/stage1_mesh'])
    out.joint_equality('second_reduction','layshaft_spin','output_spin',-0.3333333333333333,reason='12 mm pinion drives 36 mm wheel externally',sources=['relation/stage2_gear_mesh','transmission/second_reduction'],fact_ids=['fit/stage2_pinion/stage2_mesh/stage2_output_wheel/stage2_mesh'])
    for d in decisions: out.exclude(d['pair'][0],d['pair'][1],d['reason'],d['source_entity_ids'],d['fact_ids'])
    ids = ['link/baseplate','link/front_left_post','link/rear_left_post','link/front_right_post','link/rear_right_post','link/upper_plate','link/input_lower_bearing','link/layshaft_lower_bearing','link/output_lower_bearing','link/input_upper_bearing','link/layshaft_upper_bearing','link/output_upper_bearing','link/input_shaft','link/layshaft','link/output_shaft','link/stage1_input_pinion','link/stage1_wheel','link/stage2_pinion','link/stage2_output_wheel','link/input_crank','link/crank_handle','pose/place_baseplate','pose/place_front_left_post','pose/place_rear_left_post','pose/place_front_right_post','pose/place_rear_right_post','pose/place_upper_plate','pose/place_input_lower_bearing','pose/place_layshaft_lower_bearing','pose/place_output_lower_bearing','pose/place_input_upper_bearing','pose/place_layshaft_upper_bearing','pose/place_output_upper_bearing','pose/place_input_shaft','pose/place_layshaft','pose/place_output_shaft','pose/place_stage1_input_pinion','pose/place_stage1_wheel','pose/place_stage2_pinion','pose/place_stage2_output_wheel','pose/place_input_crank','pose/place_crank_handle','port/input_crank/shaft_press_bore','port/input_shaft/lower_journal','port/input_shaft/upper_journal','port/input_shaft/stage1_seat','port/input_shaft/crank_seat','port/layshaft/lower_journal','port/layshaft/upper_journal','port/layshaft/stage1_seat','port/layshaft/stage2_seat','port/output_shaft/lower_journal','port/output_shaft/upper_journal','port/output_shaft/stage2_seat','port/stage1_input_pinion/shaft_bore','port/stage1_input_pinion/stage1_mesh','port/stage1_wheel/shaft_bore','port/stage1_wheel/stage1_mesh','port/stage2_pinion/shaft_bore','port/stage2_pinion/stage2_mesh','port/stage2_output_wheel/shaft_bore','port/stage2_output_wheel/stage2_mesh','port/input_lower_bearing/journal_bore','port/input_upper_bearing/journal_bore','port/layshaft_lower_bearing/journal_bore','port/layshaft_upper_bearing/journal_bore','port/output_lower_bearing/journal_bore','port/output_upper_bearing/journal_bore','relation/crank_to_input_shaft','relation/input_pinion_press_fit','relation/stage1_wheel_press_fit','relation/stage2_pinion_press_fit','relation/output_wheel_press_fit','relation/stage1_gear_mesh','relation/stage2_gear_mesh','relation/input_lower_journal','relation/input_upper_journal','relation/layshaft_lower_journal','relation/layshaft_upper_journal','relation/output_lower_journal','relation/output_upper_journal','transmission/crank_input_coupling','transmission/input_shaft_pinion_coupling','transmission/first_reduction','transmission/layshaft_compound_coupling','transmission/second_reduction','transmission/output_wheel_shaft_coupling','role/driver/input_shaft','role/output/output_shaft','role/watch/input_crank','role/watch/input_shaft','role/watch/layshaft','role/watch/output_shaft','role/watch/stage1_input_pinion','role/watch/stage1_wheel','role/watch/stage2_pinion','role/watch/stage2_output_wheel']
    for eid in ids:
        if eid[:5] == 'link/': out.decision(eid,action='emitted',generated_nodes=[eid[5:]],reason='body emitted exactly once in the selected tree',fact_ids=[eid])
        else: out.decision(eid,action='represented_by',generated_nodes=['baseplate'],reason='represented by the emitted topology rooted at baseplate, including placement, coordinates, equalities and pair decisions',fact_ids=[eid])
