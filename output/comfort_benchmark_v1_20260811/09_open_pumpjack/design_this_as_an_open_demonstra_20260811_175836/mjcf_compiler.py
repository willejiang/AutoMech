def compile_mjcf(facts, out):
    order=['base','negative_shaft_pedestal','negative_shaft_bearing','positive_shaft_pedestal','positive_shaft_bearing','crankshaft','crank_disk','crank_pin','pitman','negative_beam_support','negative_pivot_bearing','positive_beam_support','positive_pivot_bearing','beam_pivot_pin','walking_beam','pitman_beam_pin','beam_output_pin','polished_output_rod','output_guide_post','lower_output_guide','upper_output_guide','hand_crank_arm','handle_spindle','hand_grip']
    tree=[['base','negative_shaft_pedestal'],['negative_shaft_pedestal','negative_shaft_bearing'],['base','positive_shaft_pedestal'],['positive_shaft_pedestal','positive_shaft_bearing'],['base','crankshaft'],['base','crank_disk'],['crank_disk','crank_pin'],['base','negative_beam_support'],['negative_beam_support','negative_pivot_bearing'],['base','positive_beam_support'],['positive_beam_support','positive_pivot_bearing'],['negative_pivot_bearing','beam_pivot_pin'],['base','walking_beam'],['walking_beam','pitman_beam_pin'],['walking_beam','beam_output_pin'],['base','polished_output_rod'],['base','output_guide_post'],['output_guide_post','lower_output_guide'],['output_guide_post','upper_output_guide'],['crankshaft','hand_crank_arm'],['hand_crank_arm','handle_spindle'],['handle_spindle','hand_grip']]
    coord={x:None for x in order}
    coord.update({'crankshaft':'driven_crankshaft_hinge','crank_disk':'crank_disk_hinge','crank_pin':'crank_disk_hinge','walking_beam':'walking_beam_hinge','pitman_beam_pin':'walking_beam_hinge','beam_output_pin':'walking_beam_hinge','polished_output_rod':'vertical_guided_output','hand_crank_arm':'driven_crankshaft_hinge','handle_spindle':'driven_crankshaft_hinge','hand_grip':'driven_crankshaft_hinge'})
    raw=[]
    special=[
      ('crankshaft','negative_shaft_bearing','journal running fit represented by shaft hinge',['relation/shaft_in_negative_bearing'],['pair/crankshaft/negative_shaft_bearing','fit/negative_shaft_bearing/bearing_bore/crankshaft/rotation_axis']),
      ('crankshaft','positive_shaft_bearing','journal running fit represented by shaft hinge',['relation/shaft_in_positive_bearing'],['pair/crankshaft/positive_shaft_bearing','fit/positive_shaft_bearing/bearing_bore/crankshaft/rotation_axis']),
      ('crankshaft','crank_disk','press fit represented by 1:1 equality',['relation/disk_pressed_on_shaft','transmission/shaft_to_crank_disk_press_drive'],['pair/crank_disk/crankshaft','fit/crankshaft/disk_seat/crank_disk/shaft_bore']),
      ('crank_disk','crank_pin','interference press fit rigidly carries pin',['relation/crank_pin_pressed_in_disk'],['pair/crank_disk/crank_pin','fit/crank_disk/crank_pin_seat/crank_pin/pin_shaft']),
      ('crank_pin','pitman','dedicated clearance pin bearing represented by closure',['relation/crank_pin_to_pitman_closure'],['pair/crank_pin/pitman','fit/crank_pin/pin_shaft/pitman/crank_end_bore']),
      ('pitman','pitman_beam_pin','dedicated clearance pin bearing represented by closure',['relation/pitman_to_walking_beam_closure'],['pair/pitman/pitman_beam_pin','fit/pitman_beam_pin/pin_shaft/pitman/beam_end_bore']),
      ('walking_beam','beam_pivot_pin','clearance pivot represented by beam hinge',['relation/walking_beam_fixed_pivot'],['pair/beam_pivot_pin/walking_beam','fit/beam_pivot_pin/pivot_shaft/walking_beam/pivot_bore']),
      ('walking_beam','pitman_beam_pin','interference fit rigidly carries dedicated pin',['link/walking_beam','link/pitman_beam_pin'],['pair/pitman_beam_pin/walking_beam','fit/pitman_beam_pin/pin_shaft/walking_beam/pitman_pin_bore']),
      ('walking_beam','beam_output_pin','interference fit rigidly carries dedicated pin',['link/walking_beam','link/beam_output_pin'],['pair/beam_output_pin/walking_beam','fit/beam_output_pin/pin_shaft/walking_beam/output_pin_bore']),
      ('polished_output_rod','beam_output_pin','dedicated clearance pin bearing represented by closure',['relation/beam_to_output_rod_closure'],['pair/beam_output_pin/polished_output_rod','fit/beam_output_pin/pin_shaft/polished_output_rod/beam_pin_bore']),
      ('polished_output_rod','lower_output_guide','running guide represented by vertical slide',['link/polished_output_rod','link/lower_output_guide'],['pair/lower_output_guide/polished_output_rod']),
      ('polished_output_rod','upper_output_guide','running guide represented by vertical slide',['link/polished_output_rod','link/upper_output_guide'],['pair/polished_output_rod/upper_output_guide'])]
    for a,b,r,s,f in special: raw.append((a,b,'exclude',r,s,f))
    groups={
      'crankshaft':['pitman','negative_shaft_pedestal','positive_shaft_pedestal','crank_pin','hand_grip','handle_spindle','walking_beam','base','pitman_beam_pin'],
      'crank_disk':['walking_beam','pitman','pitman_beam_pin','base','negative_shaft_pedestal','positive_shaft_pedestal','negative_shaft_bearing','positive_shaft_bearing','hand_crank_arm','handle_spindle','hand_grip','positive_beam_support','positive_pivot_bearing','negative_beam_support','negative_pivot_bearing'],
      'crank_pin':['negative_shaft_bearing'],
      'pitman':['walking_beam','negative_shaft_bearing','negative_shaft_pedestal'],
      'walking_beam':['positive_beam_support','positive_pivot_bearing','negative_beam_support','negative_pivot_bearing','output_guide_post','crank_pin','upper_output_guide','hand_crank_arm','handle_spindle','hand_grip','lower_output_guide'],
      'polished_output_rod':['base','output_guide_post','negative_pivot_bearing','positive_pivot_bearing','negative_beam_support','positive_beam_support','beam_pivot_pin']}
    for a in groups:
        for b in groups[a]: raw.append((a,b,'exclude','positive real-mesh separation with disjoint AABB extent is a collision-proxy false-contact risk',['link/'+a,'link/'+b],['pair/'+a+'/'+b,'nearby/'+a+'/'+b]))
    raw.append(('crankshaft','hand_crank_arm','keep','uncertain measured overlap is retained',['link/crankshaft','link/hand_crank_arm'],['pair/crankshaft/hand_crank_arm']))
    raw.append(('walking_beam','polished_output_rod','keep','output solids meet and uncertain overlap is retained',['link/walking_beam','link/polished_output_rod'],['pair/polished_output_rod/walking_beam']))
    contacts=[{'pair':[a,b],'action':x,'reason':r,'source_entity_ids':s,'fact_ids':f} for a,b,x,r,s,f in raw]
    out.topology_plan({'coordinate_map':coord,'tree_edges':tree,'closure_edges':[['crank_pin','pitman'],['pitman_beam_pin','pitman'],['beam_output_pin','polished_output_rod']],'rigid_carried':[['crank_disk','crank_pin'],['walking_beam','pitman_beam_pin'],['walking_beam','beam_output_pin'],['crankshaft','hand_crank_arm'],['hand_crank_arm','handle_spindle'],['handle_spindle','hand_grip']],'independent_coaxial':[],'transmissions':[{'name':'shaft_to_crank_disk_press_drive','driving_joint':'driven_crankshaft_hinge','driven_joint':'crank_disk_hinge','ratio':1.0}],'contact_decisions':contacts,'support_ground':['base'],'support_strategy':'fixed base and supports remain on global ground; no support patch required'})
    parents={x[1]:x[0] for x in tree}
    for b in order: out.body(b,parents.get(b,''))
    out.joint('crankshaft','driven_crankshaft_hinge','hinge',(0,1,0),(0,0,0),'world')
    out.joint('crank_disk','crank_disk_hinge','hinge',(0,1,0),(0,0,0),'world')
    out.freejoint('pitman','pitman_free')
    out.joint('walking_beam','walking_beam_hinge','hinge',(0,1,0),(0,0,0),'world')
    out.joint('polished_output_rod','vertical_guided_output','slide',(0,0,1),(0,0,0),'world')
    out.joint_equality('shaft_to_crank_disk_press_drive','driven_crankshaft_hinge','crank_disk_hinge',1,0,'press-fit compound transmits angle 1:1',['relation/disk_pressed_on_shaft','transmission/shaft_to_crank_disk_press_drive'],['fit/crankshaft/disk_seat/crank_disk/shaft_bore','pair/crank_disk/crankshaft'])
    out.connect('crank_pin_to_pitman_closure','crank_pin','pitman',(-.0302552632,-.01,.0553243759),'clearance crank-pin closure',['relation/crank_pin_to_pitman_closure'],['fit/crank_pin/pin_shaft/pitman/crank_end_bore','pair/crank_pin/pitman'])
    out.connect('pitman_to_walking_beam_closure','pitman_beam_pin','pitman',(-.0245620631,-.01,.0837372105),'clearance beam-end closure',['relation/pitman_to_walking_beam_closure'],['fit/pitman_beam_pin/pin_shaft/pitman/beam_end_bore','pair/pitman/pitman_beam_pin'])
    out.connect('beam_to_output_rod_closure','beam_output_pin','polished_output_rod',(.0744647438,-.01,.0976545206),'clearance output closure',['relation/beam_to_output_rod_closure'],['fit/beam_output_pin/pin_shaft/polished_output_rod/beam_pin_bore','pair/beam_output_pin/polished_output_rod'])
    for a,b,x,r,s,f in raw:
        if x=='exclude': out.exclude(a,b,r,s,f)
    ids=['link/'+x for x in order]+['pose/place_'+x for x in order]
    ports=['crankshaft/rotation_axis','crankshaft/disk_seat','crank_disk/shaft_bore','crank_disk/crank_pin_seat','crank_pin/pin_shaft','pitman/crank_end_bore','pitman/beam_end_bore','walking_beam/pitman_pin_bore','walking_beam/pivot_bore','walking_beam/output_pin_bore','pitman_beam_pin/pin_shaft','beam_pivot_pin/pivot_shaft','beam_output_pin/pin_shaft','polished_output_rod/beam_pin_bore','polished_output_rod/guided_axis']
    ids += ['port/'+x for x in ports]
    rels=['shaft_in_negative_bearing','shaft_in_positive_bearing','disk_pressed_on_shaft','crank_pin_pressed_in_disk','crank_pin_to_pitman_closure','pitman_to_walking_beam_closure','walking_beam_fixed_pivot','beam_to_output_rod_closure']
    ids += ['relation/'+x for x in rels]+['motion_joint/'+x for x in ['driven_crankshaft_hinge','crank_disk_hinge','walking_beam_hinge','vertical_guided_output']]
    ids += ['transmission/shaft_to_crank_disk_press_drive','role/driver/crankshaft','role/output/polished_output_rod']+['role/watch/'+x for x in ['crankshaft','crank_disk','crank_pin','pitman','walking_beam','polished_output_rod']]
    for e in ids:
        nodes=['base']; action='represented_by'
        if e.startswith('link/'): nodes=[e[5:]]; action='emitted'
        elif e.startswith('pose/place_'): nodes=[e[11:]]
        elif e.startswith('motion_joint/'): nodes=[e[13:]]; action='emitted'
        elif e=='relation/crank_pin_to_pitman_closure': nodes=['crank_pin_to_pitman_closure']
        elif e=='relation/pitman_to_walking_beam_closure': nodes=['pitman_to_walking_beam_closure']
        elif e=='relation/beam_to_output_rod_closure': nodes=['beam_to_output_rod_closure']
        elif e in ['relation/disk_pressed_on_shaft','transmission/shaft_to_crank_disk_press_drive']: nodes=['shaft_to_crank_disk_press_drive']
        out.decision(e,action,nodes,'represented by emitted topology, coordinate, constraint, contact decision, or role map',[e])
