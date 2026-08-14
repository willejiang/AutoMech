def compile_mjcf(facts, out):
    coord={'base':None,'lower_bearing':None,'upper_bearing':None,'crankshaft':'input_crankshaft_hinge','crank_disk':'input_crankshaft_hinge','crank_pin':'input_crankshaft_hinge','left_guide_rail':None,'right_guide_rail':None,'slider':'horizontal_slider_joint','wrist_pin':'horizontal_slider_joint','connecting_rod':'crank_end_pin_hinge'}
    data=[
    ('base','crankshaft','proxy risk: zero axial AABB overlap',['pair/base/crankshaft','nearby/base/crankshaft'],[]),
    ('lower_bearing','crankshaft','clearance running journal represented by hinge',['fit/lower_bearing/shaft_journal/crankshaft/lower_journal','pair/crankshaft/lower_bearing','nearby/crankshaft/lower_bearing'],['relation/crankshaft_lower_revolute']),
    ('upper_bearing','crankshaft','clearance running journal represented by hinge',['fit/upper_bearing/shaft_journal/crankshaft/upper_journal','pair/crankshaft/upper_bearing','nearby/crankshaft/upper_bearing'],['relation/crankshaft_upper_journal']),
    ('crank_disk','crankshaft','interference press fit rigidly carried',['fit/crankshaft/disk_seat/crank_disk/shaft_bore','pair/crank_disk/crankshaft','nearby/crank_disk/crankshaft'],['relation/disk_to_crankshaft_press_fit']),
    ('connecting_rod','crankshaft','positive separation and disjoint AABB extent',['pair/connecting_rod/crankshaft','nearby/connecting_rod/crankshaft'],[]),
    ('crank_pin','crankshaft','positive separation and disjoint AABB extents',['pair/crank_pin/crankshaft','nearby/crank_pin/crankshaft'],[]),
    ('crank_disk','crank_pin','interference press fit rigidly carried',['fit/crank_disk/eccentric_pin_bore/crank_pin/disk_press_seat','pair/crank_disk/crank_pin','nearby/crank_disk/crank_pin'],['relation/eccentric_pin_to_disk_press_fit']),
    ('crank_disk','upper_bearing','positive separation and disjoint axial AABB extent',['pair/crank_disk/upper_bearing','nearby/crank_disk/upper_bearing'],[]),
    ('connecting_rod','crank_disk','positive separation and disjoint axial AABB extent',['pair/connecting_rod/crank_disk','nearby/connecting_rod/crank_disk'],[]),
    ('crank_disk','lower_bearing','positive separation and disjoint axial AABB extent',['pair/crank_disk/lower_bearing','nearby/crank_disk/lower_bearing'],[]),
    ('base','crank_disk','positive separation and disjoint axial AABB extent',['pair/base/crank_disk','nearby/base/crank_disk'],[]),
    ('crank_disk','left_guide_rail','positive separation and disjoint AABB extents',['pair/crank_disk/left_guide_rail','nearby/crank_disk/left_guide_rail'],[]),
    ('crank_disk','right_guide_rail','positive separation and disjoint AABB extents',['pair/crank_disk/right_guide_rail','nearby/crank_disk/right_guide_rail'],[]),
    ('connecting_rod','crank_pin','dedicated pin clearance journal represented by hinge',['fit/crank_pin/rod_journal/connecting_rod/big_end_bore','pair/connecting_rod/crank_pin','nearby/connecting_rod/crank_pin'],['relation/crank_end_pin_closure']),
    ('crank_pin','upper_bearing','positive separation and disjoint AABB extents',['pair/crank_pin/upper_bearing','nearby/crank_pin/upper_bearing'],[]),
    ('crank_pin','lower_bearing','positive separation and disjoint AABB extents',['pair/crank_pin/lower_bearing','nearby/crank_pin/lower_bearing'],[]),
    ('base','slider','zero axial AABB overlap; ideal slide supplies guidance',['pair/base/slider','nearby/base/slider'],[]),
    ('slider','wrist_pin','interference press fits rigidly carry wrist pin',['fit/slider/lower_wrist_bore/wrist_pin/lower_press_seat','fit/slider/upper_wrist_bore/wrist_pin/upper_press_seat','pair/slider/wrist_pin','nearby/slider/wrist_pin'],['relation/lower_wrist_pin_press_fit','relation/upper_wrist_pin_press_fit']),
    ('connecting_rod','slider','positive separation; wrist-center closure replaces contact',['pair/connecting_rod/slider','nearby/connecting_rod/slider'],[]),
    ('left_guide_rail','slider','positive separation and disjoint lateral AABB extent',['pair/left_guide_rail/slider','nearby/left_guide_rail/slider'],[]),
    ('right_guide_rail','slider','positive separation and disjoint lateral AABB extent',['pair/right_guide_rail/slider','nearby/right_guide_rail/slider'],[]),
    ('connecting_rod','wrist_pin','dedicated pin clearance journal represented by closure',['fit/wrist_pin/rod_journal/connecting_rod/small_end_bore','pair/connecting_rod/wrist_pin','nearby/connecting_rod/wrist_pin'],['relation/slider_end_pin_closure']),
    ('connecting_rod','upper_bearing','positive separation and disjoint axial AABB extent',['pair/connecting_rod/upper_bearing','nearby/connecting_rod/upper_bearing'],[]),
    ('connecting_rod','left_guide_rail','positive separation and disjoint axial AABB extent',['pair/connecting_rod/left_guide_rail','nearby/connecting_rod/left_guide_rail'],[]),
    ('connecting_rod','right_guide_rail','positive separation and disjoint AABB extents',['pair/connecting_rod/right_guide_rail','nearby/connecting_rod/right_guide_rail'],[]),
    ('connecting_rod','lower_bearing','positive separation and disjoint axial AABB extent',['pair/connecting_rod/lower_bearing','nearby/connecting_rod/lower_bearing'],[]),
    ('base','connecting_rod','positive separation and disjoint axial AABB extent',['pair/base/connecting_rod','nearby/base/connecting_rod'],[])]
    contacts=[]
    for a,b,r,ff,rr in data:
        src=['link/'+a,'link/'+b]+rr
        contacts.append({'pair':[a,b],'action':'exclude','reason':r,'source_entity_ids':src,'fact_ids':ff})
    out.topology_plan({'coordinate_map':coord,'tree_edges':[['base','lower_bearing'],['base','upper_bearing'],['base','left_guide_rail'],['base','right_guide_rail'],['lower_bearing','crankshaft'],['crankshaft','crank_disk'],['crank_disk','crank_pin'],['crank_pin','connecting_rod'],['base','slider'],['slider','wrist_pin']],'closure_edges':[{'name':'slider_end_pin_connect','between':['connecting_rod','wrist_pin']}],'rigid_carried':[['crankshaft','crank_disk'],['crank_disk','crank_pin'],['slider','wrist_pin']],'independent_coaxial':[['crankshaft','lower_bearing'],['crankshaft','upper_bearing'],['connecting_rod','crank_pin'],['connecting_rod','wrist_pin']],'transmissions':[{'name':'crankshaft_to_disk_rigid_drive','ratio':1.0,'representation':'rigid_carried'}],'contact_decisions':contacts,'support_ground':['base'],'support_strategy':'fixed base supports normal mechanism; no support patch'})
    out.body('base')
    for n in ['lower_bearing','upper_bearing','left_guide_rail','right_guide_rail']:
        out.body(n,'base')
    out.body('crankshaft','lower_bearing')
    out.joint('crankshaft','input_crankshaft_hinge',kind='hinge',axis=(0,0,1),pos_mm=(0,0,0),frame='world')
    out.body('crank_disk','crankshaft')
    out.body('crank_pin','crank_disk')
    out.body('connecting_rod','crank_pin')
    out.joint('connecting_rod','crank_end_pin_hinge',kind='hinge',axis=(0,0,1),pos_mm=(9.829824531467902,6.882917236212553,16),frame='world')
    out.body('slider','base')
    out.joint('slider','horizontal_slider_joint',kind='slide',axis=(1,0,0),pos_mm=(0,0,0),frame='world')
    out.body('wrist_pin','slider')
    out.connect('slider_end_pin_connect','connecting_rod','wrist_pin',(0.05935381316646628,0,0.016),'measured small-end journal closure',['relation/slider_end_pin_closure'],['fit/wrist_pin/rod_journal/connecting_rod/small_end_bore','pair/connecting_rod/wrist_pin'])
    for a,b,r,ff,rr in data:
        out.exclude(a,b,r,['link/'+a,'link/'+b]+rr,ff)
    nodes={'base':'base','lower_bearing':'lower_bearing','upper_bearing':'upper_bearing','crankshaft':'crankshaft','crank_disk':'crank_disk','crank_pin':'crank_pin','left_guide_rail':'left_guide_rail','right_guide_rail':'right_guide_rail','slider':'slider','wrist_pin':'wrist_pin','connecting_rod':'connecting_rod'}
    direct={'link/base':'base','link/lower_bearing':'lower_bearing','link/upper_bearing':'upper_bearing','link/crankshaft':'crankshaft','link/crank_disk':'crank_disk','link/crank_pin':'crank_pin','link/left_guide_rail':'left_guide_rail','link/right_guide_rail':'right_guide_rail','link/slider':'slider','link/wrist_pin':'wrist_pin','link/connecting_rod':'connecting_rod','motion_joint/input_crankshaft_hinge':'input_crankshaft_hinge','motion_joint/horizontal_slider_joint':'horizontal_slider_joint','relation/crankshaft_lower_revolute':'input_crankshaft_hinge','relation/crankshaft_upper_journal':'input_crankshaft_hinge','relation/disk_to_crankshaft_press_fit':'crank_disk','relation/eccentric_pin_to_disk_press_fit':'crank_pin','relation/crank_end_pin_closure':'crank_end_pin_hinge','relation/lower_wrist_pin_press_fit':'wrist_pin','relation/upper_wrist_pin_press_fit':'wrist_pin','relation/slider_end_pin_closure':'slider_end_pin_connect','transmission/crankshaft_to_disk_rigid_drive':'crank_disk','role/driver/crankshaft':'input_crankshaft_hinge','role/output/slider':'horizontal_slider_joint','role/watch/crankshaft':'input_crankshaft_hinge','role/watch/crank_disk':'crank_disk','role/watch/connecting_rod':'crank_end_pin_hinge','role/watch/slider':'horizontal_slider_joint'}
    prefixes=[('pose/place_base','base'),('pose/place_lower_bearing','lower_bearing'),('pose/place_upper_bearing','upper_bearing'),('pose/place_crankshaft','crankshaft'),('pose/place_crank_disk','crank_disk'),('pose/place_crank_pin','crank_pin'),('pose/place_left_guide_rail','left_guide_rail'),('pose/place_right_guide_rail','right_guide_rail'),('pose/place_slider','slider'),('pose/place_wrist_pin','wrist_pin'),('pose/place_connecting_rod','connecting_rod'),('port/base/','base'),('port/lower_bearing/','lower_bearing'),('port/upper_bearing/','upper_bearing'),('port/crankshaft/','crankshaft'),('port/crank_disk/','crank_disk'),('port/crank_pin/','crank_pin'),('port/connecting_rod/','connecting_rod'),('port/slider/','slider'),('port/wrist_pin/','wrist_pin')]
    for eid in facts['entity_ids']:
        node=direct[eid] if eid in direct else None
        if node is None:
            for pre,n in prefixes:
                if eid.startswith(pre): node=n
        out.decision(eid,action='emitted' if eid in nodes else 'represented_by',generated_nodes=[node],reason='authored entity represented by emitted topology node',fact_ids=[eid])
