from afem.exchange import ImportVSP
from afem.graphics import Viewer

# v = Viewer()
fn = r'..\models\777-200LR.stp'
# fn = r'..\models\TBW_SUGAR.stp'
# fn = r'..\models\F-16.stp'
# fn = r'..\models\HWB.stp'
# fn = r'..\models\N2A_nosplit.stp'
# fn = r'..\models\manta2.stp'
# fn = r'..\models\transport_vsp313.stp'
# fn = r'..\models\777-200LR_vsp311.stp'
# fn = r'..\models\777-200LR_vsp313.stp'

ImportVSP.step_file(fn)

bodies = ImportVSP.get_bodies()
shapes = []
for name in bodies:
    b = bodies[name]
    shapes.append(b)

v = Viewer()
v.add(*shapes)
v.start()
