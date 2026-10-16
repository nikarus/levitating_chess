import json
from pathlib import Path
from time import perf_counter
import numpy as np
from model import Fixed
from methodology import airborne_verification, electrical, motion, joint_airborne, adaptive_airborne, source_fingerprints
from methodology.joint_airborne import JointDynamics
from methodology.cad import magnetics

folder=Path(__file__).resolve().parent
completed=json.loads((folder/'nominal_checkpoint.json').read_text())
construction=magnetics.ConstructionModel()
plant=motion.MotionPlant(construction)
height=plant.geometry.center_of_mass_height+(plant.geometry.max_flight_gap-plant.geometry.gap)*Fixed.airborne_reference_gap_fraction
plant.reset(position=np.array([0.,0.,height]))
drivers=electrical.CoilDrivers(construction,tuple(plant.array.identities))
for name in ('command','voltage'):
 setattr(drivers,name,np.array(completed['drivers'][name]))
matrix=plant.magnetic_matrix(window_center=(0,0))
field=np.array(completed['joint']['hall_filter']) / -np.expm1(-2*np.pi*Fixed.hall_sensor_bandwidth*Fixed.adaptive_max_step)
dynamics=JointDynamics(plant,drivers,len(field),lambda plant:matrix,lambda plant,currents:field)
state=dynamics.pack(plant,np.zeros_like(field))
rows=[]
for with_filter in (True,False):
 dynamics.field_signal=(lambda plant,currents:field) if with_filter else (lambda plant,currents:np.zeros_like(field))
 dynamics.evaluations=0
 started=perf_counter()
 segments=dynamics.integrate(0.,Fixed.adaptive_max_step,state)
 result=segments[-1].y[:,-1]
 rows.append(dict(with_filter_startup=with_filter,rhs_evaluations=dynamics.evaluations,
  accepted_steps=sum(len(segment.t)-1 for segment in segments),elapsed_s=perf_counter()-started,
  final_mechanical_and_current_state=result[:dynamics.current.stop].tolist()))
report=dict(scope='Reduced cost isolation with frozen force map and constant Hall forcing of recorded startup magnitude; not a replay or flight qualification',
 source_sha256=dict(plant.provenance, **source_fingerprints(joint_airborne.__file__, electrical.__file__, adaptive_airborne.__file__)),duration_s=Fixed.adaptive_max_step,cases=rows,
 limitations='Both cases hold magnetic coefficients fixed in a newly generated CAD plant. The second suppresses only Hall-filter startup. No calibrated noise, ADC or controller recovery is tested.')
print(json.dumps(report))

