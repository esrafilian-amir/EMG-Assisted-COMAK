"""JAM example: modify ligament properties and add a wrap object and a ligament to a model

Python version of exampleLigamentAndContactSensitivity.m.

Run from any folder with the OpenSim Python bindings available, e.g.
    python exampleLigamentAndContactSensitivity.py
All paths are relative to the folder of this script (the same layout as the
MATLAB example: ../models/... holds the shared models and experimental data).
"""
import os
import shutil
import opensim as osim

# All relative paths below refer to the folder of this script
os.chdir(os.path.dirname(os.path.abspath(__file__)))

osim.Logger.setLevelString('info')

num_simulations = 3

# Load model
model_file = '../models/healthy/lenhart2015/lenhart2015.osim'

model = osim.Model(model_file)

state = model.initSystem()


os.makedirs('./inputs', exist_ok=True)

os.makedirs('./inputs/shared', exist_ok=True)
if not os.path.isdir('./inputs/shared/Geometry'):
    shutil.copytree('../models/healthy/current/Geometry', './inputs/shared/Geometry')

for i in range(num_simulations):
    dir_name = './inputs/' + str(i)
    os.makedirs(dir_name, exist_ok=True)




# Get ACLpl1 ligament
ACLpl1 = osim.Blankevoort1991Ligament.safeDownCast(model.getComponent('/forceset/ACLpl1'))

# Set Stiffness
# ACLpl1.get_linear_stiffness()
ACLpl1.set_linear_stiffness(333)


# Set Slack Length
# ACLpl1.get_slack_length()
# ACLpl1.set_slack_length()
ACLpl1.setSlackLengthFromReferenceStrain(0.02, state)
# ACLpl1.setSlackLengthFromReferenceForce(20,state)

# Move attachment point
ACLp1_pt0 = osim.PathPoint.safeDownCast(ACLpl1.updGeometryPath().getPathPointSet().get(0))
ACLp1_pt0.set_location(osim.Vec3(1.0, 2.0, 3.0))

# Add wrap object
mcl_wrap = osim.WrapEllipsoid()
mcl_wrap.setName('mcl_wrap')
mcl_wrap.set_active(True)
mcl_wrap.set_dimensions(osim.Vec3(0.03, 0.018, 0.012))
mcl_wrap.set_translation(osim.Vec3(0.001, -0.0346, -0.0187))
mcl_wrap.set_xyz_body_rotation(osim.Vec3(-0.105, -0.035, 0.182))
mcl_wrap.set_quadrant('all')
model.getBodySet().get('tibia_proximal_r').addWrapObject(mcl_wrap)

# Add new ligament
MCL1_pt1 = osim.Vec3(0.0115, 0.0075, -0.0405)
MCL1_pt2 = osim.Vec3(0.0149, -0.0618, -0.0153)
MCL1 = osim.Blankevoort1991Ligament('MCL1',
                                    model.getBodySet().get('femur_distal_r'), MCL1_pt1,
                                    model.getBodySet().get('tibia_proximal_r'), MCL1_pt2)
model.addForce(MCL1)
MCL1.updGeometryPath().addPathWrap(mcl_wrap)
MCL1.set_linear_stiffness(300)
MCL1.set_slack_length(0.05)

model.finalizeConnections()
model.printToXML('./inputs/0/model.osim')
