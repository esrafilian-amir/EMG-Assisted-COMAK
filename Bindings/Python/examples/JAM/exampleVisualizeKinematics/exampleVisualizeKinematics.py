"""JAM example: build a knee model with contact and ligaments and run the JointMechanicsTool on measured kinematics

Python version of exampleVisualizeKinematics.m.

Run from any folder with the OpenSim Python bindings available, e.g.
    python exampleVisualizeKinematics.py
All paths are relative to the folder of this script (the same layout as the
MATLAB example: ../models/... holds the shared models and experimental data).
"""
import os
import shutil
import opensim as osim

# All relative paths below refer to the folder of this script
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Setup Environment and Folders
osim.Logger.setLevelString('Info')

os.makedirs('./inputs', exist_ok=True)
os.makedirs('./inputs/Geometry', exist_ok=True)
os.makedirs('./results', exist_ok=True)
os.makedirs('./results/passive', exist_ok=True)
os.makedirs('./results/graphics', exist_ok=True)



# Build Model

# Copy Geometries to example dir
geometry_dir = '../models/healthy/lenhart2015/Geometry/'
local_geometry_dir = './inputs/Geometry/'

femur_bone_mesh_file = 'lenhart2015-R-femur-bone.stl'
tibia_bone_mesh_file = 'lenhart2015-R-tibia-bone.stl'
patella_bone_mesh_file = 'lenhart2015-R-patella-bone.stl'
femur_cartilage_mesh_file = 'lenhart2015-R-femur-cartilage.stl'
tibia_cartilage_mesh_file = 'lenhart2015-R-tibia-cartilage.stl'
patella_cartilage_mesh_file = 'lenhart2015-R-patella-cartilage.stl'

shutil.copyfile(geometry_dir + femur_bone_mesh_file, local_geometry_dir + femur_bone_mesh_file)
shutil.copyfile(geometry_dir + tibia_bone_mesh_file, local_geometry_dir + tibia_bone_mesh_file)
shutil.copyfile(geometry_dir + patella_bone_mesh_file, local_geometry_dir + patella_bone_mesh_file)

shutil.copyfile(geometry_dir + femur_cartilage_mesh_file, local_geometry_dir + femur_cartilage_mesh_file)
shutil.copyfile(geometry_dir + tibia_cartilage_mesh_file, local_geometry_dir + tibia_cartilage_mesh_file)
shutil.copyfile(geometry_dir + patella_cartilage_mesh_file, local_geometry_dir + patella_cartilage_mesh_file)

osim.ModelVisualizer.addDirToGeometrySearchPaths(local_geometry_dir)

model = osim.Model()


# Add Bodies
# Mass properties are arbitrary as the model will be kinematically driven,
# so no dynamic simulation is performed
massCenter = osim.Vec3(0.0)
inertia = osim.Inertia(0.1, 0.1, 0.1, 0, 0, 0)

femur = osim.Body()
femur.setName('femur')
femur.setMass(1.0)
femur.setMassCenter(massCenter)
femur.setInertia(inertia)
# femur.attachGeometry(osim.Mesh(femur_bone_mesh_file))
model.addBody(femur)

tibia = osim.Body()
tibia.setName('tibia')
tibia.setMass(1.0)
tibia.setMassCenter(massCenter)
tibia.setInertia(inertia)
# tibia.attachGeometry(osim.Mesh(tibia_bone_mesh_file))
model.addBody(tibia)

patella = osim.Body()
patella.setName('patella')
patella.setMass(1.0)
patella.setMassCenter(massCenter)
patella.setInertia(inertia)
patella.attachGeometry(osim.Mesh(patella_bone_mesh_file))
model.addBody(patella)

# Add Joints

ground_transform = osim.SpatialTransform()

ground_transform.get_rotation1().set_coordinates(0, 'ground_femur_rz_r')
ground_transform.get_rotation1().set_axis(osim.Vec3(0, 0, 1))
ground_transform.get_rotation1().set_function(osim.LinearFunction())

ground_transform.get_rotation2().set_coordinates(0, 'ground_femur_rx_r')
ground_transform.get_rotation2().set_axis(osim.Vec3(1, 0, 0))
ground_transform.get_rotation2().set_function(osim.LinearFunction())

ground_transform.get_rotation3().set_coordinates(0, 'ground_femur_ry_r')
ground_transform.get_rotation3().set_axis(osim.Vec3(0, 1, 0))
ground_transform.get_rotation3().set_function(osim.LinearFunction())

ground_transform.get_translation1().set_coordinates(0, 'ground_femur_tx_r')
ground_transform.get_translation1().set_axis(osim.Vec3(1, 0, 0))
ground_transform.get_translation1().set_function(osim.LinearFunction())

ground_transform.get_translation2().set_coordinates(0, 'ground_femur_ty_r')
ground_transform.get_translation2().set_axis(osim.Vec3(0, 1, 0))
ground_transform.get_translation2().set_function(osim.LinearFunction())

ground_transform.get_translation3().set_coordinates(0, 'ground_femur_tz_r')
ground_transform.get_translation3().set_axis(osim.Vec3(0, 0, 1))
ground_transform.get_translation3().set_function(osim.LinearFunction())

ground_femur_r = osim.CustomJoint('ground_femur_r',
    model.getGround(), osim.Vec3(0.0), osim.Vec3(0.0), femur, osim.Vec3(0.0), osim.Vec3(0.0),
    ground_transform)

model.addJoint(ground_femur_r)

knee_transform = osim.SpatialTransform()

knee_transform.get_rotation1().set_coordinates(0, 'knee_flex_r')
knee_transform.get_rotation1().set_axis(osim.Vec3(0, 0, 1))
knee_transform.get_rotation1().set_function(osim.LinearFunction())

knee_transform.get_rotation2().set_coordinates(0, 'knee_add_r')
knee_transform.get_rotation2().set_axis(osim.Vec3(1, 0, 0))
knee_transform.get_rotation2().set_function(osim.LinearFunction())

knee_transform.get_rotation3().set_coordinates(0, 'knee_rot_r')
knee_transform.get_rotation3().set_axis(osim.Vec3(0, 1, 0))
knee_transform.get_rotation3().set_function(osim.LinearFunction())

knee_transform.get_translation1().set_coordinates(0, 'knee_ant_r')
knee_transform.get_translation1().set_axis(osim.Vec3(1, 0, 0))
knee_transform.get_translation1().set_function(osim.LinearFunction())

knee_transform.get_translation2().set_coordinates(0, 'knee_sup_r')
knee_transform.get_translation2().set_axis(osim.Vec3(0, 1, 0))
knee_transform.get_translation2().set_function(osim.LinearFunction())

knee_transform.get_translation3().set_coordinates(0, 'knee_lat_r')
knee_transform.get_translation3().set_axis(osim.Vec3(0, 0, 1))
knee_transform.get_translation3().set_function(osim.LinearFunction())

knee_r = osim.CustomJoint('knee_r',
    femur, osim.Vec3(0.0), osim.Vec3(0.0), tibia, osim.Vec3(0.0), osim.Vec3(0.0),
    knee_transform)

model.addJoint(knee_r)

pf_transform = osim.SpatialTransform()

pf_transform.get_rotation1().set_coordinates(0, 'pf_flex_r')
pf_transform.get_rotation1().set_axis(osim.Vec3(0, 0, 1))
pf_transform.get_rotation1().set_function(osim.LinearFunction())

pf_transform.get_rotation2().set_coordinates(0, 'pf_rot_r')
pf_transform.get_rotation2().set_axis(osim.Vec3(1, 0, 0))
pf_transform.get_rotation2().set_function(osim.LinearFunction())

pf_transform.get_rotation3().set_coordinates(0, 'pf_tilt_r')
pf_transform.get_rotation3().set_axis(osim.Vec3(0, 1, 0))
pf_transform.get_rotation3().set_function(osim.LinearFunction())

pf_transform.get_translation1().set_coordinates(0, 'pf_ant_r')
pf_transform.get_translation1().set_axis(osim.Vec3(1, 0, 0))
pf_transform.get_translation1().set_function(osim.LinearFunction())

pf_transform.get_translation2().set_coordinates(0, 'pf_sup_r')
pf_transform.get_translation2().set_axis(osim.Vec3(0, 1, 0))
pf_transform.get_translation2().set_function(osim.LinearFunction())

pf_transform.get_translation3().set_coordinates(0, 'pf_lat_r')
pf_transform.get_translation3().set_axis(osim.Vec3(0, 0, 1))
pf_transform.get_translation3().set_function(osim.LinearFunction())

pf_r = osim.CustomJoint('pf_r',
    femur, osim.Vec3(0.0), osim.Vec3(0.0), patella, osim.Vec3(0.0), osim.Vec3(0.0),
    pf_transform)

model.addJoint(pf_r)

femur_cnt_mesh = osim.Smith2018ContactMesh('femur_cartilage',
                                           femur_cartilage_mesh_file, femur)
model.addContactGeometry(femur_cnt_mesh)


tibia_cnt_mesh = osim.Smith2018ContactMesh('tibia_cartilage',
                                           tibia_cartilage_mesh_file, tibia)
model.addContactGeometry(tibia_cnt_mesh)


patella_cnt_mesh = osim.Smith2018ContactMesh('patella_cartilage',
                                             patella_cartilage_mesh_file, patella)
model.addContactGeometry(patella_cnt_mesh)

tf_contact = osim.Smith2018ArticularContactForce('knee_contact',
                                                 femur_cnt_mesh, tibia_cnt_mesh)
model.addForce(tf_contact)

pf_contact = osim.Smith2018ArticularContactForce('pf_contact',
                                                 femur_cnt_mesh, patella_cnt_mesh)
model.addForce(pf_contact)

# Add MCL Ligament
mcl_wrap = osim.WrapEllipsoid()
mcl_wrap.set_active(True)
mcl_wrap.set_dimensions(osim.Vec3(0.03, 0.018, 0.012))
mcl_wrap.set_translation(osim.Vec3(0.001, -0.0346, -0.0187))
mcl_wrap.set_xyz_body_rotation(osim.Vec3(-0.105, -0.035, 0.182))
mcl_wrap.set_quadrant('all')
tibia.addWrapObject(mcl_wrap)

MCL1_pt1 = osim.Vec3(0.0115, 0.0075, -0.0405)
MCL1_pt2 = osim.Vec3(0.0149, -0.0618, -0.0153)
MCL1 = osim.Blankevoort1991Ligament('MCL1', femur, MCL1_pt1, tibia, MCL1_pt2)
model.addForce(MCL1)
MCL1.updGeometryPath().addPathWrap(mcl_wrap)

MCL2_pt1 = osim.Vec3(0.0052, 0.0083, -0.0406)
MCL2_pt2 = osim.Vec3(0.0076, -0.0611, -0.0191)
MCL2 = osim.Blankevoort1991Ligament('MCL2', femur, MCL2_pt1, tibia, MCL2_pt2)
model.addForce(MCL2)
MCL2.updGeometryPath().addPathWrap(mcl_wrap)

MCL3_pt1 = osim.Vec3(-0.0004, 0.0072, -0.0388)
MCL3_pt2 = osim.Vec3(0.0012, -0.0619, -0.0188)
MCL3 = osim.Blankevoort1991Ligament('MCL3', femur, MCL3_pt1, tibia, MCL3_pt2)
model.addForce(MCL3)
MCL3.updGeometryPath().addPathWrap(mcl_wrap)

state = model.initSystem()

MCL1.setSlackLengthFromReferenceStrain(-0.03, state)
MCL1.set_linear_stiffness(1000)
MCL2.setSlackLengthFromReferenceStrain(0.0, state)
MCL2.set_linear_stiffness(1000)
MCL3.setSlackLengthFromReferenceStrain(0.03, state)
MCL3.set_linear_stiffness(1000)

# Print model file
model_file = './inputs/knee.osim'
model.printToXML(model_file)

# Perform Analysis with JointMechanicsTool
basename = 'passive'
jnt_mech = osim.JointMechanicsTool()
jnt_mech.set_model_file(model_file)
jnt_mech.set_input_states_file(
    '../models/healthy/experimental_data/dynamic_mri/test_passive_coordinates.sto')
jnt_mech.set_results_file_basename('passive')
jnt_mech.set_results_directory('./results/passive/joint-mechanics')
jnt_mech.set_start_time(-1)
jnt_mech.set_stop_time(-1)
jnt_mech.set_normalize_to_cycle(False)
jnt_mech.set_contacts(0, 'all')
jnt_mech.set_ligaments(0, 'all')
jnt_mech.set_muscles(0, 'none')
jnt_mech.set_attached_geometry_bodies(0, '/bodyset/femur')
jnt_mech.set_attached_geometry_bodies(1, '/bodyset/tibia')
jnt_mech.set_attached_geometry_bodies(2, '/bodyset/patella')
jnt_mech.set_output_orientation_frame('ground')
jnt_mech.set_output_position_frame('ground')
jnt_mech.set_write_vtp_files(True)
jnt_mech.set_write_h5_file(True)
jnt_mech.set_h5_kinematics_data(True)
jnt_mech.set_h5_states_data(True)
jnt_mech.set_use_visualizer(True)
jnt_mech.printToXML('./inputs/' + basename + '_joint_mechanics_settings.xml')

print('Running JointMechanicsTool...')
jnt_mech.run()
