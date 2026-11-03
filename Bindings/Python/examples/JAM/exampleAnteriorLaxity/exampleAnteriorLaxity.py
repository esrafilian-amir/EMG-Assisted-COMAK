"""JAM example: anterior laxity of a healthy and an ACL deficient knee (ForsimTool + JointMechanicsTool)

Python version of exampleAnteriorLaxity.m.

Run from any folder with the OpenSim Python bindings available, e.g.
    python exampleAnteriorLaxity.py
All paths are relative to the folder of this script (the same layout as the
MATLAB example: ../models/... holds the shared models and experimental data).
Requires numpy and scipy (pchip interpolation); matplotlib is optional and only
used to plot the simulation inputs.
"""
import os
import numpy as np
from scipy.interpolate import PchipInterpolator
import opensim as osim

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

# All relative paths below refer to the folder of this script
os.chdir(os.path.dirname(os.path.abspath(__file__)))


def colon(start, step, stop):
    """MATLAB start:step:stop"""
    n = int(np.floor((stop - start) / step + 1e-10))
    return start + step * np.arange(n + 1)


def table_from_columns(time, columns):
    """Equivalent of osimTableFromStruct: TimeSeriesTable with one column per entry"""
    labels = osim.StdVectorString()
    for label in columns:
        labels.append(label)
    table = osim.TimeSeriesTable()
    table.setColumnLabels(labels)
    row = osim.RowVector(len(columns), float('nan'))
    for i_row, t in enumerate(time):
        for i_col, values in enumerate(columns.values()):
            row[i_col] = float(values[i_row])
        table.appendRow(float(t), row)
    return table


osim.Logger.setLevelString('Info')

os.makedirs('./inputs', exist_ok=True)
os.makedirs('./results', exist_ok=True)
os.makedirs('./results/graphics', exist_ok=True)

model_file = '../models/healthy/current/full_body_healthy_knee.osim'

force_magnitude = 100  # 100 N anterior force, similar to KT-1000 arthrometer
force_point_height = -0.1  # Apply at the tibial tuberosity height

# Simulation Time
# Simulation consists of four phases:
# settle : allow knee to settle into equilbrium
# flex   : hip and knee flexion
# settle : allow knee to settle into equilbrium
# force  : ramp up the anterior force

time_step = 0.01

settle1_duration = 0.5
flex_duration = 1.0
settle2_duration = 0.5
force_duration = 1.0


settle1_time = colon(0, time_step, settle1_duration)

flex_time = colon(
    settle1_duration + time_step,
    time_step,
    settle1_duration + flex_duration)

settle2_time = colon(
    settle1_duration + flex_duration + time_step,
    time_step,
    settle1_duration + flex_duration + settle2_duration)

force_time = colon(
    settle1_duration + flex_duration + settle2_duration + time_step,
    time_step,
    settle1_duration + flex_duration + settle2_duration + force_duration)

time = np.concatenate([settle1_time, flex_time, settle2_time, force_time])

time_points = [0, settle1_duration,
               settle1_duration + flex_duration,
               settle1_duration + flex_duration + settle2_duration,
               settle1_duration + flex_duration + settle2_duration + force_duration]

num_settle1_steps = len(settle1_time)
num_flex_steps = len(flex_time)
num_settle2_steps = len(settle2_time)
num_force_steps = len(force_time)
num_steps = len(time)

# Create Input Files
# Prescribed Coordinates File
#----------------------------
prescribed_coordinates_file = './inputs/prescribed_coordinates.sto'

max_hip_flex = 25
max_knee_flex = 25

coord_data = {}

hip_flex = [0, 0, max_hip_flex, max_hip_flex, max_hip_flex]
knee_flex = [0, 0, max_knee_flex, max_knee_flex, max_knee_flex]

smooth_hip_flex = PchipInterpolator(time_points, hip_flex)(time)
smooth_knee_flex = PchipInterpolator(time_points, knee_flex)(time)

coord_data['hip_flex_r'] = smooth_hip_flex
coord_data['knee_flex_r'] = smooth_knee_flex

coord_table = table_from_columns(time, coord_data)

osim.STOFileAdapter.write(coord_table, prescribed_coordinates_file)

# Plot Prescribed coordinates
#----------------------------
if plt is not None:
    coord_fig = plt.figure('INPUT: Prescribed Coordinates', figsize=(6.67, 3.0))

    plt.subplot(1, 2, 1)
    plt.plot(time, coord_data['hip_flex_r'], linewidth=2)
    plt.ylim([0.0, 35])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [$^o$]')
    plt.title('Hip Flexion (hip\\_flex\\_r)')

    plt.subplot(1, 2, 2)
    plt.plot(time, coord_data['knee_flex_r'], linewidth=2)
    plt.ylim([0.0, 35])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [$^o$]')
    plt.title('Knee Flexion (knee\\_flex\\_r)')

    coord_fig.tight_layout()
    coord_fig.savefig('./results/graphics/prescribed_coordinates.png')

# External Loads Files
#---------------------

# write .sto file
external_loads_sto_file = 'external_loads.sto'

force_vx = [0, 0, 0, 0, force_magnitude]
smooth_force_vx = PchipInterpolator(time_points, force_vx)(time)

force_data = {}
force_data['tibia_proximal_r_force_vx'] = smooth_force_vx
force_data['tibia_proximal_r_force_vy'] = np.zeros(num_steps)
force_data['tibia_proximal_r_force_vz'] = np.zeros(num_steps)
force_data['tibia_proximal_r_force_px'] = np.zeros(num_steps)
force_data['tibia_proximal_r_force_py'] = np.ones(num_steps) * force_point_height
force_data['tibia_proximal_r_force_pz'] = np.zeros(num_steps)
force_data['tibia_proximal_r_torque_x'] = np.zeros(num_steps)
force_data['tibia_proximal_r_torque_y'] = np.zeros(num_steps)
force_data['tibia_proximal_r_torque_z'] = np.zeros(num_steps)

force_table = table_from_columns(time, force_data)

force_table.addTableMetaDataString('header', 'Anterior Tibial External Force')

osim.STOFileAdapter.write(force_table, './inputs/' + external_loads_sto_file)

# write .xml file
external_loads_xml_file = './inputs/external_loads.xml'

ext_force = osim.ExternalForce()
ext_force.setName('AnteriorForce')
ext_force.set_applied_to_body('tibia_proximal_r')
ext_force.set_force_expressed_in_body('tibia_proximal_r')
ext_force.set_point_expressed_in_body('tibia_proximal_r')
ext_force.set_force_identifier('tibia_proximal_r_force_v')
ext_force.set_point_identifier('tibia_proximal_r_force_p')
ext_force.set_torque_identifier('tibia_proximal_r_torque_')

ext_loads = osim.ExternalLoads()
ext_loads.setDataFileName(external_loads_sto_file)
ext_loads.cloneAndAppend(ext_force)  # MATLAB: adoptAndAppend
ext_loads.printToXML(external_loads_xml_file)

# Plot External Loads
if plt is not None:
    ext_loads_fig = plt.figure('INPUT: External Loads', figsize=(3.33, 3.0))
    plt.plot(time, force_data['tibia_proximal_r_force_vx'], linewidth=2)
    plt.ylim([0.0, 100])
    plt.xlabel('Time [s]')
    plt.ylabel('Anterior Force [N]')
    plt.title('External Loads on the Tibia')

    ext_loads_fig.tight_layout()
    ext_loads_fig.savefig('./results/graphics/external_loads.png')

# Create ACL deficient model

model = osim.Model(model_file)
# Note: in MATLAB acld_model = model refers to the same Java object, so removing the ACL
# there also changes the healthy model; here the ACL deficient model is a separate copy
acld_model = osim.Model(model_file)

# Remove ACL Ligaments
ACL_indices = []
for i in range(acld_model.getForceSet().getSize()):
    force = acld_model.getForceSet().get(i)
    if 'ACL' in force.getName():
        ACL_indices.append(i)

# remove from the end so the remaining indices stay valid
for i in reversed(ACL_indices):
    acld_model.updForceSet().remove(i)

acld_model.initSystem()
acld_model_file = './inputs/acl_deficient_knee.osim'
acld_model.printToXML(acld_model_file)

# Perform Simulation with ForsimTool
# Healthy
healthy_forsim_result_dir = './results/healthy_forsim'
healthy_basename = 'healthy'

forsim = osim.ForsimTool()
forsim.setModel(model)
forsim.set_results_directory(healthy_forsim_result_dir)
forsim.set_results_file_basename(healthy_basename)
forsim.set_start_time(-1)
forsim.set_stop_time(-1)
forsim.set_integrator_accuracy(1e-2)  # Note this should be 1e-6 for research
# Set all muscles to 2% activation to represent passive state
forsim.set_constant_muscle_control(0.02)
forsim.set_use_activation_dynamics(False)  # MATLAB example: set_ignore_activation_dynamics(true)
forsim.set_use_tendon_compliance(False)  # MATLAB example: set_ignore_tendon_compliance(true)
forsim.set_unconstrained_coordinates(0, '/jointset/knee_r/knee_add_r')
forsim.set_unconstrained_coordinates(1, '/jointset/knee_r/knee_rot_r')
forsim.set_unconstrained_coordinates(2, '/jointset/knee_r/knee_tx_r')
forsim.set_unconstrained_coordinates(3, '/jointset/knee_r/knee_ty_r')
forsim.set_unconstrained_coordinates(4, '/jointset/knee_r/knee_tz_r')
forsim.set_unconstrained_coordinates(5, '/jointset/pf_r/pf_flex_r')
forsim.set_unconstrained_coordinates(6, '/jointset/pf_r/pf_rot_r')
forsim.set_unconstrained_coordinates(7, '/jointset/pf_r/pf_tilt_r')
forsim.set_unconstrained_coordinates(8, '/jointset/pf_r/pf_tx_r')
forsim.set_unconstrained_coordinates(9, '/jointset/pf_r/pf_ty_r')
forsim.set_unconstrained_coordinates(10, '/jointset/pf_r/pf_tz_r')
forsim.set_prescribed_coordinates_file(prescribed_coordinates_file)
forsim.set_external_loads_file(external_loads_xml_file)
forsim.set_use_visualizer(True)
forsim.printToXML('./inputs/healthy_forsim_settings.xml')


print('Running Forsim Tool...')
forsim.run()

# ACL Deficient
acld_forsim_result_dir = './results/acld_forsim'
acld_basename = 'acld'

forsim.setModel(acld_model)
forsim.set_results_directory(acld_forsim_result_dir)
forsim.set_results_file_basename(acld_basename)
forsim.printToXML('./inputs/acld_forsim_settings.xml')

print('Running Forsim Tool...')
forsim.run()
# Perform Analysis with JointMechanicsTool
# Healthy
healthy_jnt_mech_result_dir = './results/healthy_joint_mechanics'

jnt_mech = osim.JointMechanicsTool()
jnt_mech.setModel(model)
jnt_mech.set_input_states_file(
    healthy_forsim_result_dir + '/' + healthy_basename + '_states.sto')
jnt_mech.set_results_file_basename(healthy_basename)
jnt_mech.set_results_directory(healthy_jnt_mech_result_dir)
jnt_mech.set_start_time(-1)
jnt_mech.set_stop_time(-1)
jnt_mech.set_normalize_to_cycle(False)
jnt_mech.set_contacts(0, 'all')
jnt_mech.set_ligaments(0, 'all')
jnt_mech.set_muscles(0, 'none')
jnt_mech.set_attached_geometry_bodies(0, '/bodyset/femur_distal_r')
jnt_mech.set_attached_geometry_bodies(1, '/bodyset/tibia_proximal_r')
jnt_mech.set_attached_geometry_bodies(2, '/bodyset/patella_r')
jnt_mech.set_output_orientation_frame('ground')
jnt_mech.set_output_position_frame('ground')
jnt_mech.set_write_vtp_files(True)
jnt_mech.set_write_h5_file(True)
jnt_mech.set_h5_kinematics_data(True)
jnt_mech.set_h5_states_data(True)
jnt_mech.printToXML('./inputs/healthy_joint_mechanics_settings.xml')

# ACL Deficient
# Note: the MATLAB example uses acld_jnt_mech_result_dir without defining it
acld_jnt_mech_result_dir = './results/acld_joint_mechanics'
jnt_mech.setModel(acld_model)
jnt_mech.set_input_states_file(
    acld_forsim_result_dir + '/' + acld_basename + '_states.sto')
jnt_mech.set_results_file_basename(acld_basename)
jnt_mech.set_results_directory(acld_jnt_mech_result_dir)
jnt_mech.printToXML('./inputs/acld_joint_mechanics_settings.xml')


print('Running JointMechanicsTool...')
jnt_mech.run()
