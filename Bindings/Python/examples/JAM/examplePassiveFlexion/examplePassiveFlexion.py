"""JAM example: passive knee flexion with the ForsimTool and the JointMechanicsTool

Python version of examplePassiveFlexion.m.

Run from any folder with the OpenSim Python bindings available, e.g.
    python examplePassiveFlexion.py
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


# Setup Environment and Folders
osim.Logger.setLevelString('Info')

model_file = '../models/healthy/lenhart2015/lenhart2015.osim'
results_basename = 'passive_flexion'
forsim_result_dir = './results/forsim'
jnt_mech_result_dir = './results/joint-mechanics'

os.makedirs('./inputs', exist_ok=True)
os.makedirs('./results', exist_ok=True)
os.makedirs('./results/graphics', exist_ok=True)


# Create Input Files
prescribed_coord_file = './inputs/prescribed_coordinates.sto'

# Simulation Time
# Simulation consists of two phases:
#   settle : allow unconstraind knee DOFs to settle into equilbrium
#   flex : prescribe the tibiofemoral flexion
# All time units are in seconds

time_step = 0.01

settle_duration = 0.5
flex_duration = 2.0

settle_time = colon(0, time_step, settle_duration)
flex_time = colon(settle_duration + time_step, time_step, flex_duration + settle_duration)

time = np.concatenate([settle_time, flex_time])

time_points = [0, settle_duration, settle_duration + flex_duration]

nSettleSteps = len(settle_time)
nFlexSteps = len(flex_time)

# Prescribe Knee Flexion
max_knee_flex = 90

knee_flex = [0, 0, max_knee_flex]
smooth_knee_flex = PchipInterpolator(time_points, knee_flex)(time)

prescribed_coord_data = {}
prescribed_coord_data['knee_flex_r'] = smooth_knee_flex
prescribed_coord_data['pelvis_tilt'] = np.ones(nSettleSteps + nFlexSteps) * 90

prescribed_coord_table = table_from_columns(time, prescribed_coord_data)

sto = osim.STOFileAdapter()
sto.write(prescribed_coord_table, prescribed_coord_file)

# Plot Simulation Inputs
if plt is not None:
    coord_fig = plt.figure('prescribed_coordinates', figsize=(6.67, 3.0))

    plt.subplot(1, 2, 1)
    plt.plot(time, prescribed_coord_data['pelvis_tilt'], linewidth=2)
    plt.ylim([0.0, 100])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [$^o$]')
    plt.title('Pelvis Tilt (pelvis\\_tilt\\_r)')

    plt.subplot(1, 2, 2)
    plt.plot(time, prescribed_coord_data['knee_flex_r'], linewidth=2)
    plt.ylim([0.0, 100])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [$^o$]')
    plt.title('Knee Flexion (knee\\_flex\\_r)')

    coord_fig.tight_layout()
    coord_fig.savefig('./results/graphics/prescribed_coordinates.png')

# Perform Simulation with ForsimTool
forsim = osim.ForsimTool()
forsim.set_model_file(model_file)
forsim.set_results_directory(forsim_result_dir)
forsim.set_results_file_basename(results_basename)
forsim.set_start_time(-1)
forsim.set_stop_time(-1)
forsim.set_integrator_accuracy(1e-2)  # Note this should be 1e-6 for research
forsim.set_constant_muscle_control(0.02)  # Set all muscles to 2% activation to represent passive state
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
forsim.set_prescribed_coordinates_file(prescribed_coord_file)
# forsim.set_verbose(5)  # ForsimTool has no verbose property in this version
forsim.set_use_visualizer(True)
forsim.printToXML('./inputs/forsim_settings.xml')

print('Running Forsim Tool...')
forsim.run()

# Perform Analysis with JointMechanicsTool

jnt_mech = osim.JointMechanicsTool()
jnt_mech.set_model_file(model_file)
jnt_mech.set_input_states_file(forsim_result_dir + '/' + results_basename + '_states.sto')
jnt_mech.set_results_file_basename(results_basename)
jnt_mech.set_results_directory(jnt_mech_result_dir)
jnt_mech.set_start_time(-1)
jnt_mech.set_stop_time(-1)
jnt_mech.set_normalize_to_cycle(False)
jnt_mech.set_contacts(0, 'all')
jnt_mech.set_ligaments(0, 'all')
jnt_mech.set_muscles(0, 'none')
# jmt.set_muscle_outputs(0,'none')
jnt_mech.set_attached_geometry_bodies(0, '/bodyset/femur_distal_r')
jnt_mech.set_attached_geometry_bodies(1, '/bodyset/tibia_proximal_r')
jnt_mech.set_attached_geometry_bodies(2, '/bodyset/patella_r')
jnt_mech.set_output_orientation_frame('ground')
jnt_mech.set_output_position_frame('ground')
jnt_mech.set_write_vtp_files(True)
jnt_mech.set_write_h5_file(True)
jnt_mech.set_h5_kinematics_data(True)
jnt_mech.set_h5_states_data(True)
jnt_mech.printToXML('./inputs/joint_mechanics_settings.xml')

print('Running JointMechanicsTool...')
jnt_mech.run()
