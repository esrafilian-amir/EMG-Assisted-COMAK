"""JAM example: EMG-assisted COMAK with a muscle calibration

Calibrates the muscle-tendon parameters of the Lenhart model (max_isometric_force,
optimal_fiber_length, tendon_slack_length) and the shape of the EMG to activation relation, so
that the muscles, driven by the EMG, balance the inverse dynamics moments of the walking trial
(hip, knee and ankle flexion) and cause a small knee contact force. The workflow is:

  1. COMAK with the original model and the EMG (use_tendon_compliance on). This gives the
     secondary knee and patella coordinates of the trial, which are held fixed in the next step.
  2. Muscle calibration (run_muscle_calibration.py, settings in muscle_calibration_setup.xml).
     It writes a new model and a calibrated EMG file.
  3. COMAK again with the calibrated model and EMG.
  4. Plots of the EMG against the muscle activations and of the knee contact force of both runs.

Run from any folder with the OpenSim Python bindings available (numpy, scipy and matplotlib
are needed too):
    python exampleWalking_EMG_Calibration.py
All paths are relative to the folder of this script (../models holds the shared models and
experimental data, as in exampleWalking_EMG_Assisted.py). The calibrated model is written next
to the original one, because the contact meshes and the geometry are found relative to the model
file. The whole example takes about 25 minutes (two COMAK runs and the joint mechanics).
"""
import os
import sys

import opensim as osim

# All relative paths below refer to the folder of this script
here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here)
sys.path.insert(0, here)

import analyze_runs  # noqa: E402
import run_comak_calibrated  # noqa: E402
import run_muscle_calibration  # noqa: E402

osim.Logger.setLevelString('Info')

model_file = '../models/GCD/lenhart2015/P1_lenhart2015.osim'
emg_file = '../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_emg.mot'
results_basename = 'walking'
baseline_dir = './results/comak-baseline'
calibration_dir = './results/muscle-calibration'
calibrated_dir = './results/comak-calibrated'
calibration_setup = './muscle_calibration_setup.xml'
baseline_setup = './inputs/comak_settings_baseline.xml'

os.makedirs('./inputs', exist_ok=True)
os.makedirs('./results', exist_ok=True)

# ---------------------------------------------------------------------------------------------
# 1. COMAK with the original model and the EMG
# ---------------------------------------------------------------------------------------------
comak = osim.COMAKTool()
comak.set_setup_version(1)
comak.set_model_file(model_file)
comak.set_coordinates_file('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_ik.mot')
comak.set_external_loads_file('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_grf_Setup.xml')
comak.set_results_directory(baseline_dir)
comak.set_results_prefix(results_basename)
comak.set_replace_force_set(False)
comak.set_force_set_file('../models/GCD/lenhart2015/lenhart2015_reserve_actuators.xml')
comak.set_start_time(0.04)  # the EMG drives the muscles after the 0.04 s electromechanical delay
comak.set_stop_time(0.8)
comak.set_time_step(0.01)
comak.set_lowpass_filter_frequency(6)
comak.set_print_processed_input_kinematics(False)

prescribed = ['/jointset/gnd_pelvis/pelvis_tx', '/jointset/gnd_pelvis/pelvis_ty',
              '/jointset/gnd_pelvis/pelvis_tz', '/jointset/gnd_pelvis/pelvis_tilt',
              '/jointset/gnd_pelvis/pelvis_list', '/jointset/gnd_pelvis/pelvis_rot',
              '/jointset/subtalar_r/subt_angle_r', '/jointset/mtp_r/mtp_angle_r',
              '/jointset/hip_l/hip_flex_l', '/jointset/hip_l/hip_add_l', '/jointset/hip_l/hip_rot_l',
              '/jointset/pf_l/pf_l_r3', '/jointset/pf_l/pf_l_tx', '/jointset/pf_l/pf_l_ty',
              '/jointset/knee_l/knee_flex_l', '/jointset/ankle_l/ankle_flex_l',
              '/jointset/subtalar_l/subt_angle_l', '/jointset/mtp_l/mtp_angle_l',
              '/jointset/pelvis_torso/lumbar_ext', '/jointset/pelvis_torso/lumbar_latbend',
              '/jointset/pelvis_torso/lumbar_rot', '/jointset/torso_neckhead/neck_ext',
              '/jointset/torso_neckhead/neck_latbend', '/jointset/torso_neckhead/neck_rot',
              '/jointset/acromial_r/arm_add_r', '/jointset/acromial_r/arm_flex_r',
              '/jointset/acromial_r/arm_rot_r', '/jointset/elbow_r/elbow_flex_r',
              '/jointset/radioulnar_r/pro_sup_r', '/jointset/radius_hand_r/wrist_flex_r',
              '/jointset/acromial_l/arm_add_l', '/jointset/acromial_l/arm_flex_l',
              '/jointset/acromial_l/arm_rot_l', '/jointset/elbow_l/elbow_flex_l',
              '/jointset/radioulnar_l/pro_sup_l', '/jointset/radius_hand_l/wrist_flex_l']
for i, path in enumerate(prescribed):
    comak.set_prescribed_coordinates(i, path)

primary = ['/jointset/hip_r/hip_flex_r', '/jointset/hip_r/hip_add_r', '/jointset/hip_r/hip_rot_r',
           '/jointset/knee_r/knee_flex_r', '/jointset/ankle_r/ankle_flex_r']
for i, path in enumerate(primary):
    comak.set_primary_coordinates(i, path)

# Secondary coordinates of the right knee and patella; max_change 0 leaves them free
secondary_coord_set = osim.COMAKSecondaryCoordinateSet()
secondary_coord = osim.COMAKSecondaryCoordinate()
for joint, names in (('knee_r', ['knee_add_r', 'knee_rot_r', 'knee_tx_r', 'knee_ty_r', 'knee_tz_r']),
                     ('pf_r', ['pf_flex_r', 'pf_rot_r', 'pf_tilt_r', 'pf_tx_r', 'pf_ty_r', 'pf_tz_r'])):
    for name in names:
        secondary_coord.setName(name)
        secondary_coord.set_max_change(0)
        secondary_coord.set_coordinate('/jointset/%s/%s' % (joint, name))
        secondary_coord_set.cloneAndAppend(secondary_coord)
comak.set_COMAKSecondaryCoordinateSet(secondary_coord_set)

comak.set_settle_secondary_coordinates_at_start(True)
comak.set_settle_threshold(5e-4)
comak.set_settle_accuracy(1e-2)
comak.set_settle_internal_step_limit(10000)
comak.set_print_settle_sim_results(True)
comak.set_settle_sim_results_directory(baseline_dir)
comak.set_settle_sim_results_prefix('walking_settle_sim')
comak.set_max_iterations(25)
comak.set_udot_tolerance(1)
comak.set_udot_worse_case_tolerance(50)
# The calibration fits the optimal fiber length and the tendon slack length, so the muscle force
# model of COMAK has to depend on them: the tendon compliance (experimental). The fiber
# length, tendon slack length and the force-length curve then act on the muscle force.
comak.set_use_force_length_velocity(False)
comak.set_use_passive_muscle_force(False)
comak.set_use_tendon_compliance(True)
comak.set_force_tolerance_rotation(0.1)  # N*m
comak.set_force_tolerance_translation(2.0)  # N
comak.set_unit_udot_epsilon(1e-6)
comak.set_optimization_scale_delta_coord(1)
comak.set_ipopt_diagnostics_level(3)
comak.set_ipopt_max_iterations(500)
comak.set_ipopt_convergence_tolerance(1e-4)
comak.set_ipopt_constraint_tolerance(1e-4)
comak.set_ipopt_limited_memory_history(200)
comak.set_ipopt_nlp_scaling_max_gradient(10000)
comak.set_ipopt_nlp_scaling_min_value(1e-8)
comak.set_ipopt_obj_scaling_factor(1)
comak.set_activation_exponent(2)
# the contact energy is a regularization towards low contact loads: 0.1 changes the muscle
# activations by up to 0.1 and the secondary coordinates by about 1 mm, 1 means that it dominates
comak.set_contact_energy_weight(0.1)
comak.set_non_muscle_actuator_weight(10)
comak.set_model_assembly_accuracy(1e-12)
comak.set_use_visualizer(False)
comak.set_verbose(0)

# EMG: the enveloped EMG (0 to 1) of the channels drives the muscles they are mapped to
# (label in the EMG file, muscle in the model). The EMG tracking weight (gamma) is 100 so that
# the activations follow the EMG closely.
comak.set_emg_file(emg_file)
comak.set_emg_electromechanical_delay(0.04)
comak.set_is_emg_assisted(True)
emg_gamma_weight = 100
muscle_mapper = [('semimem', 'semimem_r'), ('bifem', 'bfsh_r'), ('bifem', 'bflh_r'),
                 ('vasmed', 'vasmed_r'), ('vaslat', 'vaslat_r'), ('rf', 'recfem_r'),
                 ('medgas', 'gasmed_r'), ('latgas', 'gaslat_r'), ('tfl', 'tfl_r'),
                 ('tibant', 'tibant_r'), ('peronl', 'perlong_r'), ('soleus', 'soleus_r'),
                 ('addmagnus', 'addmagProx_r'), ('addmagnus', 'addmagMid_r'),
                 ('addmagnus', 'addmagDist_r'), ('addmagnus', 'addmagIsch_r'),
                 ('gmax', 'glmax1_r'), ('gmax', 'glmax2_r'), ('gmax', 'glmax3_r'),
                 ('gmed', 'glmed1_r'), ('gmed', 'glmed2_r'), ('gmed', 'glmed3_r'),
                 ('sartorius', 'sart_r')]
cost_fun_param_set = osim.COMAKCostFunctionParameterSet()
cost_fun_param = osim.COMAKCostFunctionParameter()
for emg_label, muscle_name in muscle_mapper:
    cost_fun_param.setName(muscle_name)
    cost_fun_param.set_actuator('/forceset/' + muscle_name)
    cost_fun_param.set_emg_gamma_weight(emg_gamma_weight)
    cost_fun_param.set_emg_mapping(emg_label)
    cost_fun_param.set_activation_lower_bound(osim.Constant(0.01))
    cost_fun_param.set_activation_upper_bound(osim.Constant(1.0))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)
comak.set_COMAKCostFunctionParameterSet(cost_fun_param_set)

# The joint mechanics (contact forces) is run right after COMAK; it is only needed to compare the
# knee contact force of the two runs, so the baseline run gets it too.
comak.set_run_joint_mechanics(True)
comak.upd_joint_mechanics().set_results_directory(baseline_dir + '/joint_mechanics')

comak.printToXML(baseline_setup)
print('Running COMAK with the original model...')
comak.run()

# ---------------------------------------------------------------------------------------------
# 2. Muscle calibration
# The trials, the muscle groups, the bounds and the weight of the knee contact force are in
# muscle_calibration_setup.xml. Several trials can be listed there; the parameters are then the
# ones that fit all of them. It writes the model (next to the original one) and the EMG.
# ---------------------------------------------------------------------------------------------
print('Running the muscle calibration...')
run_muscle_calibration.main([calibration_setup, '--recompute-geometry'])

# ---------------------------------------------------------------------------------------------
# 3. COMAK again with the calibrated model and EMG (same settings, the model and EMG replaced)
# ---------------------------------------------------------------------------------------------
calibrated_model = '../models/GCD/lenhart2015/P1_lenhart2015_emg_calibrated.osim'
calibrated_emg = calibration_dir + '/calibrated_emg.sto'
print('Running COMAK with the calibrated model...')
run_comak_calibrated.main(['--comak-setup', baseline_setup, '--cwd', here,
                           '--model', calibrated_model, '--emg', calibrated_emg,
                           '--results', calibrated_dir, '--joint-mechanics'])

# ---------------------------------------------------------------------------------------------
# 4. Comparison: the EMG against the muscle activations (one subplot per EMG channel), the
# knee contact force and a table of the agreement with the EMG
# ---------------------------------------------------------------------------------------------
analyze_runs.main(['--comak-setup', baseline_setup, '--comak-cwd', here, '--emg', emg_file,
                   '--run', 'original model=' + baseline_dir,
                   '--run', 'calibrated model=%s=%s' % (calibrated_dir, calibrated_emg),
                   '--prefix', results_basename, '--out', './results/comparison'])
print('Plots and summary: ./results/comparison')
