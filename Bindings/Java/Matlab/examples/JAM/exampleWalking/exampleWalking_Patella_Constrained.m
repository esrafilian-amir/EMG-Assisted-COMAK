%% Setup Environment and Folders
clear;
import org.opensim.modeling.*
Logger.setLevelString('Debug');

model_file = '../models/healthy/lenhart2015/lenhart2015.osim';
results_basename = 'walking';
ik_result_dir = './results/comak-inverse-kinematics';
comak_result_dir = './results/comak';
jnt_mech_result_dir = './results/joint-mechanics';

if(exist('./inputs','dir')~=7)
    mkdir('./inputs')
end
if(exist('./results','dir')~=7)
    mkdir('./results')
end
if(exist('./results/graphics','dir')~=7)
    mkdir('./results/graphics')
end

%% Perform Inverse Kinematics
comak_ik = COMAKInverseKinematicsTool();
comak_ik.set_setup_version(1);
comak_ik.set_model_file(model_file);
comak_ik.set_results_directory(ik_result_dir);
comak_ik.set_results_prefix(results_basename);
comak_ik.set_perform_secondary_constraint_sim(true);
comak_ik.set_secondary_coordinates(0,'/jointset/knee_r/knee_add_r');
comak_ik.set_secondary_coordinates(1,'/jointset/knee_r/knee_rot_r');
comak_ik.set_secondary_coordinates(2,'/jointset/knee_r/knee_tx_r');
comak_ik.set_secondary_coordinates(3,'/jointset/knee_r/knee_ty_r');
comak_ik.set_secondary_coordinates(4,'/jointset/knee_r/knee_tz_r');
comak_ik.set_secondary_coordinates(5,'/jointset/pf_r/pf_rot_r');
comak_ik.set_secondary_coordinates(6,'/jointset/pf_r/pf_tilt_r');
comak_ik.set_secondary_coordinates(7,'/jointset/pf_r/pf_tx_r');
comak_ik.set_secondary_coordinates(8,'/jointset/pf_r/pf_ty_r');
comak_ik.set_secondary_coordinates(9,'/jointset/pf_r/pf_tz_r');
comak_ik.set_constrained_coordinates(0,'/jointset/pf_r/pf_flex_r');
comak_ik.set_secondary_coupled_coordinate('/jointset/knee_r/knee_flex_r');
comak_ik.set_secondary_constraint_sim_settle_threshold(1e-4);
comak_ik.set_secondary_constraint_sim_sweep_time(3.0);
% range of the knee flexion of the sweeps (deg): settled at 0, then swept down to the minimum and up to the maximum
comak_ik.set_secondary_coupled_coordinate_start_value(-10);
comak_ik.set_secondary_coupled_coordinate_stop_value(110);
comak_ik.set_secondary_constraint_sim_integrator_accuracy(1e-2);
comak_ik.set_secondary_constraint_sim_internal_step_limit(10000);
comak_ik.set_secondary_constraint_function_file(...
    './results/comak-inverse-kinematics/secondary_coordinate_constraint_functions.xml');
comak_ik.set_constraint_function_num_interpolation_points(20);
comak_ik.set_print_secondary_constraint_sim_results(true);
comak_ik.set_constrained_model_file('./results/comak-inverse-kinematics/ik_constrained_model.osim');
comak_ik.set_perform_inverse_kinematics(true);
comak_ik.set_marker_file('../models/healthy/experimental_data/motion_analysis/overground_17.trc');

comak_ik.set_output_motion_file('overground_17_ik.mot');
comak_ik.set_time_range(0, 0);
comak_ik.set_time_range(1, 2.36);
comak_ik.set_report_errors(true);
comak_ik.set_report_marker_locations(false);
comak_ik.set_ik_constraint_weight(100);
% Filter frequency (Hz) of the input kinematics and force data. The same value is used by COMAK and by the
% smoothing pass of the inverse kinematics (smooth_ik_second_pass, on by default). -1: no filter (the smoothing
% then uses 40 Hz).
lowpass_filter_frequency = 6;
comak_ik.set_smooth_ik_lowpass_frequency(lowpass_filter_frequency);
comak_ik.set_ik_accuracy(1e-5);
comak_ik.set_use_visualizer(true);
comak_ik.set_verbose(10);


ik_task_set = IKTaskSet();

ik_task=IKMarkerTask();

ik_task.setName('S2');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.ASIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.PSIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.ASIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.PSIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Clavicle');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Clavicle');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Shoulder');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Shoulder');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Bicep');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Elbow');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Forearm');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Wrist');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Bicep');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Elbow');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Forearm');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Wrist');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Knee');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Ankle');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.MT5');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Heel');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Knee');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Ankle');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.MT5');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Heel');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

comak_ik.set_IKTaskSet(ik_task_set);
comak_ik.print('./inputs/comak_inverse_kinematics_settings.xml');

disp('Running COMAKInverseKinematicsTool...')
comak_ik.run();
%% Perform COMAK Simulation

comak = COMAKTool();
comak.set_setup_version(1);
comak.set_model_file(model_file);
comak.set_coordinates_file('./results/comak-inverse-kinematics/overground_17_ik.mot');
comak.set_external_loads_file('../models/healthy/experimental_data/motion_analysis/overground_17_ext_loads.xml'),
comak.set_results_directory(comak_result_dir);
comak.set_results_prefix(results_basename);
comak.set_replace_force_set(false);
comak.set_force_set_file('../models/healthy/lenhart2015/lenhart2015_reserve_actuators.xml');
comak.set_start_time(1.28);
comak.set_stop_time(1.91);
comak.set_time_step(0.01);
comak.set_lowpass_filter_frequency(lowpass_filter_frequency);
comak.set_print_processed_input_kinematics(false);
comak.set_prescribed_coordinates(0,'/jointset/gnd_pelvis/pelvis_tx');
comak.set_prescribed_coordinates(1,'/jointset/gnd_pelvis/pelvis_ty');
comak.set_prescribed_coordinates(2,'/jointset/gnd_pelvis/pelvis_tz');
comak.set_prescribed_coordinates(3,'/jointset/gnd_pelvis/pelvis_tilt');
comak.set_prescribed_coordinates(4,'/jointset/gnd_pelvis/pelvis_list');
comak.set_prescribed_coordinates(5,'/jointset/gnd_pelvis/pelvis_rot');
comak.set_prescribed_coordinates(6,'/jointset/subtalar_r/subt_angle_r');
comak.set_prescribed_coordinates(7,'/jointset/mtp_r/mtp_angle_r');
comak.set_prescribed_coordinates(8,'/jointset/hip_l/hip_flex_l');
comak.set_prescribed_coordinates(9,'/jointset/hip_l/hip_add_l');
comak.set_prescribed_coordinates(10,'/jointset/hip_l/hip_rot_l');
comak.set_prescribed_coordinates(11,'/jointset/pf_l/pf_l_r3');
comak.set_prescribed_coordinates(12,'/jointset/pf_l/pf_l_tx');
comak.set_prescribed_coordinates(13,'/jointset/pf_l/pf_l_ty');
comak.set_prescribed_coordinates(14,'/jointset/knee_l/knee_flex_l');
comak.set_prescribed_coordinates(15,'/jointset/ankle_l/ankle_flex_l');
comak.set_prescribed_coordinates(16,'/jointset/subtalar_l/subt_angle_l');
comak.set_prescribed_coordinates(17,'/jointset/mtp_l/mtp_angle_l');
comak.set_prescribed_coordinates(18,'/jointset/pelvis_torso/lumbar_ext');
comak.set_prescribed_coordinates(19,'/jointset/pelvis_torso/lumbar_latbend');
comak.set_prescribed_coordinates(20,'/jointset/pelvis_torso/lumbar_rot');
comak.set_prescribed_coordinates(21,'/jointset/torso_neckhead/neck_ext');
comak.set_prescribed_coordinates(22,'/jointset/torso_neckhead/neck_latbend');
comak.set_prescribed_coordinates(23,'/jointset/torso_neckhead/neck_rot');
comak.set_prescribed_coordinates(24,'/jointset/acromial_r/arm_add_r');
comak.set_prescribed_coordinates(25,'/jointset/acromial_r/arm_flex_r');
comak.set_prescribed_coordinates(26,'/jointset/acromial_r/arm_rot_r');
comak.set_prescribed_coordinates(27,'/jointset/elbow_r/elbow_flex_r');
comak.set_prescribed_coordinates(28,'/jointset/radioulnar_r/pro_sup_r');
comak.set_prescribed_coordinates(29,'/jointset/radius_hand_r/wrist_flex_r');
comak.set_prescribed_coordinates(30,'/jointset/acromial_l/arm_add_l');
comak.set_prescribed_coordinates(31,'/jointset/acromial_l/arm_flex_l');
comak.set_prescribed_coordinates(32,'/jointset/acromial_l/arm_rot_l');
comak.set_prescribed_coordinates(33,'/jointset/elbow_l/elbow_flex_l');
comak.set_prescribed_coordinates(34,'/jointset/radioulnar_l/pro_sup_l');
comak.set_prescribed_coordinates(35,'/jointset/radius_hand_l/wrist_flex_l');
 
comak.set_primary_coordinates(0,'/jointset/hip_r/hip_flex_r');
comak.set_primary_coordinates(1,'/jointset/hip_r/hip_add_r');
comak.set_primary_coordinates(2,'/jointset/hip_r/hip_rot_r');
comak.set_primary_coordinates(3,'/jointset/knee_r/knee_flex_r');
comak.set_primary_coordinates(4,'/jointset/ankle_r/ankle_flex_r');

% max_change of each secondary coordinate (rad or m): limit on the total change of
% the coordinate in one time step, summed over the COMAK iterations.
% 0 (or less) leaves the coordinate free. This is the default and gave the smoothest
% and fastest solutions in the walking, meniscus and EMG examples.
% Set a positive value to limit how far a coordinate can move in one time step.
% To choose it for a new activity, run once with 0 and set it to about twice the
% largest change per time step of the coordinate in the results.
secondary_coord_set = COMAKSecondaryCoordinateSet(); 
secondary_coord = COMAKSecondaryCoordinate();

secondary_coord.setName('knee_add_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/knee_r/knee_add_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_rot_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/knee_r/knee_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tx_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_ty_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/knee_r/knee_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tz_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_flex_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_flex_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_rot_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tilt_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tilt_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tx_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_ty_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tz_r');
secondary_coord.set_max_change(0);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

comak.set_COMAKSecondaryCoordinateSet(secondary_coord_set);

comak.set_settle_secondary_coordinates_at_start(true);
comak.set_settle_threshold(5e-4);
comak.set_settle_accuracy(1e-2);
comak.set_settle_internal_step_limit(10000);
comak.set_print_settle_sim_results(true);
comak.set_settle_sim_results_directory(comak_result_dir);
comak.set_settle_sim_results_prefix('walking_settle_sim');
comak.set_max_iterations(25);
comak.set_udot_tolerance(1);
comak.set_udot_worse_case_tolerance(50);
% Muscle force model of COMAK. All three are off by default: the muscle force is
% activation * max_isometric_force with a rigid tendon.
%  use_force_length_velocity: the maximum force follows the force-length-velocity relationship
%    and the pennation angle of the muscle (rigid tendon: it distorts long-tendon muscles such as the
%    soleus, so use_tendon_compliance is the better choice if this effect is wanted)
%  use_passive_muscle_force: adds the passive fiber force (experimental)
%  use_tendon_compliance: elastic tendon, solved as a quasi-static equilibrium; it includes the
%    force-length relationship and the passive force (experimental)
comak.set_use_force_length_velocity(false);
comak.set_use_passive_muscle_force(false);
comak.set_use_tendon_compliance(false);
% A time step has converged when the unbalanced generalized force of the
% primary and secondary coordinates is below these tolerances
% (convergence_criterion 'force', the default; udot_tolerance and
% udot_worse_case_tolerance then count multiples of them). Rough maximum from
% the walking test: up to 30 times the defaults (3 N*m and 30 N) changed the
% muscle activations by less than 0.012 and the secondary coordinates by less
% than 0.03 (deg or mm) and halved the COMAK iterations; 100 times (10 N*m
% and 100 N) changed the activations by up to 0.04 and the coordinates by up
% to 0.08. Do not go above this. Use convergence_criterion 'acceleration' for
% the previous acceleration based test.
comak.set_force_tolerance_rotation(0.1); % N*m
comak.set_force_tolerance_translation(1.0); % N
comak.set_unit_udot_epsilon(1e-6);
comak.set_optimization_scale_delta_coord(1);
comak.set_ipopt_diagnostics_level(3);
comak.set_ipopt_max_iterations(500);
comak.set_ipopt_convergence_tolerance(1e-4);
comak.set_ipopt_constraint_tolerance(1e-4);
comak.set_ipopt_limited_memory_history(200);
comak.set_ipopt_nlp_scaling_max_gradient(10000);
comak.set_ipopt_nlp_scaling_min_value(1e-8);
comak.set_ipopt_obj_scaling_factor(1);
comak.set_activation_exponent(2);
comak.set_contact_energy_weight(0);
comak.set_non_muscle_actuator_weight(1000);
comak.set_model_assembly_accuracy(1e-12);
comak.set_use_visualizer(true);
comak.set_verbose(2);

% Joint mechanics: right after COMAK the joint mechanics (contact, ligament and kinematic outputs) is run
% with the settings of the joint_mechanics section of the COMAK setup (run_joint_mechanics is true by
% default). What is not set is taken from COMAK: the model, the states, the time range and the muscle force
% model. The results are written to <results folder>/joint_mechanics/<trial name>: the vtp files and the
% ForceReporter forces .sto file. Some of the settings that can be changed (uncomment):
% comak.set_run_joint_mechanics(false);      % do not run the joint mechanics after COMAK
% jm = comak.upd_joint_mechanics();
% jm.set_results_directory('./results/joint_mechanics');   % default: <results>/joint_mechanics/<trial name>
% jm.set_contacts(0,'all');                  % contact forces and pressures of these contacts
% jm.set_ligaments(0,'all');                 % ligament forces of these ligaments
% jm.set_muscles(0,'none');                  % muscle outputs
% jm.set_write_h5_file(false);                % h5 file for the visualization
% jm.set_normalize_to_cycle(true);            % results in percent of the cycle
% jm.set_output_position_frame('/bodyset/tibia_proximal_r');     % report in the tibia frame
% jm.set_output_orientation_frame('/bodyset/tibia_proximal_r');

comak.print('./inputs/comak_settings.xml');

disp('Running COMAK Tool...')
comak.run();

%% Joint mechanics as a separate tool (optional)
% The joint mechanics is run by COMAK (see above). It can also be run on its own, with its own setup, for
% COMAK results or for the results of another simulation. To do so, set comak.set_run_joint_mechanics(false)
% above and uncomment the code below.
% import org.opensim.modeling.*
% jnt_mech = JointMechanicsTool();
% jnt_mech.set_model_file(model_file);
% jnt_mech.set_input_states_file([comak_result_dir '/' results_basename '_states.sto']);
% % the muscle force model of COMAK (force-length-velocity, passive force, tendon compliance) is read from
% % the COMAK setup, so the muscle forces are those COMAK used (activation * maximum force by default)
% jnt_mech.set_comak_setup_file('./inputs/comak_settings.xml');
% jnt_mech.set_use_muscle_physiology(false);
% jnt_mech.set_results_file_basename(results_basename);
% jnt_mech.set_results_directory(jnt_mech_result_dir);
% jnt_mech.set_start_time(1.28);
% jnt_mech.set_stop_time(1.91);
% jnt_mech.set_resample_step_size(-1);
% jnt_mech.set_normalize_to_cycle(true);
% jnt_mech.set_lowpass_filter_frequency(-1);
% jnt_mech.set_print_processed_kinematics(false);
% jnt_mech.set_contacts(0,'all');
% jnt_mech.set_contact_outputs(0,'all');
% jnt_mech.set_contact_mesh_properties(0,'none');
% jnt_mech.set_ligaments(0,'none');
% jnt_mech.set_ligament_outputs(0,'none');
% jnt_mech.set_muscles(0,'none');
% jnt_mech.set_muscle_outputs(0,'none');
% jnt_mech.set_attached_geometry_bodies(0,'none');
% jnt_mech.set_output_orientation_frame('/bodyset/tibia_proximal_r');
% jnt_mech.set_output_position_frame('/bodyset/tibia_proximal_r');
% jnt_mech.set_write_vtp_files(true);
% jnt_mech.set_vtp_file_format('binary');
% jnt_mech.set_write_h5_file(false);
% jnt_mech.set_h5_kinematics_data(true);
% jnt_mech.set_h5_states_data(true);
% jnt_mech.set_write_transforms_file(false);
% jnt_mech.set_output_transforms_file_type('sto');
% jnt_mech.set_use_visualizer(false);
% analysis_set = AnalysisSet();
% frc_reporter = ForceReporter();
% frc_reporter.setName('ForceReporter');
% analysis_set.cloneAndAppend(frc_reporter);
% jnt_mech.set_AnalysisSet(analysis_set);
% jnt_mech.print('./inputs/joint_mechanics_settings.xml');
% disp('Running JointMechanicsTool...');
% jnt_mech.run();
