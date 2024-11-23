%% Setup Environment and Folders
clear;
clc;
import org.opensim.modeling.*
Logger.setLevelString('Debug');

model_file = '../models/GCD/lenhart2015/P1_lenhart2015.osim';
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

%% Perform COMAK Simulation
comak = COMAKTool();
comak.set_model_file(model_file);
comak.set_coordinates_file('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_ik.mot');
comak.set_external_loads_file('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_grf_Setup.xml'),
comak.set_results_directory(comak_result_dir);
comak.set_results_prefix(results_basename);
comak.set_replace_force_set(false);
comak.set_force_set_file('../models/GCD/lenhart2015/lenhart2015_reserve_actuators.xml');
comak.set_start_time(0.0);
comak.set_stop_time(0.8);
comak.set_time_step(0.01);
comak.set_lowpass_filter_frequency(6);
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

secondary_coord_set = COMAKSecondaryCoordinateSet(); 
secondary_coord = COMAKSecondaryCoordinate();

secondary_coord.setName('knee_add_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/knee_r/knee_add_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/knee_r/knee_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tx_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_ty_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tz_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_flex_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_flex_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tilt_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tilt_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tx_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_ty_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tz_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

comak.set_COMAKSecondaryCoordinateSet(secondary_coord_set);

comak.set_settle_secondary_coordinates_at_start(true);
comak.set_settle_threshold(1e-3);
comak.set_settle_accuracy(1e-2);
comak.set_settle_internal_step_limit(10000);
comak.set_print_settle_sim_results(true);
comak.set_settle_sim_results_directory(comak_result_dir);
comak.set_settle_sim_results_prefix('walking_settle_sim');
comak.set_max_iterations(25);
comak.set_udot_tolerance(1);
comak.set_udot_worse_case_tolerance(50);
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
comak.set_contact_energy_weight(1);
comak.set_non_muscle_actuator_weight(1000);
comak.set_model_assembly_accuracy(1e-12);
comak.set_use_visualizer(true);
comak.set_verbose(0);

%%%%%%% Here we set EMGs
comak.set_use_muscle_physiology(true);
% comak.set_emg_file('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_emg.mot'); % this won't be used currently.
comak.set_is_emg_assisted(true);

% write EMGs to setup file
emg_data = load('../models/GCD/experimental_data/motion_analysis/DM_bouncy1_1_emg.mat'); % we read EMGs from here
% define muscle mappings and gamma weight (to follow EMGs)
% {label in EMG data, muscle in the model, gamma}
muscle_mapper = {'semimem','semimem_r',1;...
    'bifem','bfsh_r',1;...
    'bifem','bflh_r',1;...
    'vasmed','vasmed_r',1;...
    'vaslat','vaslat_r',1;...
    'rf','recfem_r',1;...
    'medgas','gasmed_r',1;...
    'latgas','gaslat_r',1;...
    'tfl','tfl_r',1;...
    'tibant','tibant_r',1;...
    'peronl','perlong_r',1;...
    'soleus','soleus_r',1;...
    'addmagnus','addmagProx_r',1;...
    'addmagnus','addmagMid_r',1;...
    'addmagnus','addmagDist_r',1;...
    'addmagnus','addmagIsch_r',1;...
    'gmax','glmax1_r',1;...
    'gmax','glmax2_r',1;...
    'gmax','glmax3_r',1;...
    'gmed','glmed1_r',1;...
    'gmed','glmed2_r',1;...
    'gmed','glmed3_r',1;...
    'sartorius','sart_r',1};
cost_fun_param_set = COMAKCostFunctionParameterSet();
cost_fun_param = COMAKCostFunctionParameter();
for ii=1:size(muscle_mapper,1)
    if muscle_mapper{ii,3}
        cost_fun_param.setName(muscle_mapper{ii,2});
        cost_fun_param.set_actuator(['/forceset/' muscle_mapper{ii,2}]);
        emg_msl_weight = muscle_mapper{ii,3};
        cost_fun_param.set_emg_gamma_weight(emg_msl_weight);
        emg_spline = SimmSpline();
        emg_indx = find(contains(emg_data.lbls_emg,muscle_mapper{ii,1}));
        for jj=1:(size(emg_data.data_emg,1))
            emg_spline.addPoint(emg_data.data_emg(jj,1),emg_data.data_emg(jj,emg_indx));
        end
        cost_fun_param.set_desired_activation(emg_spline);
        cost_fun_param.set_activation_lower_bound(Constant(0.01));
        cost_fun_param.set_activation_upper_bound(Constant(1.0));
        cost_fun_param_set.cloneAndAppend(cost_fun_param);
    end
end
comak.set_COMAKCostFunctionParameterSet(cost_fun_param_set);
%%%%%%% end of EMG block
comak.print('./inputs/comak_settings.xml');

disp('Running COMAK Tool...')
comak.run();

%% Perform Joint Mechanics Analysis
import org.opensim.modeling.*
jnt_mech = JointMechanicsTool();
jnt_mech.set_model_file(model_file);
jnt_mech.set_input_states_file([comak_result_dir '/' results_basename '_states.sto']);
jnt_mech.set_use_muscle_physiology(false);
jnt_mech.set_results_file_basename(results_basename);
jnt_mech.set_results_directory(jnt_mech_result_dir);
jnt_mech.set_start_time(0);
jnt_mech.set_stop_time(0.8);
jnt_mech.set_resample_step_size(-1);
jnt_mech.set_normalize_to_cycle(true);
jnt_mech.set_lowpass_filter_frequency(-1);
jnt_mech.set_print_processed_kinematics(false);
jnt_mech.set_contacts(0,'all');
jnt_mech.set_contact_outputs(0,'all');
jnt_mech.set_contact_mesh_properties(0,'none');
jnt_mech.set_ligaments(0,'none');
jnt_mech.set_ligament_outputs(0,'none');
jnt_mech.set_muscles(0,'none');
jnt_mech.set_muscle_outputs(0,'none');

jnt_mech.set_attached_geometry_bodies(0,'none');

jnt_mech.set_output_orientation_frame('/bodyset/tibia_proximal_r');
jnt_mech.set_output_position_frame('/bodyset/tibia_proximal_r');
jnt_mech.set_write_vtp_files(true);
jnt_mech.set_vtp_file_format('binary');
jnt_mech.set_write_h5_file(false);
jnt_mech.set_h5_kinematics_data(true);
jnt_mech.set_h5_states_data(true);
jnt_mech.set_write_transforms_file(false);
jnt_mech.set_output_transforms_file_type('sto');
jnt_mech.set_use_visualizer(false);

analysis_set = AnalysisSet();

frc_reporter = ForceReporter();
frc_reporter.setName('ForceReporter');

analysis_set.cloneAndAppend(frc_reporter);
jnt_mech.set_AnalysisSet(analysis_set);
jnt_mech.print('./inputs/joint_mechanics_settings.xml');

disp('Running JointMechanicsTool...');
jnt_mech.run();
