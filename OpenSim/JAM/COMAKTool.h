#ifndef OPENSIM_COMAK_TOOL_H_
#define OPENSIM_COMAK_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.h                                *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimJAMDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

#include "COMAKSettingsSet.h"

namespace OpenSim { 

class COMAKCostFunctionParameter;
class COMAKCostFunctionParameterSet;

 
//=============================================================================
//                         COMAK Tool
//=============================================================================
/**
The Concurrent Optimization of Muscle Activations and Kinematics (COMAK) 
algorithm enables the calculatation of muscle forces, ligament forces, 
articular contact pressures, and secondary kinematics throughout a measured 
movement. An optimization is performed that minimizes

As inputs, COMAK requires a .osim model, measured joint kinematics (motion 
 capture etc.), and the ExternalLoads experienced by the model during the 
 activity (ground reactions etc). 

A key aspect of COMAK is the differentiation between Model Coordinates as 
Prescribed, Primary or Secondary. The Prescribed and Primary Coordinates must
be measured througout the activity, while the secondary coordinates are 
predicted by the simulation. Within the COMAK optimization, the accelerations 
of the Prescribed coordinates are achieved through multibody dynamic constraints
(ie not generated by the forces in the model), whereas the the accelerations
of the Primary coordinates must be generated by the model forces 
(muscles, ligaments, contact, external, inertial and centripetal). 

The COMAKInverseKinematicsTool facilitates the calculation of the Presecribed
and Primary kinematics from motion capture data. 

A forward simulation is commonly performed to initialize the Secondary Coordinate
values at the first time step of COMAK (settle_secondary_coordinates_at_start).
Here, the model Coordinates are first set to the values at the corresponding time 
step in the coordinates_file. The Prescribed and Primary Coordinates are then 
locked, while the Secondary Coordinates are left unconstrained. 
No external loads are applied to the model, and the muscle forces are set to 
0.02 * maxIsometricForce to represent their passive contributions. Thus, in the 
forward dynamic simulation, the Secondary Coordinate are allowed to settle into 
equilibrium based on the passive muscle, ligament, and articular contact forces. 
This settling simulation is terminated when the largest change in the Secondary
Coordinates between time steps is less than the settle_threshold. 

### References
[1] Smith, C. R., Vignos, M. F., Lenhart, R. L., Kaiser, J., & Thelen, D. G.
    (2016). The influence of component alignment and ligament properties on 
    tibiofemoral contact forces in total knee replacement. 
    Journal of biomechanical engineering, 138(2), 021017.

[2] 

[3]


*/

class OSIMJAM_API COMAKTool : public Object{
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKTool, Object)

public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string, 
        "Path to .osim model to use in COMAK simulation.")

    OpenSim_DECLARE_PROPERTY(coordinates_file, std::string, 
        "Path to input .sto file containing joint angles vs time for all "
        "prescribed, primary, and secondary coordinates.")

    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "Path to .xml "
        "file that defines the ExternalLoads applied to the model.")

    OpenSim_DECLARE_PROPERTY(results_directory, std::string, 
        "Path to folder where all results files will be written.")

    OpenSim_DECLARE_PROPERTY(results_prefix, std::string, 
        "Prefix to all results files names.")

    OpenSim_DECLARE_PROPERTY(replace_force_set, bool, 
        "Replace the model ForceSet with the forces listed in force_set_file. "
        "If false, force_set_file forces are appended to the existing model "
        "force set. The default value is false.")

    OpenSim_DECLARE_PROPERTY(force_set_file,std::string,
        "Path to .xml file containing an additional ForceSet.")

    OpenSim_DECLARE_PROPERTY(start_time, double, 
        "First time step of COMAK simulation.")

    OpenSim_DECLARE_PROPERTY(stop_time, double, 
        "Last time step of COMAK simulation.")

    OpenSim_DECLARE_PROPERTY(time_step, double,
        "Time increment between steps in COMAK simulation. Set to -1 to use "
        "the time step in the input coordinates_file. "
        "The default value is -1.")

    OpenSim_DECLARE_PROPERTY(lowpass_filter_frequency, double, 
        "Lowpass filter frequency for input kinematics. "
        "If set to -1, no filtering is applied. The default value is -1.")

    OpenSim_DECLARE_PROPERTY(print_processed_input_kinematics, bool, 
        "Print the processed input Coordinate values, speeds, and "
        "accelerations to a .sto file. These kinematics are used directly "
        "within the COMAK optimization and can be helpful for debugging.")

    OpenSim_DECLARE_LIST_PROPERTY(prescribed_coordinates, std::string,
        "List the paths to the Prescribed Coordinates in the model.")

    OpenSim_DECLARE_LIST_PROPERTY(primary_coordinates, std::string,
        "List the paths to the Primary Coordinates in the model.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(COMAKSecondaryCoordinateSet,
        "List of COMAKSecondaryCoodinate objects.")

    OpenSim_DECLARE_PROPERTY(settle_secondary_coordinates_at_start, bool,
        "Perform a forward simulation to settle secondary coordinates into "
        "equilbrium at initial time step of COMAK. The default value is true.")

    OpenSim_DECLARE_PROPERTY(settle_threshold, double, 
        "Set the maximum change in secondary coordinates between timesteps "
        "that defines equilibrium. Once the change in all "
        "COMAKSecondaryCoordinate values is smaller than the "
        "settle_threshold, the settling simulation is stopped. "
        "The default value is 1e-5.")

    OpenSim_DECLARE_PROPERTY(settle_accuracy, double, 
        "Set the integrator accuracy for initializing forward simulation. "
        "The default value is 1e-6.")

    OpenSim_DECLARE_PROPERTY(settle_internal_step_limit, int, 
        "Limit on the number of internal steps that can be taken by BDF " 
        "integrator initializing forward simulation. If -1, then there is no "
        "limit. The Default value is -1.")

    OpenSim_DECLARE_PROPERTY(print_settle_sim_results, bool, 
        "Print the model states during the forward simulation to a .sto file "
        "in the settle_sim_results_dir.")

    OpenSim_DECLARE_PROPERTY(settle_sim_results_directory, std::string, 
        "Path to the directory where the settling forward simulation results "
        "will be printed.")

    OpenSim_DECLARE_PROPERTY(settle_sim_results_prefix, std::string, 
        "Prefix to settle simulation results file names.")

    OpenSim_DECLARE_PROPERTY(settle_constant_muscle_control, double,
        "Set constant control value for all muscles in settle simulation. "
        "Note muscle physiology is ignored so F_muscle = F_max_iso * control. "
        "Value must be within the bounds [0 - 1]"
        "The default value is 0.02. ")

    OpenSim_DECLARE_PROPERTY(max_iterations, int, 
        "Maximum number of COMAK iterations per time step allowed for the "
        "the simulated model accelerations to converge to the input observed "
        "acceleration values. The default value is 25.")

    OpenSim_DECLARE_PROPERTY(udot_tolerance, double, 
        "Acceptable difference between the simulated accelerations (udots) "
        "and input observed accelerations to determine if the COMAK solution "
        "at each time step has converged. The default value is 1.0.")

    OpenSim_DECLARE_PROPERTY(udot_worse_case_tolerance, double, 
        "Maximum acceptable difference between simulated and observed "
        "accelerations (udots) that is still used if no COMAK iterations "
        "converge. If the max difference between the udots for all iterations "
        "is greater than udot_worse_case_tolerance then no acceptable COMAK "
        "solution was found, and the solution from previous time step is used "
        "for the current time step. The default value is 50.0.")

    OpenSim_DECLARE_PROPERTY(unit_udot_epsilon, double, 
        "The size of the perturbation applied to COMAKSecondaryCoordinates "
        "when computing the gradient of the acceleration constraints in the "
        "COMAK optimization to changes in the secondary coordinate values. "
        "The default value is 1e-8.")

    OpenSim_DECLARE_PROPERTY(optimization_scale_delta_coord, double,
        "Scale factor applied to the delta secondardy coordinate "
        "optimization variable (parameter). This is used to reduce the "
        "gradients of the acceleration contrainsts such that the optimization "
        "problem is equally sensitive to muscle activation changes and "
        "changes in secondary coordinates."
        "The default value is 1e-3.")

    OpenSim_DECLARE_PROPERTY(ipopt_diagnostics_level, int, 
        "Set the verbosity of the IPOPT optimizer must be within 0-12. "
        "The default value is 0.")

    OpenSim_DECLARE_PROPERTY(ipopt_max_iterations, int, 
        "Set the maximum internal iterations for the IPOPT optimizer. "
        "The default value is 500.")

    OpenSim_DECLARE_PROPERTY(ipopt_convergence_tolerance, double, 
        "Set the convergence tolerance for IPOPT optimizer. The algorithm "
        "terminates successfully, if the (scaled) NLP error becomes smaller "
        "than this value. The default value is 1e-4.")

    OpenSim_DECLARE_PROPERTY(ipopt_constraint_tolerance, double, 
        "Set the constraint tolerance for the IPOPT optimizer. "
        "Absolute tolerance on the constraint violation. Successful "
        "termination requires that the max-norm of the (unscaled) constraint "
        "violation is less than this threshold. The value must be > 0, and "
        "the default value is 1e-4")

    OpenSim_DECLARE_PROPERTY(ipopt_limited_memory_history, int, 
        "Maximum size of the history for the limited quasi-Newton Hessian "
        "approximation in the IPOPT optimizer."
        "The default value is 6.")

    OpenSim_DECLARE_PROPERTY(ipopt_nlp_scaling_max_gradient, double,
        "Maximum gradient after NLP scaling. This is the gradient scaling "
        "cut-off. If the maximum gradient is above this value, then gradient "
        "based scaling will be performed. Scaling parameters are calculated "
        "to scale the maximum gradient back to this value. (This is g_max in "
        "Section 3.8 of the implementation paper.) "
        "The valid range for this real option is 0 < nlp_scaling_max_gradient "
        "and its default value is 1000.")

    OpenSim_DECLARE_PROPERTY(ipopt_nlp_scaling_min_value, double,
        "Minimum value of gradient-based scaling values. This is the lower "
        "bound for the scaling factors computed by gradient-based scaling "
        "method. If some derivatives of some functions are huge, the scaling "
        "factors will otherwise become very small, and the (unscaled) final "
        "constraint violation, for example, might then be significant. "
        "The valid range for this real option is 0 <= nlp_scaling_min_value "
        "and its default value is 1e-9.")

    OpenSim_DECLARE_PROPERTY(ipopt_obj_scaling_factor, double,
        "Scaling factor for the objective function. This option sets a "
        "scaling factor for the objective function. The scaling is seen "
        "internally by Ipopt but the unscaled objective is reported in the "
        "console output. If additional scaling parameters are computed "
        "(e.g. user-scaling or gradient-based), both factors are multiplied. "
        "If this value is chosen to be negative, Ipopt will maximize the "
        "objective function instead of minimizing it. The valid range for "
        "this real option is unrestricted and its default value is 1.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(COMAKCostFunctionParameterSet,
        "List of COMAKCostFunctionWeight objects.")

    OpenSim_DECLARE_PROPERTY(activation_exponent, double, 
        "In the COMAK cost function the muscle and actuator (reserves etc) "
        "activations are raised to this power. The default value is 2.")

    OpenSim_DECLARE_PROPERTY(contact_energy_weight, double, 
        "The weighting on Smith2018ArticularContactForce potential energy "
        "term in COMAK cost function. The default value is 0.")

    OpenSim_DECLARE_PROPERTY(non_muscle_actuator_weight, double,
        "The weighting on the COMAK cost function term that sums the "
        "activations of all actuators that are not muscles (reserves etc). "
        "The default value is 1000.")

    OpenSim_DECLARE_PROPERTY(model_assembly_accuracy, double, 
        "Model assembly accuracy used during COMAK optimization. This will "
        "override the settings in the .osim file. Try making this value "
        "smaller if there are constraints in the model and the COMAK "
        "iterations are not converging."
        "The default value is 1e-10.")

    OpenSim_DECLARE_PROPERTY(use_muscle_physiology, bool,
        "PLACEHOLDER PROPERTY."
        "Set whether activation dynamics, muscle force-length-velocity "
        "and pennation properties should be used. "
        "If false, use_activation_dynamics and use_tendon_compliance "
        "are ignored and muscle-tendon force is calculated using: "
        "Force = activation * max_isometric_force(). "            
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(geometry_folder, std::string,
        "Optional. "
        "File path to folder containing model geometries.")

    OpenSim_DECLARE_PROPERTY(use_visualizer, bool, 
        "Use SimTK visualizer to display simulations in progress. "
        "The default value is false.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(AnalysisSet,"Analyses to be performed"
        "throughout the COMAK simulation.")
    // Amir
    OpenSim_DECLARE_PROPERTY(emg_file, std::string, 
        "Path to enveloped EMG file.")
   OpenSim_DECLARE_PROPERTY(is_emg_assisted,
                                     bool,
                                     "If true, the analysis will be EMG-assisted.")
    OpenSim_DECLARE_PROPERTY(verbose, int,
                            "Controling output detail. 0: print essential info, 1: more details, 2: print all details (huge log file)")
//=============================================================================
// METHODS
//=============================================================================
  
    /**
    * Default constructor.
    */
    COMAKTool();
    
    //Construct from .xml file
    COMAKTool(const std::string file);

private:
    void constructProperties();
    void updateModelForces();
    SimTK::State initialize();
    void extractKinematicsFromFile();
    void applyExternalLoads();
    void printCOMAKascii();
    SimTK::Vector equilibriateSecondaryCoordinates();
    void performCOMAK();
    void setStateFromComakParameters(
        SimTK::State& state, const SimTK::Vector& parameters);
    void computeMuscleVolumes();
    void printOptimizationResultsToConsole(
        const SimTK::Vector& parameters,const SimTK::State& state);
    void initializeResultsStorage();
    void recordResultsStorage(const SimTK::State& state, int frame);
    void printResultsFiles();
    void updatemusclemaxforce(SimTK::State& s); //Amir

public:
    bool run();
    void setModel(Model& model);


//-----------------------------------------------------------------------------
// Members
//-----------------------------------------------------------------------------
private:
    Model _model;
    bool _model_exists;

    int _n_prescribed_coord;
    int _n_primary_coord;
    int _n_secondary_coord;

    int _n_muscles;
    Array<std::string> _muscle_path;

    int _n_non_muscle_actuators;
    Array<std::string> _non_muscle_actuator_path;

    int _n_actuators;
    int _n_parameters;
    SimTK::Vector _optim_parameters;
    Array<std::string> _optim_parameter_names;
    int _n_optim_constraints;
    Array<std::string> _optim_constraint_names;

    Array<std::string> _prescribed_coord_name;
    Array<std::string> _prescribed_coord_path;
    Array<int> _prescribed_coord_index;

    Array<std::string> _primary_coord_name;
    Array<std::string> _primary_coord_path;
    Array<int> _primary_coord_index;

    Array<std::string> _secondary_coord_name;
    Array<std::string> _secondary_coord_path;
    Array<int> _secondary_coord_index;

    int _n_frames;
    int _n_out_frames;
    int _start_frame;
    Array<double> _time;
    double _dt;
    int _consecutive_bad_frame;
    std::vector<int> _bad_frames;
    std::vector<double> _bad_times;
    std::vector<double> _bad_udot_errors;
    std::vector<std::string> _bad_udot_coord;

    SimTK::Matrix _q_matrix;
    SimTK::Matrix _u_matrix;
    SimTK::Matrix _udot_matrix;
    ExternalLoads _external_loads;

    //SimTK::Vector _secondary_coord_damping;
    SimTK::Vector _secondary_coord_max_change;
    Array<std::string> _secondary_damping_actuator_path;

    SimTK::Vector _optimal_force;
    SimTK::Vector _prev_secondary_value;
    SimTK::Vector _prev_parameters;
    SimTK::State _prev_state;
    SimTK::Vector _parameter_scale;
    SimTK::Vector _muscle_volumes;
    SimTK::Vector _normalized_muscle_volumes;
    
    FunctionSet _cost_muscle_weights;
    FunctionSet _cost_muscle_desired_act;
    FunctionSet _cost_muscle_act_lower_bound;
    FunctionSet _cost_muscle_act_upper_bound;
    SimTK::Vector _emg_gamma_weight; //Amir

    std::string _directoryOfSetupFile;

    StatesTrajectory _result_states;
    TimeSeriesTable _result_activations;
    TimeSeriesTable _result_forces;
    TimeSeriesTable _result_kinematics;
    TimeSeriesTable _result_values;
    TimeSeriesTable _convergence;
//=============================================================================
};  // END of class COMAK_TOOL


}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_COMAK_TOOL_H_