#ifndef OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_
#define OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                      COMAKInverseKinematicsTool.h                          *
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

#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Tools/IKTaskSet.h>

#include "osimJAMDLL.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 This tool enables inverse kinematics to be performed for models 
 that include joints where some Coordinates (degrees of freedom) can be 
 accurately determined from motion capture (marker_determined) and others 
 cannot (secondary). A forward dynamic simulation is performed to obstrain a 
 set of constraint functions to couple the secondary coordinates to specific 
marker determined coordinates.  
 *
 * @author Colin Smith

 */
class OSIMJAM_API COMAKInverseKinematicsTool : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKInverseKinematicsTool, Object)

public:
    OpenSim_DECLARE_PROPERTY(setup_version, int,
        "Version of the settings this setup was written for. 0 means it was "
        "not declared (a setup file or script from before this property "
        "existed): when the tool is loaded from such a file, or run, the "
        "setup is upgraded to version 1 with a warning (see "
        "upgrade_old_setup). New scripts set it to 1. The current version "
        "is 1. The default value is 0.")

    OpenSim_DECLARE_PROPERTY(upgrade_old_setup, bool,
        "If true, a setup file older than the current setup_version is "
        "upgraded when it is run: the settle threshold is set to the "
        "current default, a missing plateau check is switched on, and each "
        "change is logged. Other settings "
        "are never changed. Set to false to keep the old values. The "
        "default value is true.")

    OpenSim_DECLARE_PROPERTY(model_file, std::string, 
        "Path to .osim model file.")

    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
        "Path to directory to write output results.")

    OpenSim_DECLARE_PROPERTY(results_prefix, std::string, 
        "Prefix to all results files names.")

    OpenSim_DECLARE_PROPERTY(perform_secondary_constraint_sim, bool,
        "Perform forward simulation where secondary_coupled_coordinate is "
        "prescribed and secondardy coordinates are unconstrained to "
        "generate the coupled constraint functions for the "
        "seconday coordinates for inverse kinematics."
        "The default value is true.")
 
    OpenSim_DECLARE_LIST_PROPERTY(secondary_coordinates, std::string, 
        "List of paths to the Secondary Coordinates in the model.")

    OpenSim_DECLARE_LIST_PROPERTY(constrained_coordinates, std::string, 
        "List of paths to the Constrained Coordinates in the model. Constrained "
        "coordinates (e.g., Patella Flex) are free in the settling and sweep "
        "simulations, like the secondary coordinates. A coupled constraint "
        "function of the secondary_coupled_coordinate is computed for each "
        "from the sweep results and applied in inverse kinematics.")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(secondary_coupled_coordinate, std::string, 
        "Path to the coordinate to prescribe in "
        "secondary_constraint_simulation. In inverse kinematics, "
        "secondary_coupled_coord will be used as the "
        "independent_coordinate_name for the CoordinateCouplerConstraints "
        "for all secondary coordinates. ")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_threshold, double,
        "Set the maximum change in secondary coordinates between timesteps "
        "that defines equilibrium for the settling simulation. Once the "
        "change in all COMAKSecondaryCoordinate values is smaller than the "
        "settle_threshold, the settling simulation is finished. The final "
        "secondary coordinate values are used to initialize the sweep "
        "simulation. A tight threshold matters here: the sweep builds the "
        "coupling functions of the secondary coordinates from the settled "
        "values. The default value is 1e-4.")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_plateau_tolerance,
        double,
        "Also finish the settling simulation when the largest change in the "
        "secondary coordinates per step has plateaued: it varied by less than "
        "this fraction (e.g. 0.1 for 10%) over the last "
        "secondary_constraint_sim_settle_plateau_window steps, after having "
        "dropped below 10% of its first value. 0 disables the plateau check. "
        "The default value is 0.1.")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_plateau_window,
        int,
        "Number of steps used to detect a plateau in the settling simulation. "
        "The default value is 10.")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_muscle_activation, double,
        "Constant muscle force, as a fraction of maximum isometric force, "
        "applied in the settling and sweep simulations. "
        "The default value is 0.02.")

    OpenSim_DECLARE_PROPERTY(enforce_secondary_coordinate_range, bool,
        "Keep the secondary coordinates within the range of the coordinate "
        "(range min to range max in the model file) in the inverse "
        "kinematics solution. The default value is false, which leaves "
        "the secondary coordinates unbounded.")

    OpenSim_DECLARE_PROPERTY(settling_sim_dt, double,
        "Time step between convergence checks in the settling simulation. "
        "The default value is 0.005.")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_sweep_time, double,
        "The duration of the simulation phase where the "
        "secondary_coupled_coord is swept (linearly prescribed) through the "
        "range of motion from the  secondary_coupled_coordinate_start_value "
        "to the secondary_coupled_coordinate_stop_value.")

    OpenSim_DECLARE_PROPERTY(sweep_dt, double, 
        "The time step of the simulation phase where the "
        "secondary_coupled_coord is swept (non-linearly prescribed) through the "
        "range of motion from the  secondary_coupled_coordinate_start_value "
        "to the secondary_coupled_coordinate_stop_value.")

    OpenSim_DECLARE_PROPERTY(secondary_coupled_coordinate_start_value, double,
        "Lowest value (min) of the secondary_coupled_coordinate in the "
        "secondary_constraint_sim. The units are in meters for translational "
        "coordinates and degrees for rotational coordinates. If min is below "
        "0 and the stop value is above 0, the model is settled at 0 and two "
        "sweeps start from this settled state: one down to min and one up to "
        "the stop value (max). Otherwise the model is settled at min and "
        "swept up from it. The default value is -10.0.")

    OpenSim_DECLARE_PROPERTY(secondary_coupled_coordinate_stop_value, double,
        "Highest value (max) of the secondary_coupled_coordinate in the "
        "secondary_constraint_sim. The units are in meters for translational "
        "coordinates and degrees for rotational coordinates. "
        "The default value is 110.0. The inverse kinematics warns if the "
        "coordinate leaves the range from the start to the stop value, where "
        "the constraint functions are extrapolated.")

    OpenSim_DECLARE_PROPERTY(
        settling_sim_integrator_accuracy, double, 
        "Integrator tolerance for the settling simulation. A value below 1e-9 "
        "is not used: the secondary_constraint_sim_integrator_accuracy is "
        "used instead (the default value 1e-10 therefore means the same "
        "accuracy as the sweep).")

    OpenSim_DECLARE_PROPERTY(
        secondary_constraint_sim_integrator_accuracy, double, 
        "Integrator tolerance for the forward simulation."
        "The default value is 1e-6.")

    OpenSim_DECLARE_PROPERTY(
        secondary_constraint_sim_internal_step_limit, int,
        "Limit on the number of internal steps that can be taken by BDF "
        "integrator. If -1, then there is no limit. The Default value is -1")

     OpenSim_DECLARE_PROPERTY(
         secondary_constraint_function_file, std::string, 
        "Name for .xml results file where secondary constraint functions "
        "will be saved. "
        "The default value is "
        "'secondary_coordinate_constraint_functions.xml'.")

    OpenSim_DECLARE_PROPERTY(constraint_function_num_interpolation_points, int, 
        "Number of points used in the secondary_constraint_function spline "
        "computed based on the sweep simulation results."
        "The default value is 20."
        "'secondary_coordinate_constraint_functions.xml'.")

    OpenSim_DECLARE_PROPERTY(print_secondary_constraint_sim_results, bool, 
        "Print model states to a .sto file for secondary_constraint_sim. "
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(constrained_model_file, std::string, 
        "Print the .osim model file with added CoordinateCouplerConstraints "
        "that is used in the InverseKinematicsTool. If empty, no model file "
        "will be printed.")

    OpenSim_DECLARE_PROPERTY(perform_inverse_kinematics, bool, 
        "Perform Inverse Kinematics where CoordinateCouplerConstraints are "
        "added to the model where the secondary_coordinates are coupled to "
        "the secondary_coupled_coordinate. "
        "The default value is true.")

    OpenSim_DECLARE_PROPERTY(marker_file, std::string,
        "TRC file (.trc) containing the time history of observations of marker "
        "positions obtained during a motion capture experiment. Markers in this "
        "file that have a corresponding task and model marker are included.");

    OpenSim_DECLARE_PROPERTY(coordinate_file, std::string,
        "The name of the storage (.sto or .mot) file containing the time "
        "history of coordinate observations. Coordinate values from this file are "
        "included if there is a corresponding model coordinate and task. ");

    OpenSim_DECLARE_PROPERTY(output_motion_file, std::string,
        "The name of the storage (.sto or .mot) file containing the time "
        "history of coordinate observations. Coordinate values from this file are "
        "included if there is a corresponding model coordinate and task. ");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(time_range, double, 2,
        "The desired time range over which inverse kinematics is solved. "
        "The closest start and final times from the provided observations "
        "are used to specify the actual time range to be processed.");

    OpenSim_DECLARE_PROPERTY(report_errors, bool,
        "Flag (true or false) indicating whether or not to report marker "
        "errors from the inverse kinematics solution in a .sto file.");

    OpenSim_DECLARE_PROPERTY(report_marker_locations, bool,
        "Flag (true or false) indicating whether or not to report model "
        "marker locations to a .sto file."
        "Note, model marker locations are expressed in Ground.");

    OpenSim_DECLARE_PROPERTY(ik_constraint_weight, double,
        "A positive scalar that weights the relative importance of satisfying "
        "model constraints during the inverse kinematics optimization. A "
        "weighting of 'Infinity' (the default) results in the "
        "constraints being strictly enforced. Otherwise, the weighted-squared "
        "constraint errors are appended to the cost function.");

    OpenSim_DECLARE_PROPERTY(ik_accuracy, double,
        "The accuracy of the inverse kinematics solution in absolute terms. "
        "Default is 1e-5. It determines the number of significant digits to "
        "which the solution can be trusted.");

    OpenSim_DECLARE_PROPERTY(smooth_ik_second_pass, bool,
        "If true, the inverse kinematics is solved twice. The coordinates of "
        "the first solution are smoothed (low-pass filtered, see "
        "smooth_ik_lowpass_frequency) and the second solution fits the "
        "markers with an added soft pull of each free coordinate towards its "
        "smoothed value (smooth_ik_weight). It removes the frame-to-frame "
        "noise of the first solution where the marker fit is flat. The first "
        "solution is written to the output_motion_file with _pass1 added to "
        "its name. The default value is true.")

    OpenSim_DECLARE_PROPERTY(smooth_ik_weight, double,
        "Weight of the soft coordinate references of the second pass (see "
        "smooth_ik_second_pass), relative to the weights of the marker tasks. "
        "Coordinates that have an IKCoordinateTask or that are constrained "
        "(the secondary coordinates) get no reference. The default value is "
        "1.")

    OpenSim_DECLARE_PROPERTY(smooth_ik_lowpass_frequency, double,
        "Cutoff frequency (Hz) of the low-pass filter applied to the "
        "coordinates of the first pass to make the references of the second "
        "pass. Use the same frequency as the filter of the input kinematics "
        "(lowpass_filter_frequency of the COMAKTool). -1 (no filter set) "
        "uses 40 Hz. The default value is 6.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(IKTaskSet, 
        "Markers and coordinates to be considered (tasks) and their weightings. "
        "The sum of weighted-squared task errors composes the cost function."); 

    OpenSim_DECLARE_PROPERTY(geometry_folder, std::string,
            "Optional. "
            "File path to folder containing model geometries.")

    OpenSim_DECLARE_PROPERTY(use_visualizer, bool,
        "Use SimTK visualizer to display the model during "
        "secondary_constraint_sim and inverse kinematics."
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(auto_close_visualizer, bool,
        "Close the visualizer window of the settling simulation and of each "
        "sweep simulation 10 seconds after that simulation is done (the "
        "window of the inverse kinematics is not closed). Only used if "
        "use_visualizer is true. "
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(show_solver_info_in_visualizer, bool,
        "Show information as text in the window of the sweep simulations: the "
        "sweep direction, the time, the coupled coordinate, the integrator "
        "time step and step counts, and the contact forces (medial and "
        "lateral tibia, patella). Only used if use_visualizer is true. "
        "The default value is true.")

    OpenSim_DECLARE_PROPERTY(verbose, int,
                            "Controling output detail.")


//=============================================================================
// METHODS
//=============================================================================
  
    /**
    * Default constructor.
    */
    COMAKInverseKinematicsTool();
    
    //Construct from .xml file
    COMAKInverseKinematicsTool(const std::string file);

private:
    void constructProperties();
    void upgradeOldSetup();
    bool _old_setup_notice = false;
    bool _setup_upgraded = false;
    std::vector<std::string> _upgrade_changes;

public:
    bool initialize();
    bool run();
    void setModel(Model& model);
    /** Upgrade the setup to the current version if it is older (see
    setup_version) and save it as an XML file. where is empty (save next to the
    loaded setup file, with _v1 added to its name), "inplace" (overwrite the
    loaded file), a folder ending with a slash, or a file name. Returns the
    path of the file. */
    std::string saveUpgradedSetup(const std::string& where = "");
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
            override;
    void performIKSecondaryConstraintSimulation();
    void performIK();
    void runInverseKinematics(Model& model);
    void printDebugInfo(const Model& model, const SimTK::State& state);

    void populateReferences(const Model& model, 
        MarkersReference& markersReference,
        SimTK::Array_<CoordinateReference>&coordinateReferences) const;
   
    //--------------------------------------------------------------------------
    // Members
    //--------------------------------------------------------------------------
public:
    Model _model;
    bool _model_exists;

    SimTK::State _state;

    int _n_constrained_coord;
    int _n_primary_coord;
    int _n_secondary_coord;

    int _n_muscles;
    int _n_reserve_actuators;
    int _n_actuators;
    int _n_parameters;
    Array<std::string> _parameter_names;

    Array<std::string> _constrained_coord_name;
    Array<std::string> _constrained_coord_path;
    Array<int> _constrained_coord_index;

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


    SimTK::Vector _secondary_coord_damping;
    SimTK::Vector _optimal_force;
    SimTK::Vector _prev_secondary_value;
    SimTK::Vector _prev_parameters;
    SimTK::Vector _parameter_scale;
    SimTK::Vector _muscle_volumes;

    FunctionSet _secondary_constraint_functions;
    std::string _directoryOfSetupFile;

//=============================================================================
};  // END of class COMAK_INVERSE_KINEMATICS_TOOL

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_
