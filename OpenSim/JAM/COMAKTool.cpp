/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.cpp                              *
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

#include "COMAKTool.h"

#include "COMAKTarget.h"
#include "COMAKQPSolver.h"
#include <chrono>
#include <cstring>
#include <memory>
#include "JAMUtilities.h"
#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Common/Constant.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h"

#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/XMLDocument.h>

using namespace OpenSim;
using namespace SimTK;

COMAKTool::COMAKTool() {
    constructProperties();
    //_directoryOfSetupFile = "";
    _model_exists = false;
}

COMAKTool::COMAKTool(const std::string file) : Object(file) {
    constructProperties();
    updateFromXMLDocument();
    _model_exists = false;
    //_directoryOfSetupFile = IO::getParentDirectory(file);
    // IO::chDir(_directoryOfSetupFile);
}

void COMAKTool::constructProperties() {
    constructProperty_model_file("");
    constructProperty_coordinates_file("");
    constructProperty_external_loads_file("");
    constructProperty_results_directory("");
    constructProperty_results_prefix("");

    constructProperty_replace_force_set(false);
    constructProperty_force_set_file("");

    constructProperty_start_time(-1);
    constructProperty_stop_time(-1);
    constructProperty_time_step(-1);
    constructProperty_lowpass_filter_frequency(-1);
    constructProperty_smooth_input_kinematics(true);
    constructProperty_smoothing_spline_error_variance(-1.0);
    constructProperty_print_processed_input_kinematics(false);

    constructProperty_prescribed_coordinates();
    constructProperty_primary_coordinates();
    constructProperty_COMAKSecondaryCoordinateSet(
            COMAKSecondaryCoordinateSet());

    constructProperty_settle_secondary_coordinates_at_start(true);
    constructProperty_settle_threshold(1e-5);
    constructProperty_settle_accuracy(1e-6);
    constructProperty_settle_internal_step_limit(-1);
    constructProperty_settle_plateau_tolerance(0.0);
    constructProperty_settle_plateau_window(10);
    constructProperty_settle_zero_initial_speeds(false);
    constructProperty_print_settle_sim_results(false);
    constructProperty_settle_sim_results_directory("");
    constructProperty_settle_sim_results_prefix("");
    constructProperty_settle_constant_muscle_control(0.02);

    constructProperty_max_iterations(50);
    constructProperty_udot_tolerance(1.0);
    constructProperty_udot_worse_case_tolerance(50.0);
    constructProperty_convergence_criterion("force");
    constructProperty_force_tolerance_rotation(0.1);
    constructProperty_force_tolerance_translation(1.0);
    constructProperty_unit_udot_epsilon(1e-8);
    constructProperty_use_central_difference_jacobian(false);
    constructProperty_ignore_secondary_speed_term(false);
    constructProperty_normalize_constraint_rows(false);
    constructProperty_secondary_step_penalty_weight(0.0);
    constructProperty_step_penalty_rotation_length(0.05);
    constructProperty_secondary_velocity_carryover_to_next_step(0.0);
    constructProperty_jacobian_recompute_interval(1);
    constructProperty_activation_change_penalty_weight(0.0);
    constructProperty_secondary_smoothness_penalty_weight(0.0);
    constructProperty_enforce_secondary_coordinate_range(false);
    constructProperty_optimization_scale_delta_coord(1e-3);

    constructProperty_solver("ipopt");
    constructProperty_qp_slack_penalty(1e4);
    constructProperty_ipopt_diagnostics_level(0);
    constructProperty_ipopt_max_iterations(500);
    constructProperty_ipopt_convergence_tolerance(1e-4);
    constructProperty_ipopt_constraint_tolerance(1e-4);
    constructProperty_ipopt_limited_memory_history(6);
    constructProperty_ipopt_nlp_scaling_max_gradient(1000);
    constructProperty_ipopt_nlp_scaling_min_value(1e-9);
    constructProperty_ipopt_obj_scaling_factor(1);

    constructProperty_activation_exponent(2);
    constructProperty_contact_energy_weight(0.0);
    constructProperty_non_muscle_actuator_weight(1000);
    constructProperty_COMAKCostFunctionParameterSet(
            COMAKCostFunctionParameterSet());

    constructProperty_use_muscle_physiology(false);

    constructProperty_model_assembly_accuracy(1e-12);
    constructProperty_geometry_folder("");
    constructProperty_use_visualizer(false);

    constructProperty_AnalysisSet(AnalysisSet());
    // Amir
    constructProperty_verbose(0);
    constructProperty_emg_file("");
    constructProperty_is_emg_assisted(false);
    constructProperty_ignore_tendon_compliance(true);
    constructProperty_ignore_activation_dynamics(true);
}

// The Smith2018ArticularContactForce outputs of the target meshes ("flip_meshes")
// are only needed by analyses; computing them doubles the contact work.
static void disableFlippedContactMeshes(Model& model, SimTK::State& state) {
    for (auto& contact :
            model.updComponentList<Smith2018ArticularContactForce>()) {
        contact.setModelingOption(state, "flip_meshes", 0);
    }
}

void COMAKTool::setModel(Model& model) {
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }

    _model = model;
    set_model_file(model.getDocumentFileName());
    _model_exists = true;
}

bool COMAKTool::run() {
    bool completed = false;

    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {
        // Amir
        //  if (get_use_muscle_physiology()) {
        //      OPENSIM_THROW(Exception,
        //          "ERROR: COMAKTool use_muscle_physiology = true "
        //          "is not implemented")
        //  };

        const Stopwatch stopwatch;
        log_critical("");
        log_critical("==========================");
        log_critical("COMAKTool");
        log_critical("==========================");
        log_critical("");

        printCOMAKascii();

        performCOMAK();

        const long long elapsed = stopwatch.getElapsedTimeInNs();

        log_info("COMAK complete.");
        log_info("Finished in {}", stopwatch.formatNs(elapsed));
        log_info("Printed results to: {}", get_results_directory());
        log_info("");

        completed = true;
    }

    catch (const std::exception& x) {
        log_error("COMAKTool::run() caught an exception: \n {}", x.what());
        cwd.restore();
    } catch (...) { // e.g. may get InterruptedException
        log_error("COMAKTool::run() caught an exception.");
        cwd.restore();
    }

    cwd.restore();

    return completed;
}

SimTK::State COMAKTool::initialize() {
    // Make results directory
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " + get_results_directory() +
                                         "Possible reason: This tool cannot "
                                         "make new folder with "
                                         "subfolder.");
    }

    // Set Model
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }

    if (!_model_exists) {
        if (get_model_file().empty()) {
            OPENSIM_THROW(Exception, "No model was set in the COMAKTool.");
        }
        _model = Model(get_model_file());
    }

    updateModelForces();

    SimTK::State state = _model.initSystem();

    // Verfiy Coordinate Properties
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();
        bool isSecondary = false;

        // Reset coordinates listed in settings file to full path
        int ind = getProperty_prescribed_coordinates().findIndex(name);
        if (ind > -1) { set_prescribed_coordinates(ind, path); }

        ind = getProperty_primary_coordinates().findIndex(name);
        if (ind > -1) { set_primary_coordinates(ind, path); }

        for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {
            COMAKSecondaryCoordinate secondary_coord =
                    get_COMAKSecondaryCoordinateSet().get(i);
            if (secondary_coord.get_coordinate() == name ||
                    secondary_coord.get_coordinate() == path) {
                secondary_coord.set_coordinate(path);
                isSecondary = true;
            }
        }

        bool isPrescribed =
                getProperty_prescribed_coordinates().findIndex(path) > -1;
        bool isPrimary = getProperty_primary_coordinates().findIndex(path) > -1;

        // Set all unlisted coordinates to prescribed
        if (!isPrescribed && !isPrimary && !isSecondary) {
            append_prescribed_coordinates(path);

            log_warn("WARNING: Coordinate ({}"
                     ") was not listed in COMAKTool params file. "
                     "Assumed PRESCRIBED.",
                    name);
        }

        // Make sure coordinate is not listed multiple times
        int counted = 0;
        if (isPrescribed) counted++;
        if (isPrimary) counted++;
        if (isSecondary) counted++;

        if (counted > 1) {
            OPENSIM_THROW(
                    Exception, "Coordinate: " + name +
                                       " was listed as multiple COMAKTool "
                                       "coordinate types "
                                       "(Prescribed, Primary, Secondary).")
        }
    }

    // Make sure Coordinates exist in model and no duplicates
    for (int i = 0; i < getProperty_prescribed_coordinates().size(); ++i) {
        std::string name = get_prescribed_coordinates(i);

        try { _model.getComponent<Coordinate>(name);}
        catch (Exception const&) {
            OPENSIM_THROW(Exception, "prescribed_coordinate: " 
                + name + " not found in model.")

        }

        int n = 0;
        for (int j = 0; j < getProperty_prescribed_coordinates().size(); ++j) {
            if (name == get_prescribed_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n > 1, Exception,
                name + "listed multiple times in prescribed_coordinates")
    }

    for (int i = 0; i < getProperty_primary_coordinates().size(); ++i) {
        std::string name = get_primary_coordinates(i);

        try { _model.getComponent<Coordinate>(name); }
        catch (Exception const&) {
            OPENSIM_THROW(Exception, "primary_coordinate: " 
                + name + "not found in model.")

        }

        int n = 0;
        for (int j = 0; j < getProperty_primary_coordinates().size(); ++j) {
            if (name == get_primary_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n > 1, Exception,
                name + "listed multiple times in primary_coordinates")
    }

    for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {

        std::string path = 
            get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();
        try { _model.getComponent<Coordinate>(path); }
        catch (Exception const&){
            OPENSIM_THROW(Exception,"secondary_coordinate: " + path +
                "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {
            if (path ==
                    get_COMAKSecondaryCoordinateSet().get(j).get_coordinate())
                n++;
        }
        OPENSIM_THROW_IF(n > 1, Exception,
                path + "listed multiple times in secondary_coordinates")
    }

    // Count number of each coordinate type
    _n_prescribed_coord = getProperty_prescribed_coordinates().size();
    _n_primary_coord = getProperty_primary_coordinates().size();
    _n_secondary_coord = get_COMAKSecondaryCoordinateSet().getSize();

    _prescribed_coord_name.setSize(_n_prescribed_coord);
    _prescribed_coord_path.setSize(_n_prescribed_coord);
    _prescribed_coord_index.setSize(_n_prescribed_coord);

    _primary_coord_name.setSize(_n_primary_coord);
    _primary_coord_path.setSize(_n_primary_coord);
    _primary_coord_index.setSize(_n_primary_coord);

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);

    // Organize coordinates in nice stuctures
    for (int i = 0; i < _n_prescribed_coord; ++i) {
        _prescribed_coord_path[i] = get_prescribed_coordinates(i);
        _prescribed_coord_name[i] =
                _model.getComponent<Coordinate>(get_prescribed_coordinates(i))
                        .getName();
    }

    for (int i = 0; i < _n_primary_coord; ++i) {
        _primary_coord_path[i] = get_primary_coordinates(i);
        _primary_coord_name[i] =
                _model.getComponent<Coordinate>(get_primary_coordinates(i))
                        .getName();
    }

    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] =
                get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();
        _secondary_coord_name[i] =
                _model.getComponent<Coordinate>(_secondary_coord_path[i])
                        .getName();
    }

    // Find the index of each coordinate in the updComponentList<Coordinate>
    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(path);
        if (ind > -1) { _prescribed_coord_index[ind] = nCoord; }

        ind = _primary_coord_path.findIndex(path);
        if (ind > -1) { _primary_coord_index[ind] = nCoord; }

        ind = _secondary_coord_path.findIndex(path);
        if (ind > -1) { _secondary_coord_index[ind] = nCoord; }
        nCoord++;
    }

    log_info("Prescribed Coordinates:");
    log_info("-----------------------");
    for (int i = 0; i < _n_prescribed_coord; ++i) {
        log_info(_prescribed_coord_name[i]);
    }

    log_info("Primary Coordinates:");
    log_info("--------------------");
    for (int i = 0; i < _n_primary_coord; ++i) {
        log_info(_primary_coord_name[i]);
    }

    log_info("Secondary Coordinates:");
    log_info("----------------------");
    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info(_secondary_coord_name[i]);
    }

    // Organize COMAKSecondaryCoordinate Properties
    _secondary_coord_max_change.resize(_n_secondary_coord);
    _secondary_coord_max_change = 0;

    for (int i = 0; i < _n_secondary_coord; ++i) {
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {

            COMAKSecondaryCoordinate sec_coord =
                    get_COMAKSecondaryCoordinateSet().get(j);

            if (sec_coord.get_coordinate() == _secondary_coord_path[i]) {
                _secondary_coord_max_change[i] = sec_coord.get_max_change();
            }
        }
    }

    // Check for Coordinate Actuators on Prescribed Coordinates
    bool first_replaced = true;
    for (auto& coord_act : _model.updComponentList<CoordinateActuator>()) {

        std::string coord_path =
                coord_act.getCoordinate()->getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(coord_path);
        if (ind > -1) {
            if (first_replaced) {
                first_replaced = false;

                log_warn(
                        "The following CoordinateActuators are "
                        " acting on a prescribed coordinate. These actuators "
                        "will "
                        "be disabled because they will do nothing while adding "
                        "extra variables to the optimization.");
            }
            log_warn("{} ({}).", coord_act.getAbsolutePathString(),
                    coord_act.get_coordinate());

            coord_act.setAppliesForce(state, false);
        }
    }

    // Check for Coordinate Coupler constrainsts on prescribed coords
    for (CoordinateCouplerConstraint& cc_const :
            _model.updComponentList<CoordinateCouplerConstraint>()) {

        std::string cc_coord_name = cc_const.getDependentCoordinateName();

        std::string coord_path = _model.updCoordinateSet()
                                         .get(cc_coord_name)
                                         .getAbsolutePathString();

        int ind = _prescribed_coord_path.findIndex(coord_path);
        if (ind > -1) { cc_const.set_isEnforced(false); }
        log_info("CoordinateCouplerConstraint: {}"
                 " will not be enforced because the dependent coordinate: {}"
                 " is prescribed.",
                cc_const.getName(), cc_coord_name);
    }

    // TO DO
    // Check for damping on the secondary coordinates
    //
    // HOW? perturbing the speeds and checking for acc change will demonstrate
    // damping is present, except that changing speed
    // has a direct effect on acc ...
    // maybe check nominal acc vs acc w/disable all forces?
    /*
    std::cout << "\n\nDAMPING CHECK "<< std::endl;
    for (int i = 0; i < _n_secondary_coord; ++i) {
        _model.realizeAcceleration(state);

        Coordinate& optim_coord =
            _model.updComponent<Coordinate>(_secondary_coord_path[i]);

        double init_acc = optim_coord.getAccelerationValue(state);
        double init_speed = optim_coord.getSpeedValue(state);

        optim_coord.setSpeedValue(state, init_speed + 1.0);

        _model.realizeAcceleration(state);

        double new_acc = optim_coord.getAccelerationValue(state);
        double diff = new_acc - init_acc;
        std::cout << optim_coord.getName() << " " << diff << std::endl;
        if (diff == 0) {

        }

        optim_coord.setSpeedValue(state, init_speed);
    }
    */

    // Organize and Compute Optimal Forces For Muscles/Actuators in model
    _n_actuators = 0;
    _n_muscles = 0;
    _n_non_muscle_actuators = 0;

    for (ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {

        if (actuator.appliesForce(state)) { _n_actuators++; }
    }
    _optimal_force.resize(_n_actuators);

    int i = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        if (msl.appliesForce(state)) {
            _optimal_force[i] = msl.getMaxIsometricForce();
            i++;
            _n_muscles++;
            _muscle_path.append(msl.getAbsolutePathString());
        }
    }

    for (ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {

        if (Object::isObjectTypeDerivedFrom<Muscle>(
                    actuator.getConcreteClassName())) {
            continue;
        }

        if (actuator.appliesForce(state)) {
            _optimal_force[i] = actuator.getOptimalForce();
            i++;
            _n_non_muscle_actuators++;
            _non_muscle_actuator_path.append(actuator.getAbsolutePathString());
        }
    }

    computeMuscleVolumes();
    if (get_verbose() > 0) {
        log_debug("Muscle Properties");
        log_debug("{:<15} {:<15} {:<15}", "name ", "Fmax", "Volume");
        i = 0;
        for (const Muscle& msl : _model.getComponentList<Muscle>()) {
        if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }
            double l0 = msl.get_optimal_fiber_length();
            double fmax = msl.get_max_isometric_force();

            log_debug("{:<15} {:<15} {:<15}", msl.getName(), fmax,
                    _muscle_volumes[i]);
            i++;
        }
    }
    // Setup optimization parameters
    // parameters vector ordered
    // muscle activations
    // non muscle activations
    // secondary coord values

    _optim_parameters.resize(_n_actuators + _n_secondary_coord);
    _optim_parameters = 0;

    _n_parameters = _optim_parameters.size();
    _prev_parameters = _optim_parameters;

    _optim_parameter_names.setSize(_n_parameters);

    int p = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        if (msl.appliesForce(state)) {
            _optim_parameter_names[p] = msl.getName();
            p++;
        }
    }
    for (const auto& actuator : _model.updComponentList<ScalarActuator>()) {
        if (Object::isObjectTypeDerivedFrom<Muscle>(
                    actuator.getConcreteClassName())) {
            continue;
        }
        if (actuator.appliesForce(state)) {
            _optim_parameter_names[p] = actuator.getName();
            p++;
        }
    }
    for (int p = 0; p < _n_secondary_coord; ++p) {
        _optim_parameter_names[_n_actuators + p] = _secondary_coord_name[p];
    }

    // Optimization Constraints
    _n_optim_constraints = 0;
    for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {

        bool is_primary = _primary_coord_path.findIndex(
                                  coord.getAbsolutePathString()) > -1;
        bool is_secondary = _secondary_coord_path.findIndex(
                                    coord.getAbsolutePathString()) > -1;

        if (is_primary) {
            _optim_constraint_names.append(coord.getName());
            _n_optim_constraints++;
        }
        if (is_secondary) {
            _optim_constraint_names.append(coord.getName());
            _n_optim_constraints++;
        }
    }
    if (get_verbose() > 0) {
        log_debug("IPOPT Parameter Names:");
        for (int p = 0; p < _n_parameters; p++) {
            log_debug("{} \t {}", p, _optim_parameter_names[p]);
        }

        log_debug("IPOPT Constraint Names");
        for (int p = 0; p < _n_optim_constraints; p++) {
            log_debug("{} \t {}", p, _optim_constraint_names[p]);
        }
    }
    // Organize COMAKCostFunctionParameters
    int msl_cnt = 0;
    SimTK::Vector _gamma(_n_muscles);
    for (Muscle& msl : _model.updComponentList<Muscle>()) {
        if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }
        bool found_msl = false;

        int size_cost_fcn_param_set =
                get_COMAKCostFunctionParameterSet().getSize();

        for (int j = 0; j < size_cost_fcn_param_set; ++j) {
            COMAKCostFunctionParameter parameter =
                    get_COMAKCostFunctionParameterSet().get(j);

            if (parameter.get_actuator() == msl.getAbsolutePathString()) {
                found_msl = true;

                _cost_muscle_weights.cloneAndAppend(parameter.get_weight());

                _cost_muscle_desired_act.cloneAndAppend(
                        parameter.get_desired_activation());

                _cost_muscle_act_lower_bound.cloneAndAppend(
                        parameter.get_activation_lower_bound());

                _cost_muscle_act_upper_bound.cloneAndAppend(
                        parameter.get_activation_upper_bound());

                _gamma[msl_cnt] = parameter.get_emg_gamma_weight(); // Amir
            }
        }
        if (found_msl == false) {
            _cost_muscle_weights.cloneAndAppend(Constant(1.0));
            _cost_muscle_desired_act.cloneAndAppend(Constant(0.0));
            _cost_muscle_act_lower_bound.cloneAndAppend(Constant(0.01));
            _cost_muscle_act_upper_bound.cloneAndAppend(Constant(1.0));
            _gamma[msl_cnt] = 0.0; // Amir
        }
        msl_cnt++; // Amir
    }
    _emg_gamma_weight = _gamma; // Amir

    // Check Settle Simulation Parameters
    if (get_settle_constant_muscle_control() < 0 ||
            get_settle_constant_muscle_control() > 1) {
        OPENSIM_THROW(Exception, "COMAKTOOL: settle_constant_muscle_control "
                                 "must be within [0 1].");
    }

    // Add Analysis set
    AnalysisSet aSet = get_AnalysisSet();
    int size = aSet.getSize();

    for (int i = 0; i < size; i++) {
        Analysis* analysis = aSet.get(i).clone();
        _model.addAnalysis(analysis);
    }

    // Print Contact Mesh Debug Info
    for (auto& cnt_mesh : _model.updComponentList<Smith2018ContactMesh>()) {
        cnt_mesh.printMeshDebugInfo();
    }

    return state;
}

void COMAKTool::performCOMAK() {

    SimTK::State state = initialize();

    // Read Kinematics and Compute Desired Accelerations
    extractKinematicsFromFile();

    // Check Cost Function Parameters
    log_info("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20}", "Muscles",
            "muscle_weight", "muscle_emg_gamma", "desired_act", "lower_bound",
            "upper_bound");

    int m = 0;
    for (Muscle& msl : _model.updComponentList<Muscle>()) {
        if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }

        log_info("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20}", msl.getName(),
                _cost_muscle_weights.get(m).calcValue(
                        SimTK::Vector(1, get_start_time())),

                _emg_gamma_weight[m], // Amir

                _cost_muscle_desired_act.get(m).calcValue(
                        SimTK::Vector(1, get_start_time())),

                _cost_muscle_act_lower_bound.get(m).calcValue(
                        SimTK::Vector(1, get_start_time())),

                _cost_muscle_act_upper_bound.get(m).calcValue(
                        SimTK::Vector(1, get_start_time())));
        m++;
    }

    // Initialize Secondary Kinematics
    SimTK::Vector init_secondary_values(_n_secondary_coord);

    if (get_settle_secondary_coordinates_at_start()) {
        init_secondary_values = equilibriateSecondaryCoordinates();
    } else {
        for (int i = 0; i < _n_secondary_coord; ++i) {
            init_secondary_values(i) =
                    _q_matrix(_start_frame, _secondary_coord_index[i]);
        }
    }

    log_info("Initial Secondary Coordinate Values:");

    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info("{} :\t {}", _secondary_coord_name[i],
                init_secondary_values(i));
    }

    // Apply External Loads
    applyExternalLoads();

    // Set Prescribed Coordinates
    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        bool is_prescribed = getProperty_prescribed_coordinates().findIndex(
                                     coord.getAbsolutePathString()) > -1;

        if (is_prescribed) {
            SimTK::Vector data = _q_matrix(nCoord);
            GCVSpline func = GCVSpline(
                    5, _n_frames, &_time[0], &data[0], coord.getName());
            coord.set_prescribed_function(func);
            coord.set_prescribed(true);
        }
        nCoord++;
    }

    if (get_use_visualizer()) { _model.setUseVisualizer(true); }

    _model.set_assembly_accuracy(get_model_assembly_accuracy());

    state = _model.initSystem();

    if (get_AnalysisSet().getSize() == 0) {
        disableFlippedContactMeshes(_model, state);
    }

    // Visualizer Settings
    SimTK::Visualizer* viz = NULL;
    if (get_use_visualizer()) {
        viz = &_model.updVisualizer().updSimbodyVisualizer();
        viz->setBackgroundColor(SimTK::White);
        viz->setShowSimTime(true);
        viz->setWindowTitle("Runing COMAK for: " + get_model_file());
    }

    // Setup Results Storage
    initializeResultsStorage();

    AnalysisSet& analysisSet = _model.updAnalysisSet();

    // Prepare for Optimization
    log_info("\nPerforming COMAK...\n");

    _model.setAllControllersEnabled(false);

    for (ScalarActuator& actuator : _model.updComponentList<ScalarActuator>()) {
        actuator.overrideActuation(state, true);
    }

    // Initialize optimization parameters
    for (int i = 0; i < _n_muscles; ++i) { _optim_parameters[i] = 0.02; }
    for (int i = 0; i < _n_secondary_coord; ++i) {
        _optim_parameters[i + _n_actuators] = 0.0;
    }
    _prev_parameters = _optim_parameters;

    // Set initial Secondary Qs
    _prev_secondary_value.resize(_n_secondary_coord);

    for (int j = 0; j < _n_secondary_coord; ++j) {
        Coordinate& coord =
                _model.updComponent<Coordinate>(_secondary_coord_path[j]);

        coord.setValue(state, init_secondary_values(j), false);
        _prev_secondary_value(j) = init_secondary_values(j);
    }
    _prev_state = state;

    // Loop over each time step
    //------------------------

    _consecutive_bad_frame = 0;
    int frame_num = 0;
    // columns of the linearization, kept for jacobian_recompute_interval > 1
    SimTK::Matrix jac_msl, jac_non_muscle, jac_coord, jac_speed;
    SimTK::Vector jac_energy;
    int jac_age = 0;
    // secondary coordinate values at the start of the time step
    SimTK::Vector secondary_reference(_n_secondary_coord, 0.0);
    // secondary coordinate values two time steps back, for the smoothness
    SimTK::Vector secondary_prev2(_n_secondary_coord, 0.0);
    int frames_done = 0;
    if (get_activation_change_penalty_weight() < 0 ||
            get_secondary_smoothness_penalty_weight() < 0) {
        OPENSIM_THROW(Exception, "COMAKTool: the smoothness penalty weights "
                "cannot be negative.");
    }
    if (get_activation_change_penalty_weight() > 1 ||
            get_secondary_smoothness_penalty_weight() > 1) {
        log_warn("A smoothness penalty weight above 1 is larger than the "
                 "activation cost and can keep muscles active.");
    }
    for (int i = 0; i < _n_frames; ++i) {
        if (_time[i] < get_start_time()) { continue; }
        if (_time[i] > get_stop_time()) { break; };

        // Set Time
        state.setTime(_time[i]);

        log_info("Frame: {} / {}", ++frame_num, _n_out_frames);
        log_info("Time: {}", _time[i]);
        // Print EMGs
        if (get_is_emg_assisted() && get_verbose()>0) {
            // Check Cost Function Parameters
            log_debug("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20}", "Muscles",
                    "muscle_weight", "muscle_emg_gamma", "desired_act",
                    "lower_bound", "upper_bound");
            int m = 0;
            for (Muscle& msl : _model.updComponentList<Muscle>()) {
                if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }
                if (_emg_gamma_weight[m] > 0) {
                    log_debug("{:<20} {:<20} {:<20} {:<20} {:<20} {:<20}",
                            msl.getName(),
                            _cost_muscle_weights.get(m).calcValue(
                                    SimTK::Vector(1, _time[i])),

                            _emg_gamma_weight[m], // Amir

                            _cost_muscle_desired_act.get(m).calcValue(
                                    SimTK::Vector(1, _time[i])),

                            _cost_muscle_act_lower_bound.get(m).calcValue(
                                    SimTK::Vector(1, _time[i])),

                            _cost_muscle_act_upper_bound.get(m).calcValue(
                                    SimTK::Vector(1, _time[i])));
                }
                m++;
            }
        }
        // Set Primary Qs and Us to experimental values
        for (int j = 0; j < _n_primary_coord; ++j) {
            Coordinate& coord =
                    _model.updComponent<Coordinate>(_primary_coord_path[j]);

            coord.setValue(state, _q_matrix(i, _primary_coord_index[j]), false);
            coord.setSpeedValue(state, _u_matrix(i, _primary_coord_index[j]));
        }

        // Project Secondary values based on speeds
        for (int j = 0; j < _n_secondary_coord; ++j) {
            Coordinate& coord =
                    _model.updComponent<Coordinate>(_secondary_coord_path[j]);

            double prev_value = coord.getValue(state);
            double prev_speed = coord.getSpeedValue(state);

            double new_value = prev_value +
                    get_secondary_velocity_carryover_to_next_step() * prev_speed * _dt;
            new_value = std::min(std::max(new_value,
                    prev_value - _secondary_coord_max_change[j]),
                    prev_value + _secondary_coord_max_change[j]);
            coord.setValue(state, new_value);
        }

        // Reset the Secondary Us to zero
        for (int j = 0; j < _n_secondary_coord; ++j) {
            Coordinate& coord =
                    _model.updComponent<Coordinate>(_secondary_coord_path[j]);
            coord.setSpeedValue(state, 0.0);
        }

        _model.assemble(state);
        _model.realizeVelocity(state);

        // The changes of the secondary coordinates in this time step are
        // limited relative to where the optimization starts
        for (int j = 0; j < _n_secondary_coord; ++j) {
            secondary_reference[j] = _model.getComponent<Coordinate>(
                    _secondary_coord_path[j]).getValue(state);
        }

        // Print initial optimization
        if (frame_num == 1 && get_verbose() > 1) {
            printOptimizationResultsToConsole(_optim_parameters, state);
        }

        // Iterate for COMAK Solution
        double max_udot_error = SimTK::Infinity;
        SimTK::Vector iter_max_udot_error(get_max_iterations(), 0.0);
        std::vector<std::string> iter_max_udot_coord(get_max_iterations(), "");
        SimTK::Matrix iter_parameters(get_max_iterations(), _n_parameters, 0.0);
        std::vector<SimTK::State> iter_states(get_max_iterations());
        int n_iter = 0;
        // time spent in the model evaluations (Jacobian), the optimizer and the rest
        double seconds_jacobian = 0.0;
        double seconds_optimizer = 0.0;
        const auto frame_start = std::chrono::steady_clock::now();
        auto iteration_start = frame_start;
        SimTK::Vector desired_act(_n_muscles);
        for (int iter = 0; iter < get_max_iterations(); ++iter) {
            n_iter++;
            iteration_start = std::chrono::steady_clock::now();

            log_info("---------------------------------------"
                     "---------------------------------------");
            log_info("Frame: {} \t Time: {} \t Iteration: {}", frame_num,
                    _time[i], n_iter);

            log_info("---------------------------------------"
                     "---------------------------------------");

            // Reset the optimized change in secondary coords to zero
            for (int m = 0; m < _n_secondary_coord; ++m) {
                _optim_parameters[m + _n_actuators] = 0.0;
            }

            // Reset Reserve actuators
            for (int m = 0; m < _n_non_muscle_actuators; ++m) {
                _optim_parameters[m + _n_muscles] = 0.0;
            }

            // Set Activation Limits
            int m = 0;
            for (Muscle& msl : _model.updComponentList<Muscle>()) {
                if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }
                msl.set_min_control(
                        _cost_muscle_act_lower_bound.get(m).calcValue(
                                SimTK::Vector(1, _time[i])));

                msl.set_max_control(
                        _cost_muscle_act_upper_bound.get(m).calcValue(
                                SimTK::Vector(1, _time[i])));
                m++;
            }
            // Amir ->
            if (get_use_muscle_physiology()) {
                SimTK::State& sWorkingCopy = _model.updWorkingState();
                sWorkingCopy.setTime(state.getTime());
                _model.initStateWithoutRecreatingSystem(sWorkingCopy);
                // update Q's and U's
                // log_info("state.getQ() {} \t", state.getQ());
                // std::cout << "state.getQ()" << state.getQ() << std::endl;
                sWorkingCopy.setQ(state.getQ());
                sWorkingCopy.setU(state.getU());
                _model.getMultibodySystem().realize(
                        sWorkingCopy, SimTK::Stage::Velocity);
                updatemusclemaxforce(_model.updWorkingState());
            }
            //->Amir
            ComakTarget target = ComakTarget(state, &_model, ~_udot_matrix[i],
                    _optim_parameters, _dt, get_unit_udot_epsilon(),
                    _primary_coord_path, _secondary_coord_path, _muscle_path,
                    _non_muscle_actuator_path, _secondary_coord_max_change);

            target.setOptimalForces(_optimal_force);
            target.setMuscleVolumes(_normalized_muscle_volumes);
            target.setContactEnergyWeight(get_contact_energy_weight());
            target.setNonMuscleActuatorWeight(get_non_muscle_actuator_weight());
            target.setScaleDeltaCoordinates(
                    get_optimization_scale_delta_coord());
            target.setActivationExponent(get_activation_exponent());

            // Numerical options of the optimization
            SimTK::Vector secondary_unit_scale(_n_secondary_coord, 1.0);
            for (int m = 0; m < _n_secondary_coord; ++m) {
                if (_model.getComponent<Coordinate>(_secondary_coord_path[m])
                                .getMotionType() ==
                        Coordinate::MotionType::Rotational) {
                    secondary_unit_scale[m] = get_step_penalty_rotation_length();
                }
            }
            target.setReferenceValues(secondary_reference);
            if (frames_done > 0) {
                SimTK::Vector previous_activations(_n_muscles);
                for (int m = 0; m < _n_muscles; ++m) {
                    previous_activations[m] = _prev_parameters[m];
                }
                target.setActivationChangePenalty(
                        get_activation_change_penalty_weight(),
                        previous_activations);

                SimTK::Vector predicted(_n_secondary_coord);
                for (int j = 0; j < _n_secondary_coord; ++j) {
                    predicted[j] = frames_done > 1
                            ? 2.0 * _prev_secondary_value[j] - secondary_prev2[j]
                            : _prev_secondary_value[j];
                }
                target.setSecondarySmoothnessPenalty(
                        get_secondary_smoothness_penalty_weight(), predicted);
            }
            target.setEnforceCoordinateRange(
                    get_enforce_secondary_coordinate_range());
            target.setSecondaryStepPenalty(
                    get_secondary_step_penalty_weight(), secondary_unit_scale);
            target.setJacobianOptions(get_use_central_difference_jacobian(),
                    get_ignore_secondary_speed_term(),
                    get_normalize_constraint_rows());

            target.setParameterNames(_optim_parameter_names);
            // Amir ->
            target.setVerbose(get_verbose());
            target._is_emg_assisted = get_is_emg_assisted();
            target.setEmgGammaWeight(_emg_gamma_weight);
            // -> Amir
            // Set Muscle Weight Factors
            SimTK::Vector msl_weight(_n_muscles);
            for (int m = 0; m < _n_muscles; ++m) {
                msl_weight(m) = _cost_muscle_weights.get(m).calcValue(
                        SimTK::Vector(1, _time[i]));
            }
            target.setCostFunctionWeight(msl_weight);

            // Set Desired Activations
            
            for (int m = 0; m < _n_muscles; ++m) {
                desired_act(m) = _cost_muscle_desired_act.get(m).calcValue(
                        SimTK::Vector(1, _time[i]));
            }
            target.setDesiredActivation(desired_act);

            // Reuse the columns of an earlier linearization (a chord method)
            // unless they are due, or the iterations are not converging fast
            const int interval = std::max(1, get_jacobian_recompute_interval());
            const bool stalled = iter >= 2 && iter_max_udot_error(iter - 1) >
                    0.5 * iter_max_udot_error(iter - 2);
            const bool reuse_jacobian = interval > 1 && jac_msl.nrow() > 0 &&
                    jac_age < interval - 1 && !stalled;
            if (reuse_jacobian) {
                target.setReusedJacobian(jac_msl, jac_non_muscle, jac_coord,
                        jac_speed, jac_energy);
                jac_age++;
            }

            target.initialize();
            if (!reuse_jacobian) {
                target.getJacobianColumns(jac_msl, jac_non_muscle, jac_coord,
                        jac_speed, jac_energy);
                jac_age = 0;
            }
            seconds_jacobian += std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - iteration_start).count();

            if (get_convergence_criterion() != "acceleration" &&
                    get_convergence_criterion() != "force") {
                OPENSIM_THROW(Exception, "COMAKTool: unknown "
                        "convergence_criterion '" + get_convergence_criterion()
                        + "'. Use 'acceleration' or 'force'.");
            }

            const std::string solver_name = get_solver();
            const bool use_qp = (solver_name != "ipopt");
            if (use_qp && solver_name != "casadi_qrqp") {
                OPENSIM_THROW(Exception, "COMAKTool: unknown solver '" +
                        solver_name + "'. Use 'ipopt' or 'casadi_qrqp'.");
            }
            if (use_qp && get_activation_exponent() != 2) {
                OPENSIM_THROW(Exception, "COMAKTool: the solver '" +
                        solver_name + "' needs activation_exponent = 2.");
            }

            const auto optimizer_start = std::chrono::steady_clock::now();
            const SimTK::Vector parameters_before_optimization = _optim_parameters;

            bool qp_done = false;
            if (use_qp) {
                // The cost is separable and quadratic for exponent 2, so its
                // diagonal Hessian follows from the gradient at 0 and at 1.
                const int n = _n_parameters;
                SimTK::Vector zeros(n, 0.0), ones(n, 1.0), g(n), g1(n);
                bool new_params = true;
                target.gradientFunc(zeros, new_params, g);
                target.gradientFunc(ones, new_params, g1);
                SimTK::Vector hdiag = g1 - g;

                // constraints are c0 + A (x - x_init) = 0, i.e., A x = -c(0)
                SimTK::Matrix A;
                SimTK::Vector c_at_zero;
                target.constraintJacobian(zeros, new_params, A);
                target.constraintFunc(zeros, new_params, c_at_zero);
                SimTK::Vector rhs = -c_at_zero;

                double* lo = nullptr;
                double* up = nullptr;
                target.getParameterLimits(&lo, &up);
                SimTK::Vector lower(n), upper(n);
                for (int p = 0; p < n; ++p) {
                    lower[p] = lo[p];
                    upper[p] = up[p];
                }

                SimTK::Vector x_qp;
                double max_slack = 0.0;
                std::string qp_status;
                const bool qp_ok = solveCasadiQP(
                        solver_name.substr(strlen("casadi_")), hdiag, g, A,
                        rhs, lower, upper, _optim_parameters,
                        get_qp_slack_penalty(), x_qp, max_slack, qp_status);
                if (qp_ok) {
                    // report solutions outside the bounds
                    double max_violation = 0.0;
                    int worst = 0;
                    for (int p = 0; p < n; ++p) {
                        const double v = std::max(lower[p] - x_qp[p],
                                x_qp[p] - upper[p]);
                        if (v > max_violation) { max_violation = v; worst = p; }
                    }
                    if (max_violation > 1e-6) {
                        // the active-set solver can return such a point and
                        // still report success
                        log_info("The QP solution is outside the bounds by "
                                 "{} for {} (value {}, bounds {} to {}); "
                                 "using IPOPT for this iteration",
                                max_violation, _optim_parameter_names[worst],
                                x_qp[worst], lower[worst], upper[worst]);
                    } else {
                        _optim_parameters = x_qp;
                        qp_done = true;
                    }
                } else {
                    log_error("COMAK QP solver failed: {}. Using IPOPT for "
                              "this iteration.", qp_status);
                }
                if (max_slack > 1e-6) {
                    log_info("The QP relaxed an acceleration constraint by "
                             "up to {}", max_slack);
                }
            }
            if (!qp_done) {
            SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
            SimTK::Optimizer optimizer(target, algorithm);

            optimizer.setDiagnosticsLevel(get_ipopt_diagnostics_level());
            optimizer.setMaxIterations(get_ipopt_max_iterations());
            optimizer.setConvergenceTolerance(
                    get_ipopt_convergence_tolerance());
            optimizer.setConstraintTolerance(get_ipopt_constraint_tolerance());
            optimizer.useNumericalGradient(false);
            optimizer.useNumericalJacobian(false);

            // Some IPOPT-specific settings
            optimizer.setLimitedMemoryHistory(
                    get_ipopt_limited_memory_history());
            optimizer.setAdvancedBoolOption("warm_start", true);
            optimizer.setAdvancedRealOption("nlp_scaling_max_gradient",
                    get_ipopt_nlp_scaling_max_gradient());
            optimizer.setAdvancedRealOption(
                    "nlp_scaling_min_value", get_ipopt_nlp_scaling_min_value());
            optimizer.setAdvancedRealOption(
                    "obj_scaling_factor", get_ipopt_obj_scaling_factor());
            optimizer.setAdvancedStrOption("expect_infeasible_problem", "yes");

            // optimizer.setAdvancedStrOption("hessian_approximation", "exact");

            // For debugging cost and constraint changes
            /*optimizer.setAdvancedStrOption("derivative_test", "first-order");
            optimizer.setAdvancedBoolOption(
                "derivative_test_print_all", true);
            optimizer.setAdvancedRealOption(
                "derivative_test_perturbation", 1e-6);
            */

            try {
                optimizer.optimize(_optim_parameters);
            }
            catch (const SimTK::Exception::Base& ex) {
                log_error("COMAK Optimization failed: {}", ex.getMessage());
            }
            } // END ipopt

            seconds_optimizer += std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - optimizer_start).count();

            // A failed optimization can return NaN or infinite values, which
            // would corrupt the model state. Keep the previous parameters.
            bool parameters_finite = true;
            for (int p = 0; p < _n_parameters; ++p) {
                if (!SimTK::isFinite(_optim_parameters[p])) {
                    parameters_finite = false;
                }
            }
            if (!parameters_finite) {
                log_warn("The optimization returned non-finite parameters. "
                         "Using the parameters of the previous iteration.");
                _optim_parameters = parameters_before_optimization;
            }

            // Account for optimization scale factors
            for (int m = 0; m < _n_secondary_coord; ++m) {
                _optim_parameters[m + _n_actuators] *=
                        get_optimization_scale_delta_coord();
            }

            SimTK::Vector secondary_before(_n_secondary_coord);
            for (int j = 0; j < _n_secondary_coord; ++j) {
                secondary_before[j] = _model.getComponent<Coordinate>(
                        _secondary_coord_path[j]).getValue(state);
            }

            setStateFromComakParameters(state, _optim_parameters);

            // The model assembly in setStateFromComakParameters can move the
            // coordinates away from the optimized values
            for (int j = 0; j < _n_secondary_coord; ++j) {
                const double applied = _model.getComponent<Coordinate>(
                        _secondary_coord_path[j]).getValue(state) -
                        secondary_before[j];
                const double optimized = _optim_parameters[_n_actuators + j];
                if (std::abs(applied - optimized) > 1e-5) {
                    log_info("{}: optimized change {}, change after the "
                             "model assembly {}", _secondary_coord_name[j],
                            optimized, applied);
                }
            }

            // Save iteration in case of no convergence
            iter_states[iter] = state;
            iter_parameters[iter] = ~_optim_parameters;

            // Compute udot linearized error
            _model.realizeAcceleration(state);

            // Output Optimization Results
            if (get_verbose() > 1)
                printOptimizationResultsToConsole(_optim_parameters, state);

            log_info("Optimized Acceleration Errors:");
            log_info("{:<20} {:<20} {:<20} {:<20}", "Name", "Experimental",
                    "Simulated", "Error");

            std::string max_udot_coord = "";
            int k = 0;
            max_udot_error = 0;

            // Acceleration errors by mobility, for the unbalanced force
            SimTK::Vector udot_error_by_u(state.getNU(), 0.0);
            std::vector<int> residual_u;
            std::vector<bool> residual_rotational;
            std::vector<std::string> residual_name;

            for (const auto& coord : _model.getComponentList<Coordinate>()) {
                std::string path = coord.getAbsolutePathString();
                bool is_secondary = _secondary_coord_path.findIndex(path) > -1;
                bool is_primary = _primary_coord_path.findIndex(path) > -1;

                double coord_udot = coord.getAccelerationValue(state);
                double udot_error;
                double observed_udot;

                if (is_secondary) {
                    // Cannot get kinematics from mocap
                    observed_udot = 0.0;
                } else {
                    // Accelerations from spline fit to IK joint angles
                    observed_udot = _udot_matrix(i, k);
                }

                // Difference between simulated and observed accelerations:
                // Errors in simulated accelerations occur because effect of
                // secondary coordinate change (optimized variable) on joint
                // accelerations (optimization constraint) is linearized so
                // that the slow contact model isn't solved in every call
                // to the optimization constraint and constraint jacobian
                // functions. Iterations drive this error to zero once the
                // optimization solution is only a small change from the
                // initial secondary coordinates that you linearize around.

                udot_error = fabs(observed_udot - coord_udot);

                k++;

                if (!is_secondary && !is_primary) { continue; }

                const int u_index = _model.getMatterSubsystem()
                        .getMobilizedBody(coord.getJoint().getChildFrame()
                                .getMobilizedBodyIndex())
                        .getFirstUIndex(state) + coord.getMobilizerQIndex();
                udot_error_by_u[u_index] = coord_udot - observed_udot;
                residual_u.push_back(u_index);
                residual_rotational.push_back(coord.getMotionType() ==
                        Coordinate::MotionType::Rotational);
                residual_name.push_back(coord.getName());

                if (udot_error > max_udot_error) {
                    max_udot_coord = coord.getName();
                    max_udot_error = udot_error;
                }
                if (get_verbose() == 0) {
                    if (udot_error > get_udot_tolerance())
                        log_info("{:<20} {:<20} {:<20} {:<20}", coord.getName(),
                                observed_udot, coord_udot, udot_error);
                } else {
                    log_info("{:<20} {:<20} {:<20} {:<20}", coord.getName(),
                            observed_udot, coord_udot, udot_error);
                }
                /*
                if (get_verbose() > 1) {
                    log_debug("");
                    log_debug("Optimized Muscles:");
                    log_debug("{:<20} {:<20} {:<20}", "name", "activation",
                            "EMG");
                    for (int ii = 0; ii < _n_muscles; ++ii) {
                        if (_emg_gamma_weight(ii) > 0)
                            log_debug("{:<20} {:<20} {:<20}",
                                    _optim_parameter_names[ii],
                                    _optim_parameters[ii], desired_act(ii));
                    }
                }
                */
            }

            // Unbalanced generalized force: M * (simulated - observed udot)
            SimTK::Vector force_residual;
            _model.getMatterSubsystem().multiplyByM(
                    state, udot_error_by_u, force_residual);
            double max_force_ratio = 0.0;
            double max_force = 0.0;
            std::string max_force_coord = "";
            for (size_t r = 0; r < residual_u.size(); ++r) {
                const double force = std::abs(force_residual[residual_u[r]]);
                const double tolerance = residual_rotational[r]
                        ? get_force_tolerance_rotation()
                        : get_force_tolerance_translation();
                if (force / tolerance > max_force_ratio) {
                    max_force_ratio = force / tolerance;
                    max_force = force;
                    max_force_coord = residual_name[r];
                }
            }
            log_info("Max force residual: {} (N or N*m)      Coord: {}      "
                     "Times its tolerance: {}",
                    max_force, max_force_coord, max_force_ratio);
            if (get_convergence_criterion() == "force") {
                log_info("Max acceleration error: {} (Coord: {}), not used "
                         "for convergence", max_udot_error, max_udot_coord);
                // the rest of the iteration uses max_udot_error, in units of
                // udot_tolerance: 1 is the tolerance
                max_udot_error = max_force_ratio * get_udot_tolerance();
                max_udot_coord = max_force_coord;
            }

            for (int j = 0; j < _n_secondary_coord; ++j) {
                const double change = std::abs(
                        _model.getComponent<Coordinate>(_secondary_coord_path[j])
                                .getValue(state) - secondary_reference[j]);
                if (change > _secondary_coord_max_change[j] * (1 + 1e-6) + 1e-12) {
                    log_warn("{} changed by {} in this time step, "
                             "more than its max_change {}",
                            _secondary_coord_name[j], change,
                            _secondary_coord_max_change[j]);
                }
            }

            log_info("Max udot Error: {} \t Max Error Coord: {}",
                    max_udot_error, max_udot_coord);
            iter_max_udot_error(iter) = max_udot_error;
            iter_max_udot_coord[iter] = max_udot_coord;

            // Check for convergence
            if (max_udot_error < get_udot_tolerance()) {
                _consecutive_bad_frame = 0; // converged so reset
                break;
            }

        } // END COMAK ITERATION

        const double seconds_frame = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - frame_start).count();
        log_info("Timing of this time step (s): total {:.2f}, linearization "
                 "(model evaluations) {:.2f}, optimizer {:.2f}, other {:.2f}",
                seconds_frame, seconds_jacobian, seconds_optimizer,
                seconds_frame - seconds_jacobian - seconds_optimizer);

        int frame_converged = 0;
        double min_max_udot_error = max_udot_error;
        // print muscle activations
        if (get_is_emg_assisted() == 1) {
            log_info("");
            log_info("Optimized Muscles:");
            log_info("{:<20} {:<20} {:<20}", "name", "activation", "EMG");
            for (int ii = 0; ii < _n_muscles; ++ii) {
                if (_emg_gamma_weight(ii) > 0)
                    log_info("{:<20} {:<20} {:<20}", _optim_parameter_names[ii],
                            _optim_parameters[ii], desired_act(ii));
            }
        }
        // print COMAK report for this iteration
        if (max_udot_error > get_udot_tolerance()) {

            log_info("COMAK failed to converge.");

            _consecutive_bad_frame++;

            // Reset to best iteration solution
            double min_val = get_udot_worse_case_tolerance();
            int min_iter = -1;
            std::string bad_coord;
            for (int m = 0; m < get_max_iterations(); ++m) {
                if (iter_max_udot_error(m) < min_val) {
                    min_val = iter_max_udot_error(m);
                    min_iter = m;
                    bad_coord = iter_max_udot_coord[m];
                }
            }
            if (min_iter > -1) {
                _optim_parameters = ~iter_parameters[min_iter];
                state = iter_states[min_iter];

                log_info("########################## Using best iteration ({}) with max udot error: {} ##########################",
                        min_iter, min_val);
            } else {
                _optim_parameters = _prev_parameters;

                // Go back to the secondary coordinates of the previous time
                // step. setStateFromComakParameters adds the secondary
                // changes to the current values, so the changes are zeroed.
                SimTK::Vector reset_parameters = _prev_parameters;
                for (int m = 0; m < _n_secondary_coord; ++m) {
                    reset_parameters[m + _n_actuators] = 0.0;
                    _model.updComponent<Coordinate>(_secondary_coord_path[m])
                            .setValue(state, _prev_secondary_value(m), false);
                }
                setStateFromComakParameters(state, reset_parameters);

                state.setTime(_time[i]);

                log_info("########################## No iteration has max udot error less than "
                         "worst case tolerance ({}). ##########################",
                        get_udot_worse_case_tolerance());

                log_info("Resetting optimization parameters to previous time "
                         "step solution.");
            }

            // Save data about failed convergence
            _bad_frames.push_back(i);
            _bad_times.push_back(_time[i]);
            _bad_udot_errors.push_back(min_val);
            _bad_udot_coord.push_back(bad_coord);

            min_max_udot_error = min_val;
        } else {
            log_info("");
            log_info("########################## COMAK Converged! Number of iterations: {} ##########################", n_iter);
            frame_converged = 1;
        }

        SimTK::RowVector convergence_metrics(3);
        convergence_metrics(0) = frame_converged;
        convergence_metrics(1) = n_iter;
        convergence_metrics(2) = min_max_udot_error;
        _convergence.appendRow(_time[i], convergence_metrics);

        // Store Solution
        _model.realizeAcceleration(state);
        _prev_parameters = _optim_parameters;
        _prev_state = state;

        secondary_prev2 = _prev_secondary_value;
        for (int m = 0; m < _n_secondary_coord; ++m) {
            Coordinate& coord =
                    _model.updComponent<Coordinate>(_secondary_coord_path[m]);
            _prev_secondary_value(m) = coord.getValue(state);
        }
        frames_done++;

        // Save the results
        recordResultsStorage(state, i);

        // Visualize the Results
        if (get_use_visualizer()) {
            int k = 0;
            for (int m = 0; m < _n_muscles; ++m) {
                Muscle& msl = _model.updComponent<Muscle>(_muscle_path[m]);
                msl.overrideActuation(state, false);
                _model.realizePosition(state);
                msl.setActivation(state, _optim_parameters[k]);
                k++;
            }

            _model.realizeAcceleration(state);
            viz->drawFrameNow(state);

            for (int m = 0; m < _n_muscles; ++m) {
                Muscle& msl = _model.updComponent<Muscle>(_muscle_path[m]);
                msl.overrideActuation(state, true);
            }
        }
    } // END of COMAK timestep

    // Print Convergence Summary
    log_info("Convergence Summary:");
    log_info("--------------------");

    if (_bad_frames.empty()) {
        log_info("All frames converged!");
    } else {
        log_info(
                "{:<15} {:<15}{:<15}", "Bad Times", "Bad Frames", "udot error");

        for (int i = 0; i < (int)_bad_frames.size(); i++) {
            log_info("{:<15}{:<15} {:<15}", _bad_times[i], _bad_frames[i],
                    _bad_udot_errors[i]);
        }
    }

    // Print Results
    printResultsFiles();
}

void COMAKTool::setStateFromComakParameters(
        SimTK::State& state, const SimTK::Vector& parameters) {

    // Set Muscle Activations to Optimized
    int j = 0;
    for (int m = 0; m < _n_muscles; ++m) {
        Muscle& msl = _model.updComponent<Muscle>(_muscle_path[m]);
        msl.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        msl.setOverrideActuation(state, force);
        j++;
    }

    // Set Reserve Activations to Optimized
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        ScalarActuator& actuator = _model.updComponent<ScalarActuator>(
                _non_muscle_actuator_path[m]);
        actuator.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        actuator.setOverrideActuation(state, force);
        j++;
    }

    // Set Secondary Kinematics to Optimized
    for (int m = 0; m < _n_secondary_coord; ++m) {
        Coordinate& coord =
                _model.updComponent<Coordinate>(_secondary_coord_path[m]);

        double value = parameters(_n_actuators + m) + coord.getValue(state);
        coord.setValue(state, value, false);

        double speed = (value - _prev_secondary_value(m)) / _dt;
        coord.setSpeedValue(state, speed);
    }
    _model.assemble(state);
}

void COMAKTool::initializeResultsStorage() {

    std::vector<std::string> actuator_names;

    for (int m = 0; m < _n_muscles; ++m) {
        actuator_names.push_back(_muscle_path[m]);
    }
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        actuator_names.push_back(_non_muscle_actuator_path[m]);
    }

    _result_activations.setColumnLabels(actuator_names);
    _result_forces.setColumnLabels(actuator_names);

    std::vector<std::string> kinematics_names;
    std::vector<std::string> values_names;

    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getAbsolutePathString();
        kinematics_names.push_back(name + "/value");
        kinematics_names.push_back(name + "/speed");
        kinematics_names.push_back(name + "/acc");

        values_names.push_back(coord.getName());
    }
    _result_kinematics.setColumnLabels(kinematics_names);
    _result_values.setColumnLabels(values_names);

    std::vector<std::string> convergence_names;
    convergence_names.push_back("converged");
    convergence_names.push_back("iterations");
    convergence_names.push_back("max_udot_error");
    _convergence.setColumnLabels(convergence_names);
}

void COMAKTool::recordResultsStorage(const SimTK::State& state, int frame) {
    if (frame == 0) {
        _model.updAnalysisSet().begin(state);
    } else {
        _model.updAnalysisSet().step(state, frame);
    }

    _result_states.append(state);

    SimTK::RowVector activations(_n_actuators);
    SimTK::RowVector forces(_n_actuators);

    for (int m = 0; m < _n_actuators; ++m) {
        activations(m) = _optim_parameters(m);
        forces(m) = _optim_parameters(m) * _optimal_force(m);
    }

    _result_activations.appendRow(_time[frame], activations);
    _result_forces.appendRow(_time[frame], forces);

    SimTK::RowVector kinematics(_model.getNumCoordinates() * 3);
    SimTK::RowVector values(_model.getNumCoordinates());

    int k = 0;
    int v = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        kinematics(k) = coord.getValue(state);
        kinematics(k + 1) = coord.getSpeedValue(state);
        kinematics(k + 2) = coord.getAccelerationValue(state);
        k = k + 3;

        values(v) = coord.getValue(state);
        v++;
    }
    _result_kinematics.appendRow(_time[frame], kinematics);
    _result_values.appendRow(_time[frame], values);
}

void COMAKTool::printResultsFiles() {
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " + get_results_directory() +
                                         "Possible reason: This tool cannot "
                                         "make new folder with subfolder.");
    }
    // log_info("Printing COMAK results to: {}",get_results_directory());

    STOFileAdapter sto;

    _convergence.addTableMetaData("header", std::string("COMAK Convergence"));
    _convergence.addTableMetaData(
            "nRows", std::to_string(_convergence.getNumRows()));
    _convergence.addTableMetaData(
            "nColumns", std::to_string(_convergence.getNumColumns() + 1));

    sto.write(_convergence, get_results_directory() + "/" +
                                    get_results_prefix() + "_convergence.sto");

    TimeSeriesTable states_table = _result_states.exportToTable(_model);
    states_table.addTableMetaData("header", std::string("COMAK Model States"));
    states_table.addTableMetaData(
            "nRows", std::to_string(states_table.getNumRows()));
    states_table.addTableMetaData(
            "nColumns", std::to_string(states_table.getNumColumns() + 1));
    sto.write(states_table, get_results_directory() + "/" +
                                    get_results_prefix() + "_states.sto");

    _result_activations.addTableMetaData(
            "header", std::string("COMAK Actuator Activations"));
    _result_activations.addTableMetaData(
            "nRows", std::to_string(_result_activations.getNumRows()));
    _result_activations.addTableMetaData("nColumns",
            std::to_string(_result_activations.getNumColumns() + 1));

    sto.write(_result_activations, get_results_directory() + "/" +
                                           get_results_prefix() +
                                           "_activations.sto");

    _result_forces.addTableMetaData(
            "header", std::string("COMAK Actuator Forces"));
    _result_forces.addTableMetaData(
            "nRows", std::to_string(_result_forces.getNumRows()));
    _result_forces.addTableMetaData(
            "nColumns", std::to_string(_result_forces.getNumColumns() + 1));

    sto.write(_result_forces, get_results_directory() + "/" +
                                      get_results_prefix() + "_force.sto");

    _result_kinematics.addTableMetaData("inDegrees", std::string("no"));
    _model.getSimbodyEngine().convertRadiansToDegrees(_result_kinematics);
    _result_kinematics.addTableMetaData(
            "header", std::string("COMAK Model Kinematics"));
    _result_kinematics.addTableMetaData(
            "nRows", std::to_string(_result_kinematics.getNumRows()));
    _result_kinematics.addTableMetaData(
            "nColumns", std::to_string(_result_kinematics.getNumColumns() + 1));

    sto.write(_result_kinematics, get_results_directory() + "/" +
                                          get_results_prefix() +
                                          "_kinematics.sto");

    _result_values.addTableMetaData("inDegrees", std::string("no"));
    _model.getSimbodyEngine().convertRadiansToDegrees(_result_values);
    _result_values.addTableMetaData(
            "header", std::string("COMAK Model Values"));
    _result_values.addTableMetaData(
            "nRows", std::to_string(_result_values.getNumRows()));
    _result_values.addTableMetaData(
            "nColumns", std::to_string(_result_values.getNumColumns() + 1));

    sto.write(_result_values, get_results_directory() + "/" +
                                      get_results_prefix() + "_values.sto");

    _model.updAnalysisSet().printResults(
            get_results_prefix(), get_results_directory());
}

SimTK::Vector COMAKTool::equilibriateSecondaryCoordinates() {
    Model settle_model = _model;
    SimTK::State state = settle_model.initSystem();

    log_info("----------------------------------------"
             "----------------------------------------");

    log_info("Performing forward simulation to equilibriate secondary "
             "kinematics");

    log_info("----------------------------------------"
             "----------------------------------------");

    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        msl.set_ignore_activation_dynamics(get_ignore_activation_dynamics());
        msl.set_ignore_tendon_compliance(get_ignore_tendon_compliance());
        //if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
        //    msl.set_ignore_activation_dynamics(true);
        //    msl.set_ignore_tendon_compliance(true);
        //}
    }

    log_info("ignore activation dynamics: {}, ignore tendon compliance: {}",
            get_ignore_activation_dynamics(), get_ignore_tendon_compliance());

    if (get_use_visualizer()) { settle_model.setUseVisualizer(true); }

    state = settle_model.initSystem();
    disableFlippedContactMeshes(settle_model, state);

    if (get_use_visualizer()) {
        SimTK::Visualizer& viz =
                settle_model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::White);
        viz.setWindowTitle("Settling for COMAK for: " + get_model_file());
        viz.setShowSimTime(true);
    }

    // Set initial pose and Lock all coordinates except secondary
    int nCoord = 0;
    for (Coordinate& coord : settle_model.updComponentList<Coordinate>()) {
        coord.setValue(state, _q_matrix(_start_frame, nCoord), false);
        coord.setSpeedValue(state, get_settle_zero_initial_speeds()
                                           ? 0.0
                                           : _u_matrix(_start_frame, nCoord));

        bool is_secondary = _secondary_coord_path.findIndex(
                                    coord.getAbsolutePathString()) > -1;

        if (is_secondary) {
            coord.setLocked(state, false);
        } else {
            coord.setLocked(state, true);
        }
        nCoord++;
    }

    // Don't lock coordinates that are constrained by
    // CoordinateCouplerConstraint
    for (auto& cc_const :
            settle_model.updComponentList<CoordinateCouplerConstraint>()) {

        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = settle_model.updCoordinateSet().get(cc_coord_name);

        if (cc_const.get_isEnforced()) { coord.setLocked(state, false); }
    }

    settle_model.assemble(state);
    settle_model.realizeVelocity(state);

    // Prescribe Muscle Force
    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce() *
                       get_settle_constant_muscle_control();
        // double value = msl.getMaxIsometricForce()*0.02; //Amir
        msl.setOverrideActuation(state, value);
    }

    StatesTrajectory result_states;

    // Store Secondary Coordinate Values (to check if simulation is settled)
    std::vector<const Coordinate*> settle_coords;
    for (int k = 0; k < _n_secondary_coord; k++) {
        settle_coords.push_back(&settle_model.getComponent<Coordinate>(
                _secondary_coord_path[k]));
    }

    SimTK::Vector prev_sec_coord_value(_n_secondary_coord);

    for (int k = 0; k < _n_secondary_coord; k++) {
        const Coordinate& coord = *settle_coords[k];
        prev_sec_coord_value(k) = coord.getValue(state);
    }

    // Perform settling simulation
    SimTK::CPodesIntegrator integrator(settle_model.getSystem(),
            SimTK::CPodes::BDF, SimTK::CPodes::Newton);

    integrator.setAccuracy(get_settle_accuracy());
    integrator.setInternalStepLimit(get_settle_internal_step_limit());
    SimTK::TimeStepper timestepper(settle_model.getSystem(), integrator);
    timestepper.initialize(state);

    double dt = 0.01;

    double max_coord_delta = SimTK::Infinity;
    int i = 1;
    SettleMonitor settle_monitor(get_settle_threshold(),
            get_settle_plateau_tolerance(), get_settle_plateau_window());
    bool settled = false;
    while (!settled) {
        timestepper.stepTo(i * dt);
        state = timestepper.getState();
        if (get_print_settle_sim_results()) { result_states.append(state); }

        log_info("Time: {}", state.getTime());
        log_info("{:<20} {:<15} {:<15}", "Secondary Coord", "Value",
                "Value Change (Delta)");

        // Compute Delta Coordinate
        max_coord_delta = 0;
        for (int k = 0; k < _n_secondary_coord; k++) {
            const Coordinate& coord = *settle_coords[k];

            double value = coord.getValue(state);
            double delta = std::abs(value - prev_sec_coord_value(k));

            if (delta > max_coord_delta) { max_coord_delta = delta; }
            prev_sec_coord_value(k) = value;
            if (get_verbose() > 0) {
                log_info("{:<20} {:<15} {:<15}", coord.getName(), value, delta);
            }else {
            if (delta>get_settle_threshold())
                    log_info("{:<20} {:<15} {:<15}", coord.getName(), value,
                            delta);
            }
        }
        settled = settle_monitor.update(max_coord_delta);
        i++;
    }
    if (settle_monitor.hitSafetyLimit()) {
        log_warn("Settling simulation stopped at the safety limit of {} "
                "steps without settling.", SettleMonitor::maxSteps());
    } else {
        log_info("Settling simulation finished after {} steps: {}.",
                settle_monitor.numSteps(), settle_monitor.reason());
    }

    // Print Results
    if (get_print_settle_sim_results()) {
        TimeSeriesTable states_table =
                result_states.exportToTable(settle_model);
        states_table.addTableMetaData(
                "header", std::string("COMAK Settle Simulation States"));
        states_table.addTableMetaData(
                "nRows", std::to_string(states_table.getNumRows()));
        states_table.addTableMetaData(
                "nColumns", std::to_string(states_table.getNumColumns() + 1));
        states_table.addTableMetaData("inDegrees", std::string("no"));

        int makeDir_out = IO::makeDir(get_settle_sim_results_directory());
        if (errno == ENOENT && makeDir_out == -1) {
            OPENSIM_THROW(Exception,
                    "Could not create " + get_settle_sim_results_directory() +
                            "Possible reason: This tool cannot make new "
                            "folder with subfolder.");
        }

        std::string basefile = get_settle_sim_results_directory() + "/" +
                               get_settle_sim_results_prefix();

        STOFileAdapter sto;
        sto.write(states_table, basefile + "_states.sto");
    }

    // Collect Settled Secondary Q values;
    SimTK::Vector secondary_q(_n_secondary_coord);
    for (int i = 0; i < _n_secondary_coord; ++i) {
        Coordinate& coord =
                settle_model.updComponent<Coordinate>(_secondary_coord_path[i]);

        secondary_q(i) = coord.getValue(state);
    }
    return secondary_q;
}

void COMAKTool::extractKinematicsFromFile() {

    Storage store(get_coordinates_file());
    Array<std::string> col_names = store.getColumnLabels();

    // Set Start and Stop Times
    Array<double> in_time;
    store.getTimeColumn(in_time);

    if (get_start_time() == -1) { set_start_time(in_time.get(0)); }
    if (get_stop_time() == -1) { set_stop_time(in_time.getLast()); }

    // get_time_step() is -1 if the time step of the file is used
    const double time_tol = std::max(get_time_step(), 0.0) + 1e-9;

    if (get_start_time() < in_time.get(0) - time_tol) {
        OPENSIM_THROW(Exception, "ERROR: start_time in COMAKTool preceeds "
                                 "the times listed in the input file.");
    }

    if (get_stop_time() > in_time.getLast() + time_tol) {
        OPENSIM_THROW(Exception, "ERROR: stop_time in COMAKTool exceeds "
                                 "the times listed in the input file.");
    }

    if (store.isInDegrees()) {
        _model.getSimbodyEngine().convertDegreesToRadians(store);
    }

    if (get_time_step() != -1) { store.resampleLinear(get_time_step()); }

    if (get_lowpass_filter_frequency() != -1) {
        store.pad(store.getSize() / 2);
        store.lowpassIIR(get_lowpass_filter_frequency());
    }

    store.getTimeColumn(_time);
    _dt = _time[1] - _time[0];

    for (int i = 1; i < _time.size(); ++i) {
        if (std::abs((_time[i] - _time[i - 1]) - _dt) > 1e-3 * _dt) {
            log_warn("The time step of the input kinematics is not uniform "
                    "(dt = {} at t = {}, expected {}). COMAK assumes a "
                    "constant time step.", _time[i] - _time[i - 1], _time[i],
                    _dt);
            break;
        }
    }

    // Set number of Frames
    _n_frames = _time.size();
    _n_out_frames = 0;
    bool first_frame = true;

    for (int i = 0; i < _n_frames; ++i) {
        if (_time[i] < get_start_time()) { continue; }
        if (_time[i] > get_stop_time()) { break; };

        if (first_frame) {
            _start_frame = i;
            first_frame = false;
        }

        _n_out_frames++;
    }

    // Gather Q and U values
    Array<std::string> col_labels = store.getColumnLabels();

    std::vector<int> q_col_map(_model.getNumCoordinates(), -1);
    // q_col_map = -1;

    for (int i = 0; i < col_labels.size(); ++i) {
        std::vector<std::string> split_label = split_string(col_labels[i], "/");

        int j = 0;
        for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {
            if (contains_string(split_label, coord.getName())) {
                if (split_label.back() == "value" ||
                        split_label.back() == coord.getName()) {
                    q_col_map[j] = i;
                }
            }
            j++;
        }
    }

    _q_matrix.resize(_n_frames, _model.getNumCoordinates());
    _u_matrix.resize(_n_frames, _model.getNumCoordinates());
    _udot_matrix.resize(_n_frames, _model.getNumCoordinates());

    _q_matrix = 0;
    _u_matrix = 0;
    _udot_matrix = 0;

    int j = 0;
    for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {

        if (q_col_map[j] != -1) {
            Array<double> data;
            store.getDataColumn(col_labels[q_col_map[j]], data, _time[0]);
            for (int i = 0; i < _n_frames; ++i) { _q_matrix(i, j) = data[i]; }
        } else {
            log_warn("Coordinate Value: {} not found in coordinates_file, "
                     "assuming default value.",
                    coord.getName());
            for (int i = 0; i < _n_frames; ++i) {
                _q_matrix(i, j) = coord.getDefaultValue();
            }
        }

        // GCVSpline q_spline;
        // q_spline.setDegree(5);

        std::unique_ptr<Function> q_spline;
        if (get_smooth_input_kinematics()) {
            SimTK::Vector q_data(_n_frames);
            for (int i = 0; i < _n_frames; ++i) { q_data[i] = _q_matrix(i, j); }

            // A coordinate that does not move has no noise to estimate
            double error_variance = get_smoothing_spline_error_variance();
            if (SimTK::max(q_data) - SimTK::min(q_data) < 1e-12) {
                error_variance = 0.0;
            }
            q_spline.reset(new GCVSpline(5, _n_frames, &_time[0], &q_data[0],
                    coord.getName(), error_variance));
        } else {
            SimmSpline* simm_spline = new SimmSpline();
            for (int i = 0; i < _n_frames; ++i) {
                simm_spline->addPoint(_time[i], _q_matrix(i, j));
            }
            q_spline.reset(simm_spline);
        }

        for (int i = 0; i < _n_frames; ++i) {
            SimTK::Vector x(1);
            x(0) = _time[i];

            if (get_smooth_input_kinematics()) {
                _q_matrix(i, j) = q_spline->calcValue(x);
            }

            std::vector<int> u_order = {0};
            _u_matrix(i, j) = q_spline->calcDerivative(u_order, x);

            std::vector<int> udot_order = {0, 0};
            _udot_matrix(i, j) = q_spline->calcDerivative(udot_order, x);
        }
        j++;
    }

    if (get_print_processed_input_kinematics()) {

        std::vector<double> time_vec;
        for (int i = 0; i < _time.size(); ++i) { time_vec.push_back(_time[i]); }

        std::vector<std::string> labels;
        int numCoordinates = _model.countNumComponents<Coordinate>();

        SimTK::Matrix processed_kinematics(_time.size(), numCoordinates * 3);

        int nCol = 0;
        for (auto& coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/value");
            processed_kinematics(nCol) = _q_matrix(nCol);
            nCol++;
        }

        for (auto& coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/speed");
            processed_kinematics(nCol) = _u_matrix(nCol - numCoordinates);
            nCol++;
        }

        for (auto& coord : _model.getComponentList<Coordinate>()) {
            labels.push_back(coord.getAbsolutePathString() + "/acceleration");
            processed_kinematics(nCol) = _udot_matrix(nCol - 2 * numCoordinates);
            nCol++;
        }

        TimeSeriesTable kin_table =
                TimeSeriesTable(time_vec, processed_kinematics, labels);

        kin_table.addTableMetaData("header", std::string("processed_input_q"));
        kin_table.addTableMetaData("inDegrees", std::string("no"));
        kin_table.addTableMetaData(
                "nColumns", std::to_string(kin_table.getNumColumns() + 1));
        kin_table.addTableMetaData(
                "nRows", std::to_string(kin_table.getNumRows()));

        STOFileAdapter sto;
        sto.write(kin_table, get_results_directory() + "/" +
                                     get_results_prefix() +
                                     "processed_input_kinematics.sto");

        log_info("Printed processed input kinematics to results_dir: " +
                 get_results_directory());
    }
}

void COMAKTool::applyExternalLoads() {
    if (get_external_loads_file() == "" ||
            get_external_loads_file() == "Unassigned") {

        log_info("No external loads will be applied "
                 "(external loads file not specified).");
        return;
    }

    const std::string& aExternalLoadsFileName =
            SimTK::Pathname::getAbsolutePathname(get_external_loads_file());

    // This is required so that the references to other files inside
    // ExternalLoads file are interpreted as relative paths
    // std::string savedCwd = IO::getCwd();
    // IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));

    // Create external forces
    ExternalLoads* externalLoads = nullptr;
    try {
        externalLoads = new ExternalLoads(aExternalLoadsFileName, true);

        _model.addModelComponent(externalLoads);
    } catch (const Exception& ex) {
        // Important to catch exceptions here so we can restore current
        // working directory...
        // And then we can re-throw the exception
        log_error("Error: failed to construct ExternalLoads from file {}. "
                  "Please make sure the file exists and that it contains an "
                  "ExternalLoads object or create a fresh one.",
                aExternalLoadsFileName);

        // if (getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }

    // copy over created external loads to the external loads owned by the tool
    _external_loads = *externalLoads;

    // IO::chDir(savedCwd);
    return;
}

void COMAKTool::updateModelForces() {
    // If replacing force set read in from model file, clear it here
    if (get_replace_force_set()) {
        // Can no longer just remove the model's forces.
        // If the model is connected, then the model will
        // maintain a list of subcomponents that refer to garbage.
        _model.cleanup();
        _model.updForceSet().setSize(0);
    }

    // Load force set(s)
    if (!get_force_set_file().empty()) {
        log_info("Adding force object set from {}", get_force_set_file());

        ForceSet* forceSet = new ForceSet(get_force_set_file(), true);
        _model.updForceSet().append(*forceSet);
    }
}

void COMAKTool::computeMuscleVolumes() {
    double max_iso_stress = 350000.0; // Muscle Specific Tension

    SimTK::Vector msl_volume(_n_muscles, 0.0);

    int i = 0;
    for (const Muscle& msl : _model.getComponentList<Muscle>()) {
        if (_muscle_path.findIndex(msl.getAbsolutePathString()) < 0) { continue; }
        double l0 = msl.get_optimal_fiber_length();
        double fmax = msl.get_max_isometric_force();

        msl_volume[i] = (l0 * fmax) / max_iso_stress;
        i++;
    }
    _muscle_volumes = msl_volume;

    _normalized_muscle_volumes = msl_volume /= msl_volume.normInf();
}

void COMAKTool::printCOMAKascii() {
    log_info(
        "#################################################################################################\n"
        "#################################################################################################\n"
        "##                                                                                             ##\n"
        "##    WWWWWWWWWWWW               WWWWWWW    WWWWWWWW                   WWWWWWWWWWWW            ##\n"
        "##    WWWWWWWWWWWW               WWWWWWW    WWWWWWWW                WWWWWWWWWWWWWWWWW          ##\n"
        "##    WWWW                      WWWWWWWWW   WWWWWWWW               WWWWWW                      ##\n"
        "##    WWWWWWWWWWWW              WWWWW  WWWWWWW  WWWWW             WWWWWW      WWWWWWWWW        ##\n"
        "##    WWWWWWWWWWWW              WWWWW  WWWWWWW  WWWWW             WWWWWW      WWWWWWWWW        ##\n"
        "##    WWWW                     WWWWWW   WWWWW   WWWWW             WWWWWW         WWW           ##\n"
        "##    WWWWWWWWWWWW             WWWWWW  WWWWWW   WWWWWW            WWWWWWWWWWWWWWWWWW           ##\n"
        "##    WWWWWWWWWWWW            WWWWWWW WWWWWWWW  WWWWWWW              WWWWWWWWWWWW              ##\n"
        "##                                                                                             ##\n"
        "##    WWWWWW WWWW      WWWWWWW        WWWWWWW    WWWWWWWW       WWWWWWWWW    WWWWWWWW  WWWWWWW ##\n"
        "##  WWWWWWWWWWWWW   WWWWWWWWWWWWW    WWWWWWWWW   WWWWWWWW        WWWWWWWWW    WWWWWWW  WWWWWWW ##\n"
        "## WWWWWW    WWWW  WWWWWW   WWWWWW   WWWWW  WWWWWWW  WWWWW       WWWWWWWWW     WWWWWW  WWWW    ##\n"
        "## WWWWWW          WWWWWW   WWWWWW   WWWWW  WWWWWWW  WWWWW       WWW  WWWWW    WWWWWWWWWWW     ##\n"
        "## WWWWWW    WWW   WWWWWW   WWWWWW  WWWWWW   WWWWW   WWWWW      WWWWWWWWWWW    WWWWWW WWWWWW   ##\n"
        "## WWWWWWW  WWWWW   WWWWWW WWWWWW   WWWWWW  WWWWWW   WWWWWW   WWWWW    WWWWWW WWWWWWW  WWWWWWW ##\n"
        "##   WWWWWWWWWW       WWWWWWWWW    WWWWWWW WWWWWWWW  WWWWWWW  WWWWWW  WWWWWWW WWWWWWWW WWWWWWW ##\n"
        "##                                                                                             ##\n"
        "#################################################################################################\n"
        "#################################################################################################\n"
        "      ##                                   EMG-Assisted                                   ##\n"
        "      ##           Concurrent Optimization of Muscle Activations and Kinematics           ##\n"
        "      ##                                                                                  ##\n"
        "      ##                     Implemented by Amir Esrafilian [1],                          ##\n"
        "      ## on top of COMAK, originally developed by Colin Smith [2,3] and Darryl Thelen [2] ##\n"
        "      ##                        1. University of Eastern Finland                          ##\n"
        "      ##                       2. University of Wisconsin-Madison                         ##\n"
        "      ##                                   3. ETH Zurich                                  ##\n"
        "      ######################################################################################\n"
        "      ######################################################################################\n\n"
    );
};

void COMAKTool::printOptimizationResultsToConsole(
        const SimTK::Vector& parameters, const SimTK::State& state) {

    for (int k = 0; k < _n_secondary_coord; ++k) {
        if (parameters[_n_actuators + k] == _secondary_coord_max_change[k]) {
            log_warn("Warning: {} optimized to max change: {}",
                    _secondary_coord_name[k], _secondary_coord_max_change[k]);
        }
    }

    // Optimization Parameters
    log_debug("");
    log_debug("Optimized Muscles:");
    log_debug("{:<20} {:<20} {:<20}", "name", "activation", "force");

    int p = 0;
    for (int k = 0; k < _n_muscles; ++k) {
        log_debug("{:<20} {:<20} {:<20}", _optim_parameter_names[p],
                parameters[p], parameters[p] * _optimal_force[p]);
        p++;
    }

    log_debug("");
    log_debug("Optimized Non Muscle Actuators:");
    log_debug("{:<20} {:<20} {:<20}", "name", "activation", "force");

    for (int k = 0; k < _n_non_muscle_actuators; ++k) {
        log_debug("{:<20} {:<20} {:<20}", _optim_parameter_names[p],
                parameters[p], parameters[p] * _optimal_force[p]);
        p++;
    }

    log_debug("");
    log_debug("Optimized Secondardy Coordinates:");
    log_debug("{:<20} {:<20} {:<20}", "name", "value", "change");

    for (int k = 0; k < _n_secondary_coord; ++k) {
        double value = _model.updComponent<Coordinate>(_secondary_coord_path[k])
                               .getValue(state);

        log_debug("{:<20} {:<20} {:<20}", _secondary_coord_name[k], value,
                parameters[_n_actuators + k]);
    }

    _model.realizeReport(state);

    log_debug("");
    log_debug("Blankevoort1991Ligaments:");
    log_debug("{:<20} {:<10} {:<10} {:<10} {:<10} {:<10}", "Name",
            "Total_Force", "Spring_Force", "Damping_Force", "Lengths",
            "Strains");

    for (auto& lig : _model.updComponentList<Blankevoort1991Ligament>()) {
        double total_frc = lig.getOutputValue<double>(state, "total_force");
        double spring_frc = lig.getOutputValue<double>(state, "spring_force");
        double damping_frc = lig.getOutputValue<double>(state, "damping_force");
        double length = lig.getOutputValue<double>(state, "length");
        double strain = lig.getOutputValue<double>(state, "strain");

        log_debug("{:<20} {:<10} {:<10} {:<10} {:<10} {:<10}", lig.getName(),
                total_frc, spring_frc, damping_frc, length, strain);
    }

    log_debug("");
    log_debug("Smith2018ArticularContactForce: casting_mesh");
    log_debug("{:<20} {:<10} {:<10} {:<10} {:<10} {:<10} {:<10} {:<10}", "Name",
            "Num_Contacting_Tri", "Contact_Force", "Contact_Moment",
            "Mean Pressure", "Max Pressure", "Mean Proximity", "Max Proximity");

    for (auto& cnt :
            _model.updComponentList<Smith2018ArticularContactForce>()) {

        int num_cnt_tri = cnt.getOutputValue<int>(
                state, "casting_num_contacting_triangles");
        SimTK::Vec3 cnt_frc = cnt.getOutputValue<SimTK::Vec3>(
                state, "casting_total_contact_force");
        SimTK::Vec3 cnt_moment = cnt.getOutputValue<SimTK::Vec3>(
                state, "casting_total_contact_moment");

        double mean_prs = cnt.getOutputValue<double>(
                state, "casting_total_mean_pressure");
        double max_prs =
                cnt.getOutputValue<double>(state, "casting_total_max_pressure");
        double mean_prx = cnt.getOutputValue<double>(
                state, "casting_total_mean_proximity");
        double max_prx = cnt.getOutputValue<double>(
                state, "casting_total_max_proximity");

        log_debug("{:<20} {:<10} {:<10} {:<10} {:<10} {:<10} {:<10} {:<10}",
                cnt.getName(), num_cnt_tri, cnt_frc, cnt_moment, mean_prs,
                max_prs, mean_prx, max_prx);
    }

    log_debug("");
    log_debug("Model Coordinates");
    log_debug("{:<20} {:<10} {:<10} {:<10}", "Name", "Value", "Speed",
            "Acceleration");

    for (auto& coord : _model.updComponentList<Coordinate>()) {

        log_debug("{:<20} {:<10} {:<10} {:<10}", coord.getName(),
                coord.getValue(state), coord.getSpeedValue(state),
                coord.getAccelerationValue(state));
    }
}

void COMAKTool::updatemusclemaxforce(SimTK::State& s) {
    int i = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        if (msl.appliesForce(s)) {
            if (get_use_muscle_physiology())
                _optimal_force[i] =
                        msl.calcInextensibleTendonActiveFiberForce(s, 1.0);
            else
                _optimal_force[i] = msl.getMaxIsometricForce();
            /* std::cout << msl.getAbsolutePathString() << "\t"
                      << _optimal_force[i] << std::endl;
                      */
            i++;
        }
    }
}
