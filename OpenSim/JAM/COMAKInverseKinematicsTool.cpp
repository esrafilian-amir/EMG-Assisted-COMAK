/* -------------------------------------------------------------------------- *
 *                     COMAKInverseKinematicsTool.cpp                         *
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


//=============================================================================
// INCLUDES
//=============================================================================

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>

#include "COMAKInverseKinematicsTool.h"
#include "JAMUtilities.h"
#include "VisualizerCloser.h"
#include "VisualizerInfo.h"


#include <OpenSim/Common/Adapters.h>
#include <OpenSim/Common/SimmSpline.h>
#include "OpenSim/Common/Constant.h"
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include <OpenSim/Actuators/SpringGeneralizedForce.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <set>
#include <thread>
#include <vector>

#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Analyses/Kinematics.h>

#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"

using namespace OpenSim;
//using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
/**
 * Default constructor.
 */
COMAKInverseKinematicsTool::COMAKInverseKinematicsTool() 
{
    constructProperties();
    //_directoryOfSetupFile = "";
    _model_exists = false;
}

COMAKInverseKinematicsTool::COMAKInverseKinematicsTool(const std::string file) 
    : Object(file) {
    constructProperties();
    updateFromXMLDocument();
    _model_exists = false;
    //_directoryOfSetupFile = IO::getParentDirectory(file);
    //IO::chDir(_directoryOfSetupFile); 
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKInverseKinematicsTool::constructProperties()
{
    constructProperty_setup_version(0);
    constructProperty_upgrade_old_setup(true);
    constructProperty_model_file("");
    constructProperty_results_directory("");
    constructProperty_results_prefix("");

    constructProperty_perform_secondary_constraint_sim(true);
    constructProperty_secondary_coordinates();
    constructProperty_constrained_coordinates();
    constructProperty_secondary_coupled_coordinate("");
    constructProperty_secondary_constraint_sim_settle_threshold(1e-4);
    constructProperty_secondary_constraint_sim_settle_plateau_tolerance(0.1);
    constructProperty_secondary_constraint_sim_settle_plateau_window(10);
    constructProperty_secondary_constraint_sim_muscle_activation(0.02);
    constructProperty_settling_sim_dt(0.005);
    constructProperty_enforce_secondary_coordinate_range(false);
    constructProperty_secondary_constraint_sim_sweep_time(3.0);
    constructProperty_sweep_dt(0.005);
    constructProperty_secondary_coupled_coordinate_start_value(-10.0);
    constructProperty_secondary_coupled_coordinate_stop_value(110.0);
    constructProperty_secondary_constraint_sim_integrator_accuracy(1e-6);
    constructProperty_settling_sim_integrator_accuracy(1e-10);
    constructProperty_secondary_constraint_sim_internal_step_limit(3000);
    constructProperty_secondary_constraint_sim_recovery(false);
    constructProperty_recovery_hold_outputs(20);
    {
        Array<double> mass_scales(6.0, 2);
        mass_scales[1] = 18.0;
        constructProperty_recovery_mass_scales(mass_scales);
        Array<double> inertia_scales(50.0, 2);
        inertia_scales[1] = 150.0;
        constructProperty_recovery_inertia_scales(inertia_scales);
        Array<double> damper_scales(2.0, 2);
        damper_scales[1] = 6.0;
        constructProperty_recovery_damper_scales(damper_scales);
    }
    constructProperty_recovery_contact_modulus_scale(0.3);
    constructProperty_recovery_bodies();
    constructProperty_recovery_hop_max_time(1.0);
    constructProperty_constraint_function_num_interpolation_points(20);
    constructProperty_secondary_constraint_function_file(
        "secondary_coordinate_constraint_functions.xml");
    constructProperty_print_secondary_constraint_sim_results(false);
    constructProperty_perform_inverse_kinematics(true);
    constructProperty_IKTaskSet(IKTaskSet());
    constructProperty_marker_file("");
    constructProperty_coordinate_file("");
    constructProperty_output_motion_file("");
    constructProperty_ik_constraint_weight(SimTK::Infinity);
    constructProperty_ik_accuracy(1e-5);
    constructProperty_smooth_ik_second_pass(true);
    constructProperty_smooth_ik_weight(1.0);
    constructProperty_smooth_ik_lowpass_frequency(6.0);
    Array<double> range{SimTK::Infinity, 2};
    range[0] = -SimTK::Infinity; 
    constructProperty_time_range(range);
    constructProperty_report_errors(false);
    constructProperty_report_marker_locations(false);
    constructProperty_constrained_model_file("");
    constructProperty_geometry_folder("");
    constructProperty_use_visualizer(false);
    constructProperty_auto_close_visualizer(false);
    constructProperty_show_solver_info_in_visualizer(true);
    constructProperty_verbose(0);
}

void COMAKInverseKinematicsTool::setModel(Model& model) {
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }

    _model = model;
    set_model_file(model.getDocumentFileName());
    _model_exists = true;
}

void COMAKInverseKinematicsTool::updateFromXMLNode(SimTK::Xml::Element& node,
        int versionNumber) {
    // A setup file without setup_version was written for the old defaults,
    // unless it already has properties that were added later. The upgrade is
    // done right after the file is read, so values set afterwards in a
    // script are kept.
    const bool no_version = !node.hasElement("setup_version");
    const bool has_newer = node.hasElement("enforce_secondary_coordinate_range");
    const bool old_file = no_version && !has_newer;
    Super::updateFromXMLNode(node, versionNumber);
    if (no_version && has_newer) { set_setup_version(1); }
    if (old_file) { upgradeOldSetup(); }
}

void COMAKInverseKinematicsTool::upgradeOldSetup() {
    if (get_setup_version() >= 1) { return; }
    set_setup_version(1);
    _old_setup_notice = true;
    if (!get_upgrade_old_setup()) { return; }
    _setup_upgraded = true;

    log_warn("This COMAKInverseKinematicsTool has no setup_version: it was "
             "written for an older version, so it is upgraded to version 1 "
             "(settle settings). To use the settings exactly as given, call "
             "set_setup_version(1) in the script or set upgrade_old_setup to "
             "false.");
    // the settle threshold of an old setup is replaced by the current default
    if (std::abs(get_secondary_constraint_sim_settle_threshold() - 1e-4) >
            1e-13) {
        _upgrade_changes.push_back(
                "secondary_constraint_sim_settle_threshold: " +
                std::to_string(get_secondary_constraint_sim_settle_threshold()) +
                " -> 1e-4");
        set_secondary_constraint_sim_settle_threshold(1e-4);
    }
    if (get_smooth_ik_second_pass()) {
        _upgrade_changes.push_back("smooth_ik_second_pass: true (the "
                "coordinates are smoothed in a second pass, see "
                "smooth_ik_lowpass_frequency)");
    }
    if (get_secondary_constraint_sim_settle_plateau_tolerance() == 0.0) {
        _upgrade_changes.push_back(
                "secondary_constraint_sim_settle_plateau_tolerance: 0 -> 0.1");
        set_secondary_constraint_sim_settle_plateau_tolerance(0.1);
    }
}

std::string COMAKInverseKinematicsTool::saveUpgradedSetup(
        const std::string& where) {
    upgradeOldSetup();
    const std::string file = upgradedSetupFile(getDocumentFileName(), where,
            get_setup_version(), "comak_inverse_kinematics_settings");
    print(file);
    log_info("Saved the setup (version {}) to {}", get_setup_version(), file);
    return file;
}

bool COMAKInverseKinematicsTool::run()
{
    bool completed = false;
    upgradeOldSetup();
    
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {
        const Stopwatch stopwatch;
        log_critical("");
        log_critical("==========================");
        log_critical("COMAKInverseKinematicsTool");
        log_critical("==========================");
        log_critical("");
        if (_old_setup_notice) {
            logOldSetupNotice("COMAKInverseKinematicsTool", _upgrade_changes,
                    _setup_upgraded);
        }

        initialize();
    
        //Secondary Constraint Simulation
        if (get_perform_secondary_constraint_sim()) {
            performIKSecondaryConstraintSimulation();
        }

        //Inverse Kinematics 
        if (get_perform_inverse_kinematics()) {
            performIK();
        }

        const long long elapsed = stopwatch.getElapsedTimeInNs();

        log_info("COMAKInverseKinematics complete.");
        log_info("Finished in {}", stopwatch.formatNs(elapsed));
        log_info("Printed results to: {}", get_results_directory());
        log_info("");

        completed = true;
    }

    catch(const std::exception& x) {
        log_error("COMAKInverseKinematicsTool::run() caught an exception: \n {}", x.what());
        cwd.restore();
    }
    catch (...) { // e.g. may get InterruptedException
        log_error("COMAKInverseKinematicsTool::run() caught an exception.");
        cwd.restore();
    }

    cwd.restore();

    return completed;
}

bool COMAKInverseKinematicsTool::initialize()
{
    //Make results directory
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " +
            get_results_directory() +
            "Possible reason: This tool cannot make new folder with subfolder.");
    }

    //Set Model
    if (!get_geometry_folder().empty()) {
        ModelVisualizer::addDirToGeometrySearchPaths(get_geometry_folder());
    }

    if (!_model_exists) {
        if (get_model_file().empty()) {
            OPENSIM_THROW(Exception, "No model was set in the COMAKInverseKinematicsTool.");
        }
        _model = Model(get_model_file());
    }

    // check marker file exists
    if (!SimTK::Pathname::fileExists(get_marker_file()) &&
            get_perform_inverse_kinematics()) {
        OPENSIM_THROW(Exception,"COMAKInverseKinematicsTool: marker_file "
            "does not exist! " + get_marker_file())
    }
    

    std::string function_file = get_secondary_constraint_function_file();

    if (get_secondary_constraint_function_file() == "") {
        OPENSIM_THROW(Exception, "secondary_constraint_function file not set.")
    }
    
    _model.initSystem();

    //Verfiy Coordinate Properties    
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();

        //Reset to full path
        if (get_secondary_coupled_coordinate() == name) {
            set_secondary_coupled_coordinate(path);
        }

        int ind = getProperty_secondary_coordinates().findIndex(name);
        if (ind > -1) {
            set_secondary_coordinates(ind, path);
        }

        ind = getProperty_constrained_coordinates().findIndex(name);
        if (ind > -1) {
            set_constrained_coordinates(ind, path);
        }
    }

    //Make sure Coordinate exists in model and no duplicates
    std::string name = get_secondary_coupled_coordinate();
    try { _model.getComponent<Coordinate>(name); }
    catch (Exception const&) {
        OPENSIM_THROW(Exception, "secondary_coupled_coord: " + 
            name + " not found in model.")
    }
    
    for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
        std::string name = get_secondary_coordinates(i);
        try { _model.getComponent<Coordinate>(name); }
        catch (Exception const&){
            OPENSIM_THROW(Exception,"secondary_coordinate: " + 
                name + "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < getProperty_secondary_coordinates().size(); ++j) {
            if (name == get_secondary_coordinates(j)) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, name + 
            "listed multiple times in secondary_coordinates")
    }

    // add here if constrained coordinate is duplicated or repeated elsewhere**************************************

    //Count numbers
    _n_secondary_coord = getProperty_secondary_coordinates().size();

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);

    _n_constrained_coord = getProperty_constrained_coordinates().size();
    _constrained_coord_name.setSize(_n_constrained_coord);
    _constrained_coord_path.setSize(_n_constrained_coord);
    _constrained_coord_index.setSize(_n_constrained_coord);


    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] = get_secondary_coordinates(i);
        _secondary_coord_name[i] = _model.getComponent<Coordinate>
            (get_secondary_coordinates(i)).getName();
    }

    for (int i = 0; i < _n_constrained_coord; ++i) {
        _constrained_coord_path[i] = get_constrained_coordinates(i);
        _constrained_coord_name[i] =
                _model.getComponent<Coordinate>(get_constrained_coordinates(i))
                        .getName();
    }

    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _secondary_coord_path.findIndex(path);
        if (ind > -1) {
            _secondary_coord_index[ind] = nCoord;
        }
        nCoord++;
    }

    nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _constrained_coord_path.findIndex(path);
        if (ind > -1) { _constrained_coord_index[ind] = nCoord; }
        nCoord++;
    }
    if (get_settling_sim_integrator_accuracy() < 1e-9) {
        log_info("settling_sim_integrator_accuracy {} is below 1e-9: the "
                "secondary_constraint_sim_integrator_accuracy {} is used.",
                get_settling_sim_integrator_accuracy(),
                get_secondary_constraint_sim_integrator_accuracy());
    }

    log_info("Secondary Coordinates:");
    log_info("----------------------");
    for (int i = 0; i < _n_secondary_coord; ++i) {
        log_info(_secondary_coord_name[i]);
    }

    log_info("----------------------");
    log_info("constrained Coordinates:");
    log_info("----------------------");
    for (int i = 0; i < _n_constrained_coord; ++i) {
        log_info(_constrained_coord_name[i]);
    }
    log_info("----------------------");
    log_info("Secondary Coupled Coordinate: ",
        get_secondary_coupled_coordinate());

    if (get_perform_secondary_constraint_sim()) {
        log_info("Settle Threshold: {}",
            get_secondary_constraint_sim_settle_threshold());

        log_info("Sweep Time: {}", 
            get_secondary_constraint_sim_sweep_time());

        log_info("Sweep secondary_coupled_coordinate start value: {}",
            get_secondary_coupled_coordinate_start_value());

        log_info("Sweep secondary_coupled_coordinate stop value: {}",
            get_secondary_coupled_coordinate_stop_value());

    }

    _state = _model.initSystem();

    return true;
}




void COMAKInverseKinematicsTool::performIKSecondaryConstraintSimulation() {
    log_info("Performing IK Secondary Constraint Simulation...");

    //Initialize Model
    Model model = *_model.clone();
    // Declared after the model: its windows are closed before the model goes.
    VisualizerCloser visualizer_closer(
            get_use_visualizer() && get_auto_close_visualizer(),
            kAutoCloseVisualizerDelay);
    // Closes the current window if the simulation ends with an exception.
    VisualizerCloseOnError visualizer_close_on_error(visualizer_closer, model);
    // The visualizer is switched on just before the settling simulation
    // (below). Opening it here as well only creates an empty first window.
    model.setUseVisualizer(false);
    model.initSystem();

    for (Muscle& msl : model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }

    //Set coordinate types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(get_enforce_secondary_coordinate_range());
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            coord.set_clamped(false);
            coord.set_prescribed(true);
        } 
        else if (getProperty_constrained_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(false);
        }
        else {
            coord.set_locked(true);
        }
    }
    
    for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
        coord.set_locked(false);
        coord.set_clamped(false);
    }

    Coordinate& coupled_coord = 
        model.updComponent<Coordinate>(get_secondary_coupled_coordinate());
    
    double start_value;
    double stop_value;

    if (model.getComponent<Coordinate>(get_secondary_coupled_coordinate()).
            getMotionType() == Coordinate::MotionType::Rotational) {

        start_value = 
            get_secondary_coupled_coordinate_start_value() * SimTK::Pi / 180;
        stop_value = 
            get_secondary_coupled_coordinate_stop_value() * SimTK::Pi / 180;
    }
    else {
        start_value = get_secondary_coupled_coordinate_start_value();
        stop_value = get_secondary_coupled_coordinate_stop_value();
    }

    //Initialize Simulation 
    //---------------------
    // The constrained coordinates (e.g., Patella Flex) are free in the settling
    // and sweep simulations, like the secondary coordinates. They move in
    // response to the ligament, contact and muscle forces and the knee
    // motion, and their constraint functions are computed from the sweep.
    for (int i = 0; i < _n_constrained_coord; ++i) {
        log_info("Constrained coordinate {} is free in the secondary "
                "constraint simulation.", get_constrained_coordinates(i));
    }
       
    // The model is settled at 0 if the range of the sweep spans 0 (min below
    // 0, max above), otherwise at min. The sweeps start from the settled state.
    const double settle_value =
            (start_value < 0 && stop_value > 0) ? 0.0 : start_value;

    //Perform Settling Simulation
    //---------------------------

    //prescribe coupled coord
    Constant settle_func = Constant(settle_value);
    coupled_coord.set_prescribed_function(settle_func);
    
    model.setUseVisualizer(get_use_visualizer());
    SimTK::State state = model.initSystem();

    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setWindowTitle("Settling: " + get_model_file());
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
        viz.setDesiredFrameRate(100);        
    }

    //prescribe muscle force
    for (Muscle& msl : model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce() *
                get_secondary_constraint_sim_muscle_activation();
        msl.setOverrideActuation(state, value);
    }
    model.equilibrateMuscles(state);

    //setup integrator
    SimTK::CPodesIntegrator integrator(
        model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator.setAccuracy(
            get_settling_sim_integrator_accuracy() < 1e-9
                    ? get_secondary_constraint_sim_integrator_accuracy()
                    : get_settling_sim_integrator_accuracy());

    if (get_secondary_constraint_sim_internal_step_limit() != -1) {
        integrator.setInternalStepLimit(
            get_secondary_constraint_sim_internal_step_limit());
    }
    SimTK::TimeStepper timestepper(model.getSystem(), integrator);

    timestepper.initialize(state);
    
    StatesTrajectory settle_states;

    double dt = get_settling_sim_dt();
 

    log_info("Starting Settling Simulation.");
    

    // The constrained coordinates are free in the settling simulation, so they
    // have to settle as well.
    std::vector<const Coordinate*> settle_coords;
    for (int k = 0; k < _n_secondary_coord; k++) {
        settle_coords.push_back(
                &model.getComponent<Coordinate>(_secondary_coord_path[k]));
    }
    for (int k = 0; k < _n_constrained_coord; k++) {
        settle_coords.push_back(
                &model.getComponent<Coordinate>(_constrained_coord_path[k]));
    }
    const int n_settle = (int)settle_coords.size();

    SimTK::Vector prev_sec_coord_value(n_settle, 0.0);

    for (int k = 0; k < n_settle; k++) {
        const Coordinate& coord = *settle_coords[k];
        double value = coord.getValue(state);
        prev_sec_coord_value(k) = value;
    }

    double max_coord_delta = SimTK::Infinity;
    int i = 1;
    SettleMonitor settle_monitor(
            get_secondary_constraint_sim_settle_threshold(),
            get_secondary_constraint_sim_settle_plateau_tolerance(),
            get_secondary_constraint_sim_settle_plateau_window());
    bool settled = false;
    while (!settled) {
        timestepper.stepTo(i*dt);
        state = timestepper.getState();
        if (get_print_secondary_constraint_sim_results()) {
            settle_states.append(state);
        }
            
        log_info("Time: {}", state.getTime());
        log_info("{:<30} {:<20} {:<20}","Name", "Value", "Value Change");
        

        //Compute Delta Coordinate
        max_coord_delta = 0;
        for (int k = 0; k < n_settle; k++) {
            
            const Coordinate& coord = *settle_coords[k];
            double value = coord.getValue(state);
            double delta = std::abs(value - prev_sec_coord_value(k));
            
            if (delta > max_coord_delta) {
                max_coord_delta = delta;
            }
            prev_sec_coord_value(k) = value;

            if (get_verbose()>0)
            log_info("{:<30} {:<20} {:<20}", coord.getName(), value, delta);
            else {
                if (delta > get_secondary_constraint_sim_settle_threshold())
                    log_info("{:<30} {:<20} {:<20}", coord.getName(), value,
                            delta);
            }

        }
        if (get_verbose() > 0) printDebugInfo(model, state);

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

    // The settling window is done (the sweep opens its own window)
    visualizer_closer.closeLater(model, "Settling: " + get_model_file());

    SimTK::Vector settled_secondary_values(_secondary_coord_path.getSize());
    SimTK::Vector settled_secondary_speeds(_secondary_coord_path.getSize());
    SimTK::Vector settled_constrained_values(_n_constrained_coord);
    SimTK::Vector settled_constrained_speeds(_n_constrained_coord);

    //Save secondardy coord values to initialize sweep simulation
    for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
        std::string secondary_coord = _secondary_coord_path[c];
        Coordinate& coord = model.updComponent<Coordinate>(secondary_coord);
        settled_secondary_values.set(c, coord.getValue(state));
        settled_secondary_speeds.set(c, coord.getSpeedValue(state));
    }
    // the constrained coordinates (e.g. patella flexion) are free in the
    // settling simulation too, so the sweep starts from their settled values
    for (int c = 0; c < _n_constrained_coord; c++) {
        const Coordinate& coord =
                model.getComponent<Coordinate>(_constrained_coord_path[c]);
        settled_constrained_values.set(c, coord.getValue(state));
        settled_constrained_speeds.set(c, coord.getSpeedValue(state));
    }


     log_info("Finished Settling Simulation in {} s.", state.getTime());
     log_info("Starting Sweep Simulation.");


    //Perform Sweep Simulation
    //------------------------

    // The ligament lengths, strains and forces during the sweep are written
    // only if print_secondary_constraint_sim_results is true
    StatesTrajectory sweep_states;
    // the text in the window of the sweeps (show_solver_info_in_visualizer)
    std::shared_ptr<SolverInfo> sweep_info = std::make_shared<SolverInfo>();
    std::vector<KneeForces> sweep_knees;
    bool sweep_contact_failed = false;
    std::vector<const Blankevoort1991Ligament*> sweep_ligaments;
    TimeSeriesTable lig_table;
    int n_lig_columns = 0;
    if (get_print_secondary_constraint_sim_results()) {
        std::vector<std::string> lig_labels;
        lig_labels.push_back(get_secondary_coupled_coordinate() + "/value");
        for (int j = 0; j < _n_constrained_coord; ++j) {
            lig_labels.push_back(_constrained_coord_path[j] + "/value");
        }
        for (const auto& lig :
                model.getComponentList<Blankevoort1991Ligament>()) {
            sweep_ligaments.push_back(&lig);
            lig_labels.push_back(lig.getName() + "_length");
            lig_labels.push_back(lig.getName() + "_strain");
            lig_labels.push_back(lig.getName() + "_total_force");
        }
        lig_table.setColumnLabels(lig_labels);
        n_lig_columns = (int)lig_labels.size();
    }

    // One sweep: the coupled coordinate is prescribed from the settled value to
    // target_value (quadratic in time, starting at rest) from the settled
    // state. The coordinates are recorded in the returned table; the states
    // and ligament outputs only if record is true.
    // Fits the constraint functions of a table of sweep results (the
    // coupled coordinate must increase) and prints them to file.
    auto fit_constraint_functions = [&](const TimeSeriesTable& table,
            const std::string& file) {
        //Compute Coupled Constraint Functions
        std::vector<double> time = table.getIndependentColumn();
    
        SimTK::Vector ind_data = table.getDependentColumn(get_secondary_coupled_coordinate() + "/value");
    
        SimTK::Matrix data((int)time.size(), _n_secondary_coord + _n_constrained_coord);
    
        for (int j = 0; j < _n_secondary_coord; ++j) {
            std::string path = _secondary_coord_path[j];
            SimTK::Vector col_data = table.getDependentColumn(path + "/value");
    
            for (int i = 0; i < (int)time.size(); ++i) {
                data(i, j) = col_data(i);
    
            }
        }
        for (int j = 0; j < _n_constrained_coord; ++j) {
            std::string path = _constrained_coord_path[j];
            SimTK::Vector col_data = table.getDependentColumn(path + "/value");
    
            for (int i = 0; i < (int)time.size(); ++i) {
                data(i, j + _n_secondary_coord) = col_data(i);
            }
        }
    
        for (int i = 1; i < ind_data.size(); ++i) {
            OPENSIM_THROW_IF(ind_data(i) <= ind_data(i - 1), Exception,
                    "The secondary_coupled_coordinate must increase during the "
                    "sweep simulation to compute the constraint functions. Set "
                    "secondary_coupled_coordinate_stop_value greater than "
                    "secondary_coupled_coordinate_start_value.")
        }
    
        for (int j = 0; j < _n_constrained_coord; ++j) {
            const int last = data.nrow() - 1;
            const double ind_range = ind_data(last) - ind_data(0);
            const double change =
                    data(last, j + _n_secondary_coord) - data(0, j + _n_secondary_coord);
            const double ratio = (ind_range != 0) ? change / ind_range : 0.0;
            log_info("Constrained coordinate {} changed by {} over the sweep, "
                    "{} x the change of {}.",
                    _constrained_coord_path[j], change, ratio,
                    get_secondary_coupled_coordinate());
            if (std::abs(ratio) < 0.25) {
                log_warn("Constrained coordinate {} barely moved with {}. Check "
                        "the ligament slack lengths and the contact geometry "
                        "(set print_secondary_constraint_sim_results to write "
                        "the ligament lengths and strains of the sweep).",
                        _constrained_coord_path[j],
                        get_secondary_coupled_coordinate());
            }
        }
    
        double ind_max = SimTK::max(ind_data);
        double ind_min = SimTK::min(ind_data);
    
        int npts = get_constraint_function_num_interpolation_points();
        OPENSIM_THROW_IF(npts < 2, Exception,
                "constraint_function_num_interpolation_points must be at least 2.")
        double step = (ind_max - ind_min) / (npts - 1);
    
        SimTK::Vector ind_pt_data(npts);
    
        for (int i = 0; i < npts; ++i) {
            ind_pt_data(i) = ind_min + i * step;
        }
    
        _secondary_constraint_functions.clearAndDestroy();
    
        for (int j = 0; j < _n_secondary_coord; ++j) {
            std::string path = _secondary_coord_path[j];
    
            SimTK::Vector secondary_data = data(j);
    
            SimmSpline data_fit = SimmSpline(secondary_data.size(), &ind_data[0], &secondary_data[0]);
            //GCVSpline* spline = new GCVSpline(5, secondary_data.nrow(), &ind_data[0], &secondary_data[0], path, -1);
            //GCVSpline* spline = new GCVSpline();
            //spline->setDegree(5);
            //spline->set
            //SimmSpline* spline = new SimmSpline();
            //spline->setName(path);
    
            SimTK::Vector dep_interp_pts(npts);
    
            for (int i = 0; i < npts; ++i) {
                //spline->addPoint(ind_pt_data(i), data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i))));
                dep_interp_pts(i) = data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i)));
            }
            SimmSpline* spline = new SimmSpline(npts, &ind_pt_data[0], &dep_interp_pts[0],path);
            
            //GCVSpline* spline = new GCVSpline(5, npts, &ind_data[0], &dep_interp_pts[0], path, -1);
    
            _secondary_constraint_functions.adoptAndAppend(spline);
        }
    
        for (int j = 0; j < _n_constrained_coord; ++j) {
            std::string path = _constrained_coord_path[j];
    
             SimTK::Vector secondary_data = data(j + _n_secondary_coord);
    
            SimmSpline data_fit = SimmSpline(
                    secondary_data.size(), &ind_data[0], &secondary_data[0]);
            // GCVSpline* spline = new GCVSpline(5, secondary_data.nrow(),
            // &ind_data[0], &secondary_data[0], path, -1); GCVSpline* spline = new
            // GCVSpline(); spline->setDegree(5); spline->set SimmSpline* spline =
            // new SimmSpline(); spline->setName(path);
    
            SimTK::Vector dep_interp_pts(npts);
    
            for (int i = 0; i < npts; ++i) {
                // spline->addPoint(ind_pt_data(i),
                // data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i))));
                dep_interp_pts(i) =
                        data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i)));
            }
            SimmSpline* spline =
                    new SimmSpline(npts, &ind_pt_data[0], &dep_interp_pts[0], path);
    
            // GCVSpline* spline = new GCVSpline(5, npts, &ind_data[0],
            // &dep_interp_pts[0], path, -1);
    
            _secondary_constraint_functions.adoptAndAppend(spline);
        }
    
        //Print Secondardy Constraint Functions to file
        _secondary_constraint_functions.print(file);
    };

    auto run_sweep = [&](double target_value, double sweep_time,
            bool record) -> TimeSeriesTable {
        //setup quadratic sweep function
        double Vx = 0;
        double Vy = settle_value;
        double Px = Vx + sweep_time;
        double Py = target_value;
        double a = (Py - Vy) / SimTK::square(Px - Vx);

        double C1 = a;
        double C2 = -2 * a * Vx;
        double C3 = a * SimTK::square(Vx) + Vy;

        SimTK::Vector coefficients(3);
        coefficients.set(0, C1);
        coefficients.set(1, C2);
        coefficients.set(2, C3);

        PolynomialFunction sweep_func = PolynomialFunction(coefficients);
        coupled_coord.set_prescribed_function(sweep_func);

        // The window of the sweep (also after the system is rebuilt by a
        // recovery step)
        auto setup_sweep_viz = [&]() {
            if (get_use_visualizer()) {
                SimTK::Visualizer& viz =
                        model.updVisualizer().updSimbodyVisualizer();
                viz.setWindowTitle("Sweeping: " + get_model_file());
                viz.setBackgroundColor(SimTK::White);
                viz.setShowSimTime(true);
                viz.setDesiredFrameRate(100);
                if (get_show_solver_info_in_visualizer()) {
                    // The rows are updated after every output step of the sweep
                    viz.addDecorationGenerator(
                            new SolverInfoDecorator(sweep_info, 2.0 / 3.0));
                }
            }
        };
        auto prescribe_muscles = [&](SimTK::State& s) {
            for (Muscle& msl : model.updComponentList<Muscle>()) {
                msl.overrideActuation(s, true);
                double value = msl.getMaxIsometricForce() *
                        get_secondary_constraint_sim_muscle_activation();
                msl.setOverrideActuation(s, value);
            }
            model.equilibrateMuscles(s);
        };

        state = model.initSystem();
        setup_sweep_viz();

        //prescribe muscle force
        prescribe_muscles(state);

        //set settled secondary coordinate values
        for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
            std::string secondary_coord = _secondary_coord_path[c];
            Coordinate& coord =
                    model.updComponent<Coordinate>(secondary_coord);
            coord.setValue(state, settled_secondary_values(c));
            coord.setSpeedValue(state, settled_secondary_speeds(c));
        }
        for (int c = 0; c < _n_constrained_coord; c++) {
            Coordinate& coord =
                    model.updComponent<Coordinate>(_constrained_coord_path[c]);
            coord.setValue(state, settled_constrained_values(c));
            coord.setSpeedValue(state, settled_constrained_speeds(c));
        }

        const double sweep_start = 0;
        const double sweep_stop = Px;
        const double sweep_dt = get_sweep_dt();
        const int nSteps = (int)lround((sweep_stop - sweep_start) / sweep_dt);

        //Setup storage for computing constraint functions
        TimeSeriesTable q_table;
        SimTK::RowVector q_row(model.getNumCoordinates());
        std::vector<std::string> q_names;

        for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
            q_names.push_back(coord.getAbsolutePathString() + "/value");
        }

        q_table.setColumnLabels(q_names);

        //setup integrator
        const bool recovery = get_secondary_constraint_sim_recovery();
        const double base_accuracy =
                get_secondary_constraint_sim_integrator_accuracy();
        double itg_accuracy = base_accuracy;
        double itg_max_step = SimTK::Infinity;
        double itg_init_step = -1;
        std::unique_ptr<SimTK::CPodesIntegrator> sweep_integrator;
        std::unique_ptr<SimTK::TimeStepper> sweep_timestepper;
        // (Re)starts the integrator from a state of the current system
        auto start_integrator = [&](const SimTK::State& from) {
            sweep_timestepper.reset();
            sweep_integrator.reset(new SimTK::CPodesIntegrator(
                    model.getSystem(), SimTK::CPodes::BDF,
                    SimTK::CPodes::Newton));
            sweep_integrator->setAccuracy(itg_accuracy);
            if (get_secondary_constraint_sim_internal_step_limit() != -1) {
                sweep_integrator->setInternalStepLimit(
                    get_secondary_constraint_sim_internal_step_limit());
            }
            if (itg_max_step < SimTK::Infinity) {
                sweep_integrator->setMaximumStepSize(itg_max_step);
            }
            if (itg_init_step > 0) {
                sweep_integrator->setInitialStepSize(itg_init_step);
            }
            sweep_timestepper.reset(new SimTK::TimeStepper(
                    model.getSystem(), *sweep_integrator));
            sweep_timestepper->initialize(from);
        };
        start_integrator(state);

        // Recovery from an integrator failure (secondary_constraint_sim_recovery)
        //--------------------------------------------------------------------
        // The bodies, dampers and contact meshes that steps B and C change,
        // with their original values
        struct BodyOrig { std::string path; double mass; SimTK::Vec6 inertia; };
        struct ValueOrig { std::string path; double value; };
        std::set<std::string> rec_bodies;
        std::vector<BodyOrig> body_orig;
        std::vector<ValueOrig> damper_orig;
        std::vector<ValueOrig> mesh_orig;
        int cur_b = -1;          // scales of B1 (0) or B2 (1) applied, -1 none
        bool cur_soft = false;   // softer contact (step C) applied
        int active_rung = -1;    // recovery step in use, -1 none
        int hold_left = 0;
        static const char* rung_names[] = {"A1 restarted integrator",
                "A2 small step size", "A3 looser accuracy",
                "B1 heavier patella", "B2 heavier patella (more)",
                "C softer patella contact", "D quasi-static settle"};
        const int n_rungs = 7;

        if (recovery) {
            if (getProperty_recovery_bodies().size() > 0) {
                for (int k = 0; k < getProperty_recovery_bodies().size(); ++k) {
                    std::string p = get_recovery_bodies(k);
                    if (p.empty() || p[0] != '/') p = "/bodyset/" + p;
                    rec_bodies.insert(p);
                }
            } else {
                auto add_body_of = [&](const std::string& path) {
                    const Coordinate& c = model.getComponent<Coordinate>(path);
                    const Joint& jt = c.getJoint();
                    const Body* b = dynamic_cast<const Body*>(
                            &jt.getChildFrame().findBaseFrame());
                    if (!b) return;
                    // the patella and the menisci, not the tibia
                    std::string lower = b->getName();
                    std::transform(lower.begin(), lower.end(), lower.begin(),
                            [](unsigned char ch) { return std::tolower(ch); });
                    if (lower.find("patella") != std::string::npos ||
                            lower.find("menisc") != std::string::npos) {
                        rec_bodies.insert(b->getAbsolutePathString());
                    }
                };
                for (int c = 0; c < _secondary_coord_path.getSize(); ++c) {
                    add_body_of(_secondary_coord_path[c]);
                }
                for (int c = 0; c < _n_constrained_coord; ++c) {
                    add_body_of(_constrained_coord_path[c]);
                }
            }
            for (const std::string& p : rec_bodies) {
                const Body& body = model.getComponent<Body>(p);
                body_orig.push_back({p, body.get_mass(), body.get_inertia()});
            }
            for (const SpringGeneralizedForce& sgf :
                    model.getComponentList<SpringGeneralizedForce>()) {
                const std::string cname = sgf.get_coordinate();
                if (!model.getCoordinateSet().contains(cname)) continue;
                const Body* b = dynamic_cast<const Body*>(
                        &model.getCoordinateSet().get(cname).getJoint()
                                .getChildFrame().findBaseFrame());
                if (b && rec_bodies.count(b->getAbsolutePathString())) {
                    damper_orig.push_back(
                            {sgf.getAbsolutePathString(), sgf.get_viscosity()});
                }
            }
            for (const Smith2018ContactMesh& m :
                    model.getComponentList<Smith2018ContactMesh>()) {
                if (rec_bodies.count(m.getMeshFrame().findBaseFrame()
                        .getAbsolutePathString())) {
                    mesh_orig.push_back(
                            {m.getAbsolutePathString(), m.get_elastic_modulus()});
                }
            }
            log_info("Sweep recovery is on: {} bodies, {} dampers and {} "
                    "contact meshes can be changed.", (int)body_orig.size(),
                    (int)damper_orig.size(), (int)mesh_orig.size());
        }

        // Sets the properties of steps B (scales b = 0 or 1, -1 original) and
        // C (soft). The system has to be rebuilt after.
        auto set_mods = [&](int b, bool soft) {
            const double ds = b >= 0 ? get_recovery_damper_scales(b) : 1.0;
            for (const BodyOrig& o : body_orig) {
                Body& body = model.updComponent<Body>(o.path);
                body.set_mass(o.mass *
                        (b >= 0 ? get_recovery_mass_scales(b) : 1.0));
                body.set_inertia(o.inertia *
                        (b >= 0 ? get_recovery_inertia_scales(b) : 1.0));
            }
            for (const ValueOrig& o : damper_orig) {
                model.updComponent<SpringGeneralizedForce>(o.path)
                        .set_viscosity(o.value * ds);
            }
            for (const ValueOrig& o : mesh_orig) {
                model.updComponent<Smith2018ContactMesh>(o.path)
                        .set_elastic_modulus(o.value *
                                (soft ? get_recovery_contact_modulus_scale()
                                        : 1.0));
            }
            cur_b = b;
            cur_soft = soft;
        };

        // Rebuilds the system (after properties changed) and returns the
        // state of the new system with the time and the continuous states
        // of old
        auto rebuild_from = [&](const SimTK::State& old) -> SimTK::State {
            sweep_timestepper.reset();
            sweep_integrator.reset();
            visualizer_closer.closeLater(model, "Sweeping: " + get_model_file());
            SimTK::State s = model.initSystem();
            setup_sweep_viz();
            prescribe_muscles(s);
            OPENSIM_THROW_IF(s.getNY() != old.getNY(), Exception,
                    "The recovery of the sweep simulation changed the number "
                    "of states.")
            s.updY() = old.getY();
            s.setTime(old.getTime());
            return s;
        };

        // Integrator settings and scales of a recovery step (-1: normal)
        auto configure = [&](int rung, int& b, bool& soft) {
            itg_accuracy = base_accuracy;
            itg_max_step = SimTK::Infinity;
            itg_init_step = -1;
            b = -1;
            soft = false;
            if (rung >= 1) {
                itg_max_step = sweep_dt / 10;
                itg_init_step = sweep_dt * 1e-3;
            }
            if (rung >= 2) {
                itg_accuracy = std::max(base_accuracy,
                        std::min(10 * base_accuracy, 1e-2));
            }
            if (rung == 3) b = 0;
            if (rung >= 4) b = 1;
            if (rung == 5) soft = true;
        };

        // Step D: the coupled coordinate is held at the next pose, the
        // model settles there (like the settling simulation), then the
        // sweep goes on from that state at t_target.
        auto hop = [&](const SimTK::State& from, double t_target) {
            const double q_target =
                    sweep_func.calcValue(SimTK::Vector(1, t_target));
            coupled_coord.set_prescribed_function(Constant(q_target));
            SimTK::State s = rebuild_from(from);
            s.updU() = 0;
            start_integrator(s);

            std::vector<std::string> paths;
            for (int k = 0; k < _n_secondary_coord; ++k) {
                paths.push_back(_secondary_coord_path[k]);
            }
            for (int k = 0; k < _n_constrained_coord; ++k) {
                paths.push_back(_constrained_coord_path[k]);
            }
            std::vector<double> prev(paths.size());
            for (size_t k = 0; k < paths.size(); ++k) {
                prev[k] = model.getComponent<Coordinate>(paths[k]).getValue(s);
            }
            SettleMonitor monitor(
                    get_secondary_constraint_sim_settle_threshold(),
                    get_secondary_constraint_sim_settle_plateau_tolerance(),
                    get_secondary_constraint_sim_settle_plateau_window());
            const double dt = get_settling_sim_dt();
            const int max_j = std::max(1,
                    (int)std::ceil(get_recovery_hop_max_time() / dt));
            const double t0 = s.getTime();
            bool settled = false;
            for (int j = 1; !settled && j <= max_j; ++j) {
                sweep_timestepper->stepTo(t0 + j * dt);
                const SimTK::State& cs = sweep_timestepper->getState();
                double max_delta = 0;
                for (size_t k = 0; k < paths.size(); ++k) {
                    const double v =
                            model.getComponent<Coordinate>(paths[k]).getValue(cs);
                    max_delta = std::max(max_delta, std::abs(v - prev[k]));
                    prev[k] = v;
                }
                settled = monitor.update(max_delta);
            }
            const SimTK::State settled_state = sweep_timestepper->getState();
            coupled_coord.set_prescribed_function(sweep_func);
            SimTK::State s2 = rebuild_from(settled_state);
            s2.updU() = 0;
            s2.setTime(t_target);
            start_integrator(s2);
        };

        // Advances the sweep to t_target. Returns true if a recovery step was
        // needed. Throws if the integrator fails and the recovery is off or
        // every step of it fails.
        auto advance = [&](double t_target) -> bool {
            std::exception_ptr first_error;
            std::string first_what;
            try {
                sweep_timestepper->stepTo(t_target);
                return false;
            }
            catch (const std::exception& e) {
                first_error = std::current_exception();
                first_what = e.what();
                // Keep the last output state and the state at which the
                // integrator failed, to find out what the contact or ligament
                // forces do there.
                try {
                    StatesTrajectory failed;
                    failed.append(state);
                    failed.append(sweep_timestepper->getState());
                    const std::string path = get_results_directory() +
                            "/sweep_integrator_failure_states.sto";
                    STOFileAdapter::write(failed.exportToTable(model), path);
                    log_error("The sweep integrator failed at t = {} ({}). "
                              "The last output state and the state at the "
                              "failure are in {}.",
                              sweep_timestepper->getState().getTime(),
                              first_what, path);
                }
                catch (...) {}
                if (!recovery) throw;
            }

            // Recovery
            const SimTK::State last_good = state;
            bool rebuilt = false; // last_good is then of the previous system
            // Two passes of the steps. Pass 0: from the step after the one
            // in use (from A1 if none is). Pass 1 (only if a change of the
            // bodies, dampers or contact is on): all steps again, with these
            // changes kept, and step D with the softer contact.
            const int keep_b = cur_b;
            const bool keep_soft = cur_soft;
            const int first_rung = active_rung >= 1 ? active_rung + 1 : 0;
            for (int pass = 0; pass < 2; ++pass) {
              if (pass == 1 && keep_b == -1 && !keep_soft) break;
              for (int rung = pass == 0 ? first_rung : 0; rung < n_rungs;
                      ++rung) {
                log_warn("Sweep recovery at t = {}: trying step {}{}.",
                        last_good.getTime(), rung_names[rung],
                        pass == 1 ? " (changes kept)" : "");
                try {
                    int b; bool soft;
                    configure(rung == 6 ? (pass == 1 ? 5 : 4) : rung, b, soft);
                    if (pass == 1) {
                        b = std::max(b, keep_b);
                        soft = soft || keep_soft;
                    }
                    SimTK::State s = last_good;
                    if (b != cur_b || soft != cur_soft || rebuilt) {
                        set_mods(b, soft);
                        rebuilt = true;
                        s = rebuild_from(last_good);
                    }
                    if (rung == 6) {
                        hop(s, t_target);
                    }
                    else {
                        start_integrator(s);
                        sweep_timestepper->stepTo(t_target);
                    }
                    active_rung = (rung >= 1 || cur_b != -1 || cur_soft)
                            ? std::max(rung, 1) : -1;
                    hold_left = get_recovery_hold_outputs();
                    log_warn("Sweep recovery step {} worked at t = {}.",
                            rung_names[rung], t_target);
                    return true;
                }
                catch (const std::exception& e) {
                    log_warn("Sweep recovery step {} failed: {}",
                            rung_names[rung], e.what());
                }
              }
            }

            // E: nothing worked. Keep what is done.
            try {
                if (q_table.getNumRows() > 0) {
                    const std::string path = get_results_directory() +
                            "/sweep_partial_coordinates.sto";
                    STOFileAdapter::write(q_table, path);
                    double reached = q_table.getDependentColumn(
                            get_secondary_coupled_coordinate() + "/value")
                            (int(q_table.getNumRows()) - 1);
                    if (coupled_coord.getMotionType() ==
                            Coordinate::Rotational) {
                        reached = SimTK::convertRadiansToDegrees(reached);
                    }
                    log_error("Every sweep recovery step failed at t = {}. "
                              "The {} outputs done (up to {} of {}) are in "
                              "{}.", t_target, (int)q_table.getNumRows(),
                              reached, get_secondary_coupled_coordinate(),
                              path);

                    // The constraint functions up to the angle reached, in
                    // a file of their own (not the usual file name, so a
                    // partial result is not taken for a complete one).
                    // Only for the sweep up, which starts at the settled
                    // pose; they are valid up to the angle reached.
                    if (record && q_table.getNumRows() > 2) {
                        std::string fn = get_secondary_constraint_function_file();
                        const size_t dot = fn.rfind('.');
                        fn = (dot == std::string::npos) ? fn + "_partial"
                                : fn.substr(0, dot) + "_partial" + fn.substr(dot);
                        fit_constraint_functions(q_table, fn);
                        log_error("The constraint functions up to {} of {} "
                                  "are in {}. They are valid only up to "
                                  "that angle.", reached,
                                  get_secondary_coupled_coordinate(), fn);
                    }
                }
            }
            catch (const std::exception& e) {
                log_warn("The partial constraint functions were not "
                         "written: {}", e.what());
            }
            std::rethrow_exception(first_error);
        };

        for (int i = 0; i <= nSteps; ++i) {

            visualizer_closer.closeDue();
            const bool recovered = advance(sweep_start + i*sweep_dt);
            state = sweep_timestepper->getState();

            // Information as text in the window of the sweep
            if (get_use_visualizer() && get_show_solver_info_in_visualizer()) {
                const SimTK::Integrator& itg = sweep_timestepper->getIntegrator();
                const bool rotational = coupled_coord.getMotionType() ==
                        Coordinate::Rotational;
                sweep_info->rows.clear();
                addInfoRow(*sweep_info, "Sweep", "%s %.1f %s",
                        record ? "up to" : "down to",
                        rotational ? SimTK::convertRadiansToDegrees(
                                target_value) : target_value,
                        rotational ? "deg" : "m");
                addInfoRow(*sweep_info, "Time", "%.3f / %.3f s",
                        state.getTime(), Px);
                addInfoRow(*sweep_info, coupled_coord.getName(), "%.2f %s",
                        rotational ? SimTK::convertRadiansToDegrees(
                                coupled_coord.getValue(state))
                                : coupled_coord.getValue(state),
                        rotational ? "deg" : "m");
                addInfoRow(*sweep_info, "Time step", "%.2e s",
                        itg.getPreviousStepSizeTaken());
                addInfoRow(*sweep_info, "Steps", "%d (%d error test failures)",
                        itg.getNumStepsTaken(), itg.getNumErrorTestFailures());
                if (active_rung >= 1) {
                    addInfoRow(*sweep_info, "Recovery", "%s",
                            rung_names[active_rung]);
                }
                // contact forces per knee (see VisualizerInfo.h)
                if (!sweep_contact_failed) {
                    model.realizeDynamics(state);
                    updateKneeForces(model, state, sweep_knees,
                            sweep_contact_failed);
                    addKneeForceRows(sweep_knees, *sweep_info);
                }
            }

            if (record && get_print_secondary_constraint_sim_results()) {
                sweep_states.append(state);
            }

            int j = 0;
            for (const auto& coord : model.getComponentList<Coordinate>()) {
                q_row(j) = coord.getValue(state);
                j++;
            }
            q_table.appendRow(state.getTime(), q_row);

            if (record && !sweep_ligaments.empty()) {
                model.realizeDynamics(state);
                SimTK::RowVector lig_row(n_lig_columns);
                int c = 0;
                lig_row(c++) = model.getComponent<Coordinate>(
                        get_secondary_coupled_coordinate()).getValue(state);
                for (int j = 0; j < _n_constrained_coord; ++j) {
                    lig_row(c++) = model.getComponent<Coordinate>(
                            _constrained_coord_path[j]).getValue(state);
                }
                for (const auto* lig : sweep_ligaments) {
                    lig_row(c++) = lig->getOutputValue<double>(state, "length");
                    lig_row(c++) = lig->getOutputValue<double>(state, "strain");
                    lig_row(c++) =
                            lig->getOutputValue<double>(state, "total_force");
                }
                lig_table.appendRow(state.getTime(), lig_row);
            }

            log_info("{}",state.getTime());
            if (get_verbose() > 0) printDebugInfo(model, state);

            // The recovery change is undone after recovery_hold_outputs
            // outputs without a failure.
            if (recovery && active_rung >= 1 && !recovered &&
                    --hold_left <= 0) {
                log_info("Sweep recovery: back to the normal settings at "
                        "t = {}.", state.getTime());
                int b; bool soft;
                configure(-1, b, soft);
                SimTK::State s = state;
                if (cur_b != -1 || cur_soft) {
                    set_mods(-1, false);
                    s = rebuild_from(state);
                }
                start_integrator(s);
                state = sweep_timestepper->getState();
                active_rung = -1;
            }
        }
        // the properties are back to the original values for the next sweep
        if (recovery && (cur_b != -1 || cur_soft)) {
            set_mods(-1, false);
        }
        // this sweep is done
        visualizer_closer.closeLater(model, "Sweeping: " + get_model_file());
        return q_table;
    };

    TimeSeriesTable q_table = run_sweep(stop_value,
            get_secondary_constraint_sim_sweep_time(), true);

    // Second sweep, from the same settled state (0) down to min (e.g. knee
    // hyperextension). It is the same quadratic function of time as the sweep
    // up, with the same acceleration of the coupled coordinate (so the same
    // speed at the same distance from 0), starting at rest at 0 like it. The
    // rows of both are joined in order of increasing coupled coordinate, so
    // the function is one curve from min to max and the part above 0 is the
    // same as without the second sweep.
    if (start_value < settle_value) {
        const double down_time = get_secondary_constraint_sim_sweep_time() *
                std::sqrt((settle_value - start_value) /
                          (stop_value - settle_value));
        log_info("Starting Sweep Simulation down to {} ({} s).", start_value,
                down_time);
        TimeSeriesTable q_down = run_sweep(start_value, down_time, false);

        TimeSeriesTable joined;
        joined.setColumnLabels(q_table.getColumnLabels());
        const double join_dt = get_sweep_dt();
        double t = 0;
        // the first row of each sweep is the settled value
        for (int r = (int)q_down.getNumRows() - 1; r >= 1; --r) {
            joined.appendRow(t, q_down.getRowAtIndex(r));
            t += join_dt;
        }
        for (int r = 0; r < (int)q_table.getNumRows(); ++r) {
            joined.appendRow(t, q_table.getRowAtIndex(r));
            t += join_dt;
        }
        q_table = joined;
    }

    fit_constraint_functions(q_table, get_secondary_constraint_function_file());

    //Write Outputs
    if (get_print_secondary_constraint_sim_results()) {
        log_info("Printing secondary constraint simulation results: {}",
            get_results_directory());

        std::string name = "secondary_constraint_sim_states";

        TimeSeriesTable settle_table = settle_states.exportToTable(model);
        settle_table.addTableMetaData("header", name);
        settle_table.addTableMetaData("nRows", 
            std::to_string(settle_table.getNumRows()));
        settle_table.addTableMetaData("nColumns", 
            std::to_string(settle_table.getNumColumns() + 1));

        TimeSeriesTable sweep_table = sweep_states.exportToTable(model);
        sweep_table.addTableMetaData("header", name);
        sweep_table.addTableMetaData("nRows", 
            std::to_string(sweep_table.getNumRows()));
        sweep_table.addTableMetaData("nColumns", 
            std::to_string(sweep_table.getNumColumns() + 1));

        std::string settle_file =
            get_results_directory() + "/" + get_results_prefix() +
            "_secondary_constraint_settle_states.sto";

        std::string sweep_file =
            get_results_directory() + "/" + get_results_prefix() +
            "_secondary_constraint_sweep_states.sto";

        STOFileAdapter sto_file_adapt;
        sto_file_adapt.write(settle_table, settle_file);
        sto_file_adapt.write(sweep_table, sweep_file);

        if (lig_table.getNumRows() > 0) {
            lig_table.addTableMetaData("header", std::string("sweep_ligaments"));
            lig_table.addTableMetaData("nRows",
                    std::to_string(lig_table.getNumRows()));
            lig_table.addTableMetaData("nColumns",
                    std::to_string(lig_table.getNumColumns() + 1));
            lig_table.addTableMetaData("inDegrees", std::string("no"));
            sto_file_adapt.write(lig_table,
                    get_results_directory() + "/" + get_results_prefix() +
                            "_secondary_constraint_sweep_ligaments.sto");
        }
    }

    if (!get_constrained_model_file().empty()) {
        Model model_cons = _model;
        model_cons.setUseVisualizer(false);
        model_cons.initSystem();
        // Add CoordinateCouplerConstraints for Secondary Kinematics.
        SimTK::Vector coupled_coord_default_value = SimTK::Vector(
                1, model_cons.getComponent<Coordinate>(
                                get_secondary_coupled_coordinate())
                           .getDefaultValue());

        for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
            std::string path = get_secondary_coordinates(i);
            Coordinate& coord = model_cons.updComponent<Coordinate>(path);
            std::string coord_name = coord.getName();

            std::string ind_coord_name =
                    model_cons.getComponent<Coordinate>(
                                 get_secondary_coupled_coordinate())
                            .getName();

            std::string joint_path = coord.getJoint().getAbsolutePathString();

            const Function& function =
                    _secondary_constraint_functions.get(path);
            CoordinateCouplerConstraint* cc_constraint =
                    new CoordinateCouplerConstraint();

            cc_constraint->setIndependentCoordinateNames(
                    Array<std::string>(ind_coord_name, 1, 2));
            cc_constraint->setDependentCoordinateName(coord_name);
            cc_constraint->setFunction(function);
            cc_constraint->setName(coord_name + "_function");

            coord.setDefaultValue(
                    function.calcValue(coupled_coord_default_value));

            model_cons.addConstraint(cc_constraint);
        }
        // Add CoordinateCouplerConstraints for constrained coordinate
        for (int i = 0; i < getProperty_constrained_coordinates().size(); ++i) {
            std::string path = get_constrained_coordinates(i);
            Coordinate& coord = model_cons.updComponent<Coordinate>(path);
            std::string coord_name = coord.getName();

            std::string ind_coord_name =
                    model_cons
                            .getComponent<Coordinate>(
                                    get_secondary_coupled_coordinate())
                            .getName();

            std::string joint_path = coord.getJoint().getAbsolutePathString();

            const Function& function =
                    _secondary_constraint_functions.get(path);
            CoordinateCouplerConstraint* cc_constraint =
                    new CoordinateCouplerConstraint();

            cc_constraint->setIndependentCoordinateNames(
                    Array<std::string>(ind_coord_name, 1, 2));
            cc_constraint->setDependentCoordinateName(coord_name);
            cc_constraint->setFunction(function);
            cc_constraint->setName(coord_name + "_function");

            coord.setDefaultValue(
                    function.calcValue(coupled_coord_default_value));

            model_cons.addConstraint(cc_constraint);
        }

        for (int i = model_cons.upd_ForceSet().getSize() - 1; i >= 0; i--) {
            if (model_cons.getForceSet().get(i).getConcreteClassName() ==
                    "Blankevoort1991Ligament") {
                std::string lg_name = model_cons.getForceSet().get(i).getName();
                if (lg_name[0] == 'P' && lg_name[1] == 'T') {

                } else {
                    model_cons.upd_ForceSet().remove(i);
                }

            } else if (model_cons.getForceSet().get(i).getConcreteClassName() ==
                       "SpringGeneralizedForce") {
                model_cons.upd_ForceSet().remove(i);

            } else if (model_cons.getForceSet().get(i).getConcreteClassName() ==
                       "Smith2018ArticularContactForce") {
                model_cons.upd_ForceSet().remove(i);
            }

        }
        model_cons.updContactGeometrySet().clearAndDestroy();
        model_cons.print(get_constrained_model_file());
    }

    // auto_close_visualizer: the remaining time of the last windows
    visualizer_closer.waitAndCloseAll();
}

void COMAKInverseKinematicsTool::performIK()
{
    Model model = _model;
    model.setUseVisualizer(false);
    model.initSystem();

    // Check that IK tasks exist as markers

    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        std::string iktask_name = get_IKTaskSet().get(i).getName();

        if (get_IKTaskSet().get(i).getConcreteClassName() == "IKMarkerTask"){
            try {
                model.getMarkerSet().get(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKMarkerTask: " + iktask_name + 
                    " does not exist as a marker in the model.");
            }
            try {
                TimeSeriesTableVec3 trc = TimeSeriesTableVec3(get_marker_file());
                trc.getColumnIndex(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKMarkerTask: " + iktask_name + 
                    " does not exist as a marker in the .trc marker_file: " +
                    get_marker_file());
            }
            
        }
        if (get_IKTaskSet().get(i).getConcreteClassName() == "IKCoordinateTask"){
            try {
                model.getCoordinateSet().get(iktask_name);
            }
            catch (Exception const&) {
                OPENSIM_THROW(Exception, "COMAKInverseKinematics "
                    "IKCoordinateTask: " + iktask_name + 
                    " does not exist as a coordinate in the model.");
            }
        }
    }

    try {
        _secondary_constraint_functions =
            FunctionSet(get_secondary_constraint_function_file());
    }
    catch (Exception const&) {
        OPENSIM_THROW(Exception, "Function file: " +
            get_secondary_constraint_function_file() + " does not exist.");

    }

    SimTK::Vector coupled_coord_default_value = SimTK::Vector(1,
        model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getDefaultValue());

    const std::string& secondary_coupled_coord_name = 
        model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getName();

   //Add CoordinateCouplerConstraints for Secondary Kinematics
   for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
        std::string path = get_secondary_coordinates(i);
        Coordinate& coord = model.updComponent<Coordinate>(path);
        std::string coord_name = coord.getName();

        std::string ind_coord_name = model.getComponent<Coordinate>(
            get_secondary_coupled_coordinate()).getName();

        std::string joint_path = coord.getJoint().getAbsolutePathString();
        
        const Function& function = _secondary_constraint_functions.get(path);
        CoordinateCouplerConstraint* cc_constraint = new CoordinateCouplerConstraint();

        cc_constraint->setIndependentCoordinateNames(
            Array<std::string>(ind_coord_name, 1, 2));
        cc_constraint->setDependentCoordinateName(coord_name);
        cc_constraint->setFunction(function);
        cc_constraint->setName(coord_name + "_function");

        coord.setDefaultValue(function.calcValue(coupled_coord_default_value));

        model.addConstraint(cc_constraint);
   }
   // Add CoordinateCouplerConstraints for constrained Kinematics
   for (int i = 0; i < getProperty_constrained_coordinates().size(); ++i) {
       std::string path = get_constrained_coordinates(i);
       Coordinate& coord = model.updComponent<Coordinate>(path);
       std::string coord_name = coord.getName();

       std::string ind_coord_name =
               model.getComponent<Coordinate>(
                            get_secondary_coupled_coordinate())
                       .getName();

       std::string joint_path = coord.getJoint().getAbsolutePathString();

       const Function& function = _secondary_constraint_functions.get(path);
       CoordinateCouplerConstraint* cc_constraint =
               new CoordinateCouplerConstraint();

       cc_constraint->setIndependentCoordinateNames(
               Array<std::string>(ind_coord_name, 1, 2));
       cc_constraint->setDependentCoordinateName(coord_name);
       cc_constraint->setFunction(function);
       cc_constraint->setName(coord_name + "_function");

       coord.setDefaultValue(function.calcValue(coupled_coord_default_value));

       model.addConstraint(cc_constraint);
   }

    //Set coordinate types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(false);
            coord.set_prescribed(false);
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            //coord.set_clamped(true);
        }
    }

    for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
        coord.set_locked(false);
        coord.set_clamped(false);
    }

    SimTK::State state = model.initSystem();

    //if (!get_constrained_model_file().empty()) {
    //    model.print(get_constrained_model_file());
    //}

    runInverseKinematics(model);
}

void COMAKInverseKinematicsTool::runInverseKinematics(Model& model) {

     Kinematics* kinematicsReporter = nullptr;

    try{
        // although newly loaded model will be finalized
        // there is no guarantee that the model has not been edited/modified
        model.finalizeFromProperties();
        model.printBasicInfo();

        // Define reporter for output
        kinematicsReporter = new Kinematics();
        kinematicsReporter->setRecordAccelerations(false);
        kinematicsReporter->setInDegrees(true);
        model.addAnalysis(kinematicsReporter);

        log_info("Running Inverse Kinematics\n");

        // Initialize the model's underlying system and get its default state.
        // No window for the inverse kinematics itself: the solver does not
        // draw anything in it (it stayed blank and open). The windows are
        // those of the settling and sweep simulations.
        model.setUseVisualizer(false);
        SimTK::State& s = model.initSystem();

        //Convert old Tasks to references for assembly and tracking
        MarkersReference markersReference;
        SimTK::Array_<CoordinateReference> coordinateReferences;
        // populate the references according to the setting of this Tool
        populateReferences(model, markersReference, coordinateReferences);

        // Determine the start time, if the provided time range is not 
        // specified then use time from marker reference.
        // Adjust the time range for the tool if the provided range exceeds
        // that of the marker data.
        SimTK::Vec2 markersValidTimeRange = markersReference.getValidTimeRange();
        double start_time = (markersValidTimeRange[0] > get_time_range(0)) ?
            markersValidTimeRange[0] : get_time_range(0);
        double final_time = (markersValidTimeRange[1] < get_time_range(1)) ?
            markersValidTimeRange[1] : get_time_range(1);

        SimTK_ASSERT2_ALWAYS(final_time >= start_time,
            "InverseKinematicsTool final time (%f) is before start time (%f).",
            final_time, start_time);

        const auto& markersTable = markersReference.getMarkerTable();
        const int start_ix = int(
            markersTable.getNearestRowIndexForTime(start_time) );
        const int final_ix = int(
            markersTable.getNearestRowIndexForTime(final_time) );
        const int Nframes = final_ix - start_ix + 1;
        const auto& times = markersTable.getIndependentColumn();

        // create the solver given the input data
        InverseKinematicsSolver ikSolver(model, markersReference,
            coordinateReferences, get_ik_constraint_weight());
        ikSolver.setAccuracy(get_ik_accuracy());
        s.updTime() = times[start_ix];

        // If the assembly fails, it is tried again with a looser accuracy,
        // up to 1e-7, before it is an error.
        bool assembled = false;
        std::string assembly_error;
        double accuracy = get_ik_accuracy();
        for (int i = 0; i < 7 && !assembled; ++i) {
            ikSolver.setAccuracy(accuracy);
            try {
                ikSolver.assemble(s);
                assembled = true;
            }
            catch (std::exception const&) {
                try {
                    ikSolver.track(s);
                    assembled = true;
                }
                catch (std::exception const& ex) {
                    assembly_error = ex.what();
                    if (accuracy >= 1e-7) { break; }
                    accuracy = std::min(accuracy * 10, 1e-7);
                    log_warn("Assembly failed, retrying with accuracy {}.",
                            accuracy);
                }
            }
        }
        if (!assembled) {
            throw Exception("The model could not be assembled at the first "
                    "frame (" + std::to_string(times[start_ix]) + " s), also "
                    "not with an accuracy of 1e-7: " + assembly_error +
                    " Check the markers and the coordinates file at this "
                    "time.");
        }
        // setAccuracy() would undo the assembly, so a relaxed accuracy is kept
        // for the tracking
        if (accuracy != get_ik_accuracy()) {
            log_warn("The model was assembled with a relaxed accuracy of {} "
                    "(ik_accuracy is {}), which is used for all frames.",
                    accuracy, get_ik_accuracy());
        }

        kinematicsReporter->begin(s);

        AnalysisSet& analysisSet = model.updAnalysisSet();
        analysisSet.begin(s);
        // Get the actual number of markers the Solver is using, which
        // can be fewer than the number of references if there isn't a
        // corresponding model marker for each reference.
        int nm = ikSolver.getNumMarkersInUse();
        SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
        SimTK::Array_<SimTK::Vec3> markerLocations(nm, SimTK::Vec3(0));
        
        Storage *modelMarkerLocations = get_report_marker_locations() ?
            new Storage(Nframes, "ModelMarkerLocations") : nullptr;
        Storage *modelMarkerErrors = get_report_errors() ? 
            new Storage(Nframes, "ModelMarkerErrors") : nullptr;

        Stopwatch watch;

        for (int i = start_ix; i <= final_ix; ++i) {
            s.updTime() = times[i];
            ikSolver.track(s);
            // show progress line every 1000 frames so users see progress
            if (std::remainder(i - start_ix, 1000) == 0 && i != start_ix)
                std::cout << "Solved " << i - start_ix << " frames..." << std::endl;
            if(get_report_errors()){
                Array<double> markerErrors(0.0, 3);
                double totalSquaredMarkerError = 0.0;
                double maxSquaredMarkerError = 0.0;
                int worst = -1;

                ikSolver.computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
                for(int j=0; j<nm; ++j){
                    totalSquaredMarkerError += squaredMarkerErrors[j];
                    if(squaredMarkerErrors[j] > maxSquaredMarkerError){
                        maxSquaredMarkerError = squaredMarkerErrors[j];
                        worst = j;
                    }
                }

                double rms = nm > 0 ? sqrt(totalSquaredMarkerError / nm) : 0;
                markerErrors.set(0, totalSquaredMarkerError); 
                markerErrors.set(1, rms);
                markerErrors.set(2, sqrt(maxSquaredMarkerError));
                modelMarkerErrors->append(s.getTime(), 3, &markerErrors[0]);

                std::cout << "Frame " << i << " (t=" << s.getTime() << "):\t"
                    << "total squared error = " << totalSquaredMarkerError
                    << ", marker error: RMS=" << rms << ", max="
                    << sqrt(maxSquaredMarkerError) << " (" 
                    << ikSolver.getMarkerNameForIndex(worst) << ")" << std::endl;
            }

            if(get_report_marker_locations()){
                ikSolver.computeCurrentMarkerLocations(markerLocations);
                Array<double> locations(0.0, 3*nm);
                for(int j=0; j<nm; ++j){
                    for(int k=0; k<3; ++k)
                        locations.set(3*j+k, markerLocations[j][k]);
                }

                modelMarkerLocations->append(s.getTime(), 3*nm, &locations[0]);

            }

            kinematicsReporter->step(s, i);
            analysisSet.step(s, i);
        }

        if (get_smooth_ik_second_pass()) {
            // no filter frequency set (-1): 40 Hz
            const double cutoff = (get_smooth_ik_lowpass_frequency() > 0) ?
                    get_smooth_ik_lowpass_frequency() : 40.0;
            log_info("Second pass of the inverse kinematics: coordinates "
                     "smoothed at {} Hz, weight {}",
                    cutoff, get_smooth_ik_weight());

            // the first pass, written with _pass1 in its name
            Storage* pass1 = kinematicsReporter->getPositionStorage();
            if (get_output_motion_file() != "" &&
                    get_output_motion_file() != "Unassigned") {
                std::string name1 = get_output_motion_file();
                const size_t dot = name1.find_last_of('.');
                name1 = (dot == std::string::npos)
                        ? name1 + "_pass1"
                        : name1.substr(0, dot) + "_pass1" + name1.substr(dot);
                IO::makeDir(get_results_directory());
                pass1->print(get_results_directory() + "/" + name1);
            }

            // smoothed coordinates of the first pass (radians)
            Storage smooth(*pass1);
            if (smooth.isInDegrees()) {
                model.getSimbodyEngine().convertDegreesToRadians(smooth);
            }
            smooth.pad(smooth.getSize() / 2);
            smooth.lowpassIIR(cutoff);
            GCVSplineSet smoothed_splines(5, &smooth);

            // soft references for the free coordinates that have no task
            SimTK::Array_<CoordinateReference> references2 =
                    coordinateReferences;
            int n_added = 0;
            for (const Coordinate& coord :
                    model.getComponentList<Coordinate>()) {
                const std::string& name = coord.getName();
                bool has_task = false;
                for (const auto& ref : coordinateReferences) {
                    if (ref.getName() == name) { has_task = true; }
                }
                const int index = smoothed_splines.getIndex(name);
                if (has_task || index < 0 || coord.getLocked(s) ||
                        coord.isConstrained(s)) {
                    continue;
                }
                CoordinateReference ref(name, smoothed_splines.get(index));
                ref.setWeight(get_smooth_ik_weight());
                references2.push_back(ref);
                ++n_added;
            }
            log_info("The second pass has {} soft coordinate references.",
                    n_added);

            InverseKinematicsSolver ikSolver2(model, markersReference,
                    references2, get_ik_constraint_weight());
            ikSolver2.setAccuracy(get_ik_accuracy());
            s.updTime() = times[start_ix];
            {
                // relaxed up to 1e-7 if the assembly fails, like the first pass
                double accuracy2 = get_ik_accuracy();
                bool assembled2 = false;
                while (!assembled2) {
                    ikSolver2.setAccuracy(accuracy2);
                    try {
                        ikSolver2.assemble(s);
                        assembled2 = true;
                    }
                    catch (std::exception const&) {
                        if (accuracy2 >= 1e-7) { throw; }
                        accuracy2 = std::min(accuracy2 * 10, 1e-7);
                        log_warn("Assembly of the second pass failed, "
                                "retrying with accuracy {}.", accuracy2);
                    }
                }
            }

            Kinematics* kinematicsReporter2 = new Kinematics();
            kinematicsReporter2->setRecordAccelerations(false);
            kinematicsReporter2->setInDegrees(true);
            model.addAnalysis(kinematicsReporter2);
            // added after initSystem(), so it is not connected to the model yet
            kinematicsReporter2->setModel(model);
            kinematicsReporter2->begin(s);

            Storage* modelMarkerErrors2 = get_report_errors() ?
                    new Storage(Nframes, "ModelMarkerErrors") : nullptr;
            SimTK::Array_<double> squaredErrors2(nm, 0.0);
            for (int i = start_ix; i <= final_ix; ++i) {
                s.updTime() = times[i];
                ikSolver2.track(s);
                if (modelMarkerErrors2) {
                    double total = 0.0;
                    double max_squared = 0.0;
                    ikSolver2.computeCurrentSquaredMarkerErrors(squaredErrors2);
                    for (int j = 0; j < nm; ++j) {
                        total += squaredErrors2[j];
                        max_squared = std::max(max_squared, squaredErrors2[j]);
                    }
                    Array<double> errors(0.0, 3);
                    errors.set(0, total);
                    errors.set(1, nm > 0 ? sqrt(total / nm) : 0);
                    errors.set(2, sqrt(max_squared));
                    modelMarkerErrors2->append(s.getTime(), 3, &errors[0]);
                }
                kinematicsReporter2->step(s, i);
            }

            // the second pass replaces the first one in the results
            model.removeAnalysis(kinematicsReporter);
            kinematicsReporter = kinematicsReporter2;
            if (modelMarkerErrors2) {
                delete modelMarkerErrors;
                modelMarkerErrors = modelMarkerErrors2;
            }
        }

        // the constraint functions are only computed over the range of the
        // sweep and are extrapolated outside of it
        if (get_perform_secondary_constraint_sim()) {
            const std::string coupled_name = model.getComponent<Coordinate>(
                    get_secondary_coupled_coordinate()).getName();
            Array<double> coupled_values;
            kinematicsReporter->getPositionStorage()->getDataColumn(
                    coupled_name, coupled_values);
            if (coupled_values.getSize() > 0) {
                double lo = coupled_values[0];
                double hi = coupled_values[0];
                for (int k = 1; k < coupled_values.getSize(); ++k) {
                    lo = std::min(lo, coupled_values[k]);
                    hi = std::max(hi, coupled_values[k]);
                }
                if (lo < get_secondary_coupled_coordinate_start_value() ||
                        hi > get_secondary_coupled_coordinate_stop_value()) {
                    log_warn("{} is {} to {} in this trial, outside the range "
                            "of the sweep ({} to {}, set by "
                            "secondary_coupled_coordinate_start_value and "
                            "secondary_coupled_coordinate_stop_value). The "
                            "constraint functions of the secondary "
                            "coordinates are extrapolated there.",
                            coupled_name, lo, hi,
                            get_secondary_coupled_coordinate_start_value(),
                            get_secondary_coupled_coordinate_stop_value());
                }
            }
        }

        if (get_output_motion_file() != "" &&
                get_output_motion_file() != "Unassigned") {
            kinematicsReporter->getPositionStorage()->print(
                    get_results_directory() + "/" + get_output_motion_file());
        }
        // Remove the analysis we added to the model, this also deletes it
        model.removeAnalysis(kinematicsReporter);

        if (modelMarkerErrors) {
            Array<std::string> labels("", 4);
            labels[0] = "time";
            labels[1] = "total_squared_error";
            labels[2] = "marker_error_RMS";
            labels[3] = "marker_error_max";

            modelMarkerErrors->setColumnLabels(labels);
            modelMarkerErrors->setName("Model Marker Errors from IK");

            IO::makeDir(get_results_directory());
            std::string errorFileName = get_results_prefix() + 
                "_ik_marker_errors";
            Storage::printResult(modelMarkerErrors, errorFileName,
                                 get_results_directory(), -1, ".sto");

            delete modelMarkerErrors;
        }

        if(modelMarkerLocations){
            Array<std::string> labels("", 3*nm+1);
            labels[0] = "time";
            Array<std::string> XYZ("", 3*nm);
            XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

            for(int j=0; j<nm; ++j){
                for(int k=0; k<3; ++k)
                    labels.set(3*j+k+1, ikSolver.getMarkerNameForIndex(j)+XYZ[k]);
            }
            modelMarkerLocations->setColumnLabels(labels);
            modelMarkerLocations->setName("Model Marker Locations from IK");
    
            IO::makeDir(get_results_directory());
            std::string markerFileName = get_results_prefix() + 
                "_ik_model_marker_locations";
            Storage::printResult(modelMarkerLocations, markerFileName,
                                 get_results_directory(), -1, ".sto");

            delete modelMarkerLocations;
        }

        log_info("InverseKinematicsTool completed {} frames in {}",
            Nframes, watch.getElapsedTimeFormatted());
    }
    catch (const std::exception& ex) {
         log_error("InverseKinematicsTool Failed: {}", ex.what());
        // If failure happened after kinematicsReporter was added, make sure to cleanup
        if (kinematicsReporter!= nullptr)
            model.removeAnalysis(kinematicsReporter);
        throw (Exception("InverseKinematicsTool Failed, "
            "please see messages window for details..."));
    }
}

void COMAKInverseKinematicsTool::populateReferences(const Model& model, 
    MarkersReference& markersReference,
    SimTK::Array_<CoordinateReference>&coordinateReferences) const
{
    FunctionSet *coordFunctions = NULL;
    // Load the coordinate data
    // bool haveCoordinateFile = false;
    if (get_coordinate_file() != "" && get_coordinate_file() != "Unassigned") {
        Storage coordinateValues(get_coordinate_file());
        warnIfTranslationsNotMeters(model, coordinateValues,
                get_coordinate_file());
        // only a file in degrees is converted
        if (coordinateValues.isInDegrees()) {
            model.getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        } else {
            log_info("The coordinate_file {} is in radians (inDegrees=no).",
                    get_coordinate_file());
        }
        // haveCoordinateFile = true;
        coordFunctions = new GCVSplineSet(5, &coordinateValues);
    }

    Set<MarkerWeight> markerWeights;
    // Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
    // For coordinates, create the functions for coordinate reference values
    int index = 0;
    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        if (!get_IKTaskSet()[i].getApply()) continue;
        if (IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&get_IKTaskSet()[i])) {
            CoordinateReference *coordRef = NULL;
            if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
                if (!coordFunctions)
                    throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");

                index = coordFunctions->getIndex(coordTask->getName(), index);
                if (index >= 0) {
                    coordRef = new CoordinateReference(coordTask->getName(), coordFunctions->get(index));
                }
            }
            else if ((coordTask->getValueType() == IKCoordinateTask::ManualValue)) {
                Constant reference(Constant(coordTask->getValue()));
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }
            else { // assume it should be held at its default value
                double value = model.getCoordinateSet().get(coordTask->getName()).getDefaultValue();
                Constant reference = Constant(value);
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }

            if (coordRef == NULL)
                throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");
            else
                coordRef->setWeight(coordTask->getWeight());

            coordinateReferences.push_back(*coordRef);
        }
        else if (IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&get_IKTaskSet()[i])) {
            if (markerTask->getApply()) {
                // Only track markers that have a task and it is "applied"
                markerWeights.adoptAndAppend(
                    new MarkerWeight(markerTask->getName(), markerTask->getWeight()));
            }
        }
    }

    //Read in the marker data file and set the weights for associated markers.
    //Markers in the model and the marker file but not in the markerWeights are
    //ignored
    markersReference.initializeFromMarkersFile(get_marker_file(), markerWeights);
}

void COMAKInverseKinematicsTool::printDebugInfo(const Model& model, const SimTK::State& state) {

    model.realizeReport(state);

    // Unconstrained Coordinates
    log_debug("{:<30} {:<20} {:<20}", "Unconstrained Coordinate", "Value",
            "Speed");

    for (int i = 0; i < getProperty_secondary_coordinates().size();
            ++i) {
        std::string coord_path = get_secondary_coordinates(i);
        const Coordinate& coord = model.getComponent<Coordinate>(coord_path);
        log_debug("{:<30} {:<20} {:<20}", coord.getName(),
                coord.getValue(state), coord.getSpeedValue(state));
    }
    log_debug("");

    // Muscle
    log_debug("{:<20} {:<20} {:<20} {:<20}", "Muscle", "Force", "Activation",
            "Control");

    for (const Muscle& msl : model.getComponentList<Muscle>()) {
        log_debug("{:<20} {:<20} {:<20} {:<20}", msl.getName(),
                msl.getActuation(state), msl.getActivation(state),
                msl.getControl(state));
    }
    log_debug("");

    // Ligament
    log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}", "Ligament",
            "Total Force", "Spring Force", "Damping Force", "Strain",
            "Strain Rate", "Length");

    for (const Blankevoort1991Ligament& lig :
            model.getComponentList<Blankevoort1991Ligament>()) {

        log_debug("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
                lig.getName(), lig.getOutputValue<double>(state, "total_force"),
                lig.getOutputValue<double>(state, "spring_force"),
                lig.getOutputValue<double>(state, "damping_force"),
                lig.getOutputValue<double>(state, "strain"),
                lig.getOutputValue<double>(state, "strain_rate"),
                lig.getOutputValue<double>(state, "length"));
    }
    log_debug("");

    // Contact
    log_debug("{:<20} {:<20} {:<20}", "Contact", "Force", "COP");

    for (const Smith2018ArticularContactForce& cnt :
            model.getComponentList<Smith2018ArticularContactForce>()) {

        log_debug("{:<20} {:<20} {:<20}", cnt.getName(),
                cnt.getOutputValue<SimTK::Vec3>(
                        state, "casting_total_contact_force").toString(),
                cnt.getOutputValue<SimTK::Vec3>(
                        state, "casting_total_center_of_pressure").toString());
    }
    log_debug("");
    /*
        if (get_verbose() >= 3) {
            log_debug("{:<20} {:<20} {:<20}", "Contact " << std::setw(20)
                << "Force" << std::setw(w) << "COP" << std::endl;

            for (Smith2018ArticularContactForce& cnt :
       model.updComponentList<Smith2018ArticularContactForce>()) {

                std::cout << std::setw(w) << cnt.getName()
                    << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state,
       "casting_triangle_proximity")
                    << std::setw(w) << cnt.getOutputValue<SimTK::Vector>(state,
       "casting_triangle_pressure")
                    << std::endl;
            }
        }

        if (get_verbose() >= 2) {
            std::cout << "Press Any Key to Continue." << std::endl;
            std::cin.ignore();
        }*/
}