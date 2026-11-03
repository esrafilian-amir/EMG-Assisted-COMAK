/* -------------------------------------------------------------------------- *
 *                               COMAKTarget.cpp                              *
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
#include "COMAKTarget.h"
#include <chrono>
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
//#include <OpenSim.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================

ComakTarget::
ComakTarget(SimTK::State s, Model* aModel,
    const SimTK::Vector& observed_udot,
    const SimTK::Vector& init_parameters, 
    double dt, double unit_udot_epsilon,
    Array<std::string> primary_coords, Array<std::string> secondary_coords,
    Array<std::string> muscle_path,
    Array<std::string> non_muscle_actuator_path,
    const SimTK::Vector& max_change) : _init_parameters(init_parameters)
{
    // Set Parameters
    _model = aModel;
    _state = s;

    _primary_coords = primary_coords;
    _nPrimaryCoord = primary_coords.size();

    _secondary_coords = secondary_coords;
    _nSecondaryCoord = secondary_coords.size();

    _muscle_path = muscle_path;
    _non_muscle_actuator_path = non_muscle_actuator_path;

    _observed_udot = observed_udot;

    _max_change = max_change;
    _unit_udot_epsilon = unit_udot_epsilon;
    _dt = dt;

    // Set Defaults
    _activation_exponent = 2.0;
    _scale_delta_coord = 1.0;
    _contact_energy_weight = 0.0;
    _non_muscle_actuator_weight = 1000;

}

void ComakTarget::initialize(){
    //Number of Parameters
    _nMuscles = _muscle_path.size();
    _nNonMuscleActuators = _non_muscle_actuator_path.size();
    _nActuators = _nMuscles + _nNonMuscleActuators;

    _nParameters = _nActuators + _nSecondaryCoord;
    
    //Number of Constraints
    int nC = 0;
    int nCoord = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

            _constraint_names.append(coord.getName());
            nC++;
        }
        else if(
            _secondary_coords.findIndex(coord.getAbsolutePathString())> -1) {

            _constraint_names.append(coord.getName());
            nC++;
        }
        nCoord++;
    }
    _nConstraints = nC;
    _nCoordinates = nCoord;

    // Optimization variables: the parameters and the slack variables
    _nSlack = _relax ? nC : 0;
    setNumParameters(_nParameters + _nSlack);
    setParameterBounds(1);

    setNumEqualityConstraints(nC);
    setNumLinearEqualityConstraints(nC);
    setNumInequalityConstraints(0);

    if(_muscle_weight.size() == 0){
        _muscle_weight.resize(_nMuscles);
        _muscle_weight = 1.0;
    }
    if(_desired_act.size() == 0){
        _desired_act.resize(_nMuscles);
        _desired_act = 0.0;
    }
    if(_muscle_volumes.size() == 0){
        _muscle_volumes.resize(_nMuscles);
        _muscle_volumes = 1.0;
    }
    if (_optimal_force.size() == 0) {
        _optimal_force.resize(_nActuators);
        _optimal_force = 1.0;
    }
    if (_passive_force.size() == 0) {
        _passive_force.resize(_nActuators);
        _passive_force = 0.0;
    }
    if (_parameter_names.size() == 0) {
        _parameter_names.setSize(_nParameters);
    }
    
    if (_emg_gamma_weight.size() == 0) {
        _emg_gamma_weight.resize(_nMuscles);
        _emg_gamma_weight = 0.0;
    }
     //Precompute Constraint Matrix
    precomputeConstraintMatrix();
    if (_verbose > 1) {
        log_debug("Num Parameters: {}", _nParameters);
        log_debug("Num Constraints: {}", _nConstraints);
        log_debug("Num Actuators: {}", _nActuators);
        log_debug("Num Coordinates: {}", _nCoordinates);
        log_debug("Num Secondary Coordinates: {}", _secondary_coords.size());
        log_debug("Num Primary Coordinates: {}", _primary_coords.size());
    }
}

void ComakTarget::precomputeConstraintMatrix() {
    _constraint_initial_udot.resize(_nConstraints);
    _constraint_desired_udot.resize(_nConstraints);

    //Calculate the initial udots generated by the initial state and 
    //optimization parameters

    SimTK::Vector initial_udot(_nCoordinates);
    realizeAccelerationFromParameters(_state, _init_parameters);

    int i = 0;
    for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
        initial_udot(i) = coord.getAccelerationValue(_state);
        i++;
    }

    i = 0;
    int j = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = _observed_udot[j];
            _constraint_initial_udot[i] = initial_udot[j];
            i++;
        }
        if (_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = 0;
            _constraint_initial_udot[i] = initial_udot[j];
            i++;
        }
        j++;
    }

    //Calculate Unit Udots (change in accelerations due to unit change in 
    // optimization parameters)
    // rows - coordinates, columns - actuators

    _msl_unit_udot.resize(_nConstraints, _nMuscles);
    _non_muscle_actuator_unit_udot.resize(_nConstraints, _nNonMuscleActuators);
    _secondary_coord_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_speed_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_coord_unit_energy.resize(_nSecondaryCoord);
       
    _msl_unit_udot = -1;
    _non_muscle_actuator_unit_udot = -1;
    _secondary_coord_unit_udot = -1;
    _secondary_speed_unit_udot = -1;
    _secondary_coord_unit_energy = -1;

    //Compute initial Contact Energy
    _initial_contact_energy = 0;
    for (const auto& cnt_frc : 
        _model->updComponentList<Smith2018ArticularContactForce>()) {
        
        _initial_contact_energy += 
            cnt_frc.getOutputValue<double>(_state,"potential_energy");
    }

    // Reused columns skip the model evaluations of the next sections
    auto t_muscle = std::chrono::steady_clock::now();
    auto t_reserve = t_muscle, t_secondary = t_muscle, t_speed = t_muscle,
         t_end = t_muscle;
    if (_reuse_jacobian && _reused_msl_unit_udot.nrow() == _nConstraints &&
            _reused_msl_unit_udot.ncol() == _nMuscles &&
            _reused_secondary_coord_unit_udot.ncol() == _nSecondaryCoord) {
        _msl_unit_udot = _reused_msl_unit_udot;
        _non_muscle_actuator_unit_udot = _reused_non_muscle_actuator_unit_udot;
        _secondary_coord_unit_udot = _reused_secondary_coord_unit_udot;
        _secondary_speed_unit_udot = _reused_secondary_speed_unit_udot;
        _secondary_coord_unit_energy = _reused_secondary_coord_unit_energy;
    } else {
    t_muscle = std::chrono::steady_clock::now();
    //Compute Muscle Unit Udot
    j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double force = _init_parameters[j] * _optimal_force[j] + _passive_force[j];

        msl.setOverrideActuation(_state, force + 1.0);

        _model->realizeAcceleration(_state);
        
        int k = 0;
        for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _msl_unit_udot(k, j) = coord.getAccelerationValue(_state) - 
                    _constraint_initial_udot(k);
                k++;
            }
        }
        msl.setOverrideActuation(_state, force);
        j++;
    }

    t_reserve = std::chrono::steady_clock::now();
    //Compute Non Muscle Actuator Unit Udot
    j = 0;
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double force = 
            _init_parameters[_nMuscles + j] * _optimal_force[_nMuscles + j];

        actuator.setOverrideActuation(_state, force + 1.0);

        _model->realizeAcceleration(_state);
        
        int k = 0;
         for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _non_muscle_actuator_unit_udot(k, j) =  
                    (coord.getAccelerationValue(_state) - 
                        _constraint_initial_udot(k));
                k++;
            }
        }
        actuator.setOverrideActuation(_state, force);

        j++;
    }

    t_secondary = std::chrono::steady_clock::now();
    //Compute Secondary Coordinate Unit Udot
    //(forward differences, or central differences if requested)
    for (int j = 0; j < _nSecondaryCoord; ++j) {
        Coordinate& optim_coord =
            _model->updComponent<Coordinate>(_secondary_coords[j]);

        const double init_value = optim_coord.getValue(_state);

        // Accelerations of the constrained coordinates and the contact energy
        // with the secondary coordinate set to value
        auto evaluate = [&](double value, SimTK::Vector& udot, double& energy) {
            optim_coord.setValue(_state, value, true);
            _model->realizeAcceleration(_state);

            int k = 0;
            for (const auto& coord : _model->updComponentList<Coordinate>()) {
                bool is_primary = _primary_coords.findIndex(
                        coord.getAbsolutePathString()) > -1;
                bool is_secondary = _secondary_coords.findIndex(
                        coord.getAbsolutePathString()) > -1;

                if (is_primary || is_secondary) {
                    udot[k] = coord.getAccelerationValue(_state);
                    k++;
                }
            }

            energy = 0;
            for (Smith2018ArticularContactForce& cnt_frc :
                _model->updComponentList<Smith2018ArticularContactForce>()) {
                energy += cnt_frc.getOutputValue<double>(
                        _state, "potential_energy");
            }
        };

        SimTK::Vector udot_plus(_nConstraints);
        double energy_plus = 0;
        evaluate(init_value + _unit_udot_epsilon, udot_plus, energy_plus);

        if (_central_difference) {
            SimTK::Vector udot_minus(_nConstraints);
            double energy_minus = 0;
            evaluate(init_value - _unit_udot_epsilon, udot_minus, energy_minus);

            for (int k = 0; k < _nConstraints; ++k) {
                _secondary_coord_unit_udot(k, j) = _scale_delta_coord *
                    (udot_plus[k] - udot_minus[k]) / (2.0 * _unit_udot_epsilon);
            }
            _secondary_coord_unit_energy(j) = _scale_delta_coord *
                (energy_plus - energy_minus) / (2.0 * _unit_udot_epsilon);
        } else {
            for (int k = 0; k < _nConstraints; ++k) {
                _secondary_coord_unit_udot(k, j) = _scale_delta_coord *
                    (udot_plus[k] - _constraint_initial_udot(k)) /
                    _unit_udot_epsilon;
            }
            _secondary_coord_unit_energy(j) = _scale_delta_coord *
                (energy_plus - _initial_contact_energy) / _unit_udot_epsilon;
        }

        optim_coord.setValue(_state, init_value, true);
    }
    _model->realizeAcceleration(_state);

    t_speed = std::chrono::steady_clock::now();
    //Compute COMAK damping unit udot
    //Force in each damping actuator is zeroed in initialize()

    for (int j = 0; j < _nSecondaryCoord; ++j) {
        Coordinate& optim_coord = 
            _model->updComponent<Coordinate>(_secondary_coords[j]);

        double init_speed = optim_coord.getSpeedValue(_state);
        double speed_epsilon = _unit_udot_epsilon / _dt;

        optim_coord.setSpeedValue(_state, init_speed + speed_epsilon);

        _model->realizeAcceleration(_state);

        int k = 0;
        for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _secondary_speed_unit_udot(k, j) = _scale_delta_coord *
                    (coord.getAccelerationValue(_state) - 
                        _constraint_initial_udot(k)) / _unit_udot_epsilon;

                k++;
            }
        }

        optim_coord.setSpeedValue(_state, init_speed);
    }

    t_end = std::chrono::steady_clock::now();
    } // END computing columns
    {
        auto sec = [](std::chrono::steady_clock::time_point a,
                           std::chrono::steady_clock::time_point b) {
            return std::chrono::duration<double>(b - a).count();
        };
        if (!_reuse_jacobian) log_info("Linearization columns (s): muscles {:.2f} ({}), reserves "
                 "{:.2f} ({}), secondary coordinates {:.2f} ({}), secondary "
                 "speeds {:.2f} ({})",
                sec(t_muscle, t_reserve), _nMuscles,
                sec(t_reserve, t_secondary), _nNonMuscleActuators,
                sec(t_secondary, t_speed), _nSecondaryCoord,
                sec(t_speed, t_end), _nSecondaryCoord);
    }

    //Assemble Constraint Matrix 
    //Constraints = Rows
    //Optimized Parameters = Columns
    
    _constraint_matrix.resize(_nConstraints,_nParameters);
    _constraint_matrix = 0.0;

    for (int i = 0; i < _nConstraints; ++i) {
        int p = 0;

        //Muscles
        for (int j = 0; j < _nMuscles; ++j) {
            _constraint_matrix(i,p) = _optimal_force[p] * _msl_unit_udot(i, j);
            p++;
        }

        //Non Muscle Actuators
        for (int j = 0; j < _nNonMuscleActuators; ++j) {
            _constraint_matrix(i,p) = 
                _optimal_force[p] * _non_muscle_actuator_unit_udot(i, j);
            p++;
        }

        //Secondary Coordinates
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            _constraint_matrix(i,p) = _secondary_coord_unit_udot(i,j);
            if (!_ignore_secondary_speed_term) {
                _constraint_matrix(i,p) += _secondary_speed_unit_udot(i,j);
            }
            p++;
        }
    }

    // Normalize each constraint row by its largest entry, so that light
    // coordinates with very large acceleration sensitivities do not dominate
    if (_normalize_rows) {
        for (int i = 0; i < _nConstraints; ++i) {
            double row_max = 1e-12;
            for (int p = 0; p < _nParameters; ++p) {
                row_max = std::max(row_max, std::abs(_constraint_matrix(i, p)));
            }
            const double scale = 1.0 / row_max;
            for (int p = 0; p < _nParameters; ++p) {
                _constraint_matrix(i, p) *= scale;
            }
            _constraint_initial_udot[i] *= scale;
            _constraint_desired_udot[i] *= scale;
        }
    }

    // The slack variables of the relaxed constraints are in the units of the
    // constraint rows (accelerations). Scaling the rows by their largest entry
    // would hide large acceleration errors in rows with large entries.
    _row_scale.resize(_nConstraints);
    _row_scale = 1.0;

    if (_verbose > 1) {
        log_trace("Constraint Matrix: ");
        for (int i = 0; i < _nConstraints; ++i) {
            log_trace(_constraint_names[i]);
            int p = 0;

            // Muscles
            log_trace("Muscles");
            for (int j = 0; j < _nMuscles; ++j) {
                log_trace("{}", _constraint_matrix(i, p));
                p++;
            }

            // Non Muscle Actuators
            log_trace("Non Muscle Actuators");
            for (int j = 0; j < _nNonMuscleActuators; ++j) {
                log_trace("{}", _constraint_matrix(i, p));
                p++;
            }

            // Secondary Coordinates
            log_trace("Delta Secondary Coord");
            for (int j = 0; j < _nSecondaryCoord; ++j) {
                log_trace("{}", _constraint_matrix(i, p));
                p++;
            }
        }
    }
}

void ComakTarget::realizeAccelerationFromParameters
    (SimTK::State& s, const SimTK::Vector &parameters) 
{
    //Apply Muscle Forces
    int j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);
        msl.overrideActuation(s, true);
        double force = _optimal_force[j] * parameters[j] + _passive_force[j];
        msl.setOverrideActuation(s,force);
        j++;
    }

    //Apply Non Muscle Actuator Forces
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);
        actuator.overrideActuation(s, true);
        double force = _optimal_force[j] * parameters[j];
        actuator.setOverrideActuation(s,force);
        j++;
    }

    //Set Secondary Coordinates 
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        Coordinate& coord = 
            _model->updComponent<Coordinate>(_secondary_coords[i]);

        double value = coord.getValue(s) + parameters(_nActuators + i);

        coord.setValue(s, value, false);
    }

    _model->assemble(s);
    _model->realizeAcceleration(s);
}

//=============================================================================
// OPTIMIZATION FUNCTIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Objective Function (cost)
//-----------------------------------------------------------------------------

int ComakTarget::
objectiveFunc(const SimTK::Vector &parameters, const bool new_parameters,
    SimTK::Real &performance) const
{
    int p = 0;

    //Muscle Activations
    double msl_cost = 0.0;
    for (int i = 0; i < _nMuscles; i++) {
        //msl_cost += _muscle_volumes(i) *
        //            pow(fabs(parameters(p)), _activation_exponent);
        msl_cost += _muscle_weight(i) * _muscle_volumes(i) *
                    pow(fabs(parameters(i)), _activation_exponent);
        // follow EMGs
        if (_is_emg_assisted) {
            msl_cost += _muscle_weight(i) * 
                        _emg_gamma_weight(i) *
                        pow(parameters(i) - _desired_act(i), 2);
        }
        // smooth change from the previous time step
        if (_act_change_weight > 0 &&
                _previous_activations.size() == _nMuscles) {
            msl_cost += _act_change_weight * _muscle_weight(i) *
                        _muscle_volumes(i) *
                        pow(parameters(i) - _previous_activations(i), 2);
        }
        p++;
    }
    
    //Non Muscle Actuators
    double non_muscle_cost = 0.0;
    for (int i = 0; i < _nNonMuscleActuators; i++) {
        non_muscle_cost +=  
            pow(fabs(parameters(p)), _activation_exponent);
        p++;
    }

    //Contact Energy
    //double contact_cost = 0.0;
    double contact_cost = _initial_contact_energy;
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        contact_cost += parameters(p) * _secondary_coord_unit_energy[i];
        p++;
    }

    performance = msl_cost 
        + non_muscle_cost * _non_muscle_actuator_weight
        + contact_cost * _contact_energy_weight;

    // Penalty on the change of the secondary coordinates
    if (_step_penalty_weight > 0 && _step_unit_scale.size() == _nSecondaryCoord) {
        double step_cost = 0.0;
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            const double d = _step_unit_scale[j] * _scale_delta_coord *
                    parameters(_nActuators + j);
            step_cost += d * d;
        }
        performance += _step_penalty_weight * step_cost;
    }

    // Penalty on the slack variables of the relaxed constraints
    if (_nSlack > 0 && parameters.size() == _nParameters + _nSlack) {
        double slack_cost = 0.0;
        for (int i = 0; i < _nSlack; ++i) {
            slack_cost += parameters(_nParameters + i) *
                    parameters(_nParameters + i);
        }
        performance += _slack_penalty * slack_cost;
    }

    if (_verbose > 1) {
        std::cout << "Total Cost: " << performance << "\t";
        std::cout << "Muscle Cost: " << msl_cost << "\t";
        std::cout << "Non Muscle Actuator Cost: " << non_muscle_cost << "\t";
        std::cout << "Contact Cost: " << contact_cost << std::endl;
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Objective Function Gradient (w.r.t. optimization parameters)
//-----------------------------------------------------------------------------
int ComakTarget::
gradientFunc(const SimTK::Vector &parameters, const bool new_parameters,
    SimTK::Vector &gradient) const
{
    gradient = 0;
    int p = 0;
    /*for (int i = 0; i < _nMuscles; i++) {
        gradient[p] += _activation_exponent * 
            _muscle_volumes(i) * _muscle_weight(i) *
            (parameters[p] - _desired_act(p)) * 
            pow(fabs(_desired_act(p) - parameters[p]),
                _activation_exponent - 2.0);

        p++;
    }
    */
    
    for (int i = 0; i < _nMuscles; i++) {
        gradient(i) += _activation_exponent * _muscle_weight(i) * _muscle_volumes(i) *
                        pow(fabs(parameters(i)), _activation_exponent - 1.0);
        // follow EMGs
        if (_is_emg_assisted) {
            gradient(i) += 2.0 * _muscle_weight(i) * _emg_gamma_weight(i) *
                            (parameters(i) - _desired_act(i));
        }
        if (_act_change_weight > 0 &&
                _previous_activations.size() == _nMuscles) {
            gradient(i) += 2.0 * _act_change_weight * _muscle_weight(i) *
                            _muscle_volumes(i) *
                            (parameters(i) - _previous_activations(i));
        }
        p++;
    }

    for (int i = 0; i < _nNonMuscleActuators; i++) {
        const double sign_p = (parameters(p) > 0) - (parameters(p) < 0);
        gradient(p) += _activation_exponent * _non_muscle_actuator_weight *
                       sign_p *
                       pow(fabs(parameters(p)), _activation_exponent - 1.0);

        p++;
    }

    for (int i = 0; i < _nSecondaryCoord; ++i) {
        gradient(p) += 
            _contact_energy_weight * _secondary_coord_unit_energy[i];
        p++;
    }
    if (_step_penalty_weight > 0 && _step_unit_scale.size() == _nSecondaryCoord) {
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            const double sc = _step_unit_scale[j] * _scale_delta_coord;
            gradient(_nActuators + j) += 2.0 * _step_penalty_weight * sc * sc *
                    parameters(_nActuators + j);
        }
    }
    if (_nSlack > 0 && parameters.size() == _nParameters + _nSlack) {
        for (int i = 0; i < _nSlack; ++i) {
            gradient(_nParameters + i) =
                    2.0 * _slack_penalty * parameters(_nParameters + i);
        }
    }
    if (_verbose > 1) {
        log_trace("Cost Gradient");
        for (int i = 0; i < _nParameters; ++i) {
            log_trace("{} : {}", _parameter_names[i], gradient(i));
        }
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Constraint Function 
//-----------------------------------------------------------------------------

int ComakTarget::
constraintFunc(const SimTK::Vector &parameters, bool new_parameters,
    SimTK::Vector &constraints) const
{
    const SimTK::Vector x = parameters(0, _nParameters);
    constraints = _constraint_initial_udot - _constraint_desired_udot +
        _constraint_matrix * (x - _init_parameters);

    // relaxed constraints: row / scale + slack = 0
    if (_nSlack > 0 && parameters.size() == _nParameters + _nSlack) {
        for (int i = 0; i < _nConstraints; ++i) {
            constraints[i] = constraints[i] / _row_scale[i] +
                    parameters(_nParameters + i);
        }
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Constraint Jacobian
// (gradient of contstraints w.r.t. optimization parameters)
//-----------------------------------------------------------------------------
int ComakTarget::constraintJacobian(const SimTK::Vector &parameters,
    bool new_parameters, SimTK::Matrix &jac) const
{
    if (_nSlack > 0 && parameters.size() == _nParameters + _nSlack) {
        jac.resize(_nConstraints, _nParameters + _nSlack);
        jac = 0.0;
        for (int i = 0; i < _nConstraints; ++i) {
            for (int p = 0; p < _nParameters; ++p) {
                jac(i, p) = _constraint_matrix(i, p) / _row_scale[i];
            }
            jac(i, _nParameters + i) = 1.0;
        }
    } else {
        jac = _constraint_matrix;
    }

    return 0;
}

void ComakTarget::setParameterBounds(double scale) {

    SimTK::Vector lower_bounds(_nParameters), upper_bounds(_nParameters);
    int p = 0;

    //Muscles 
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double min_value = msl.getMinControl();
        double max_value = msl.getMaxControl();

        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    //Non Muscle Actuators
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double min_control = actuator.getMinControl();
        double max_control = actuator.getMaxControl();
        double min_value;
        double max_value;
        if (min_control > -1000)
            min_value = actuator.getMinControl()*scale;
        else {
            min_value = -1000*scale;
        }
        if (max_control < 1000)
            max_value = actuator.getMaxControl()*scale;
        else {
            max_value = 1000*scale;
        }
        
        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    //Secondary Coordinates
    for (int j = 0; j < _nSecondaryCoord; j++) {
        // change of the coordinate value allowed in this optimization
        // A maximum change of 0 or less means the coordinate is free
        const bool limited = _max_change[j] > 0;
        double change_lower = limited ? -_max_change[j] * scale : -1e19;
        double change_upper = limited ? _max_change[j] * scale : 1e19;

        const Coordinate& coord =
                _model->getComponent<Coordinate>(_secondary_coords[j]);
        const double value = coord.getValue(_state);

        // The change since the previous time step, summed over the COMAK
        // iterations, also stays within the maximum change
        if (limited && _reference_values.size() == _nSecondaryCoord) {
            change_lower = std::max(change_lower,
                    _reference_values[j] - _max_change[j] * scale - value);
            change_upper = std::min(change_upper,
                    _reference_values[j] + _max_change[j] * scale - value);
        }

        // Keep the coordinate value within its range
        if (_enforce_coordinate_range) {
            change_lower = std::max(change_lower, coord.getRangeMin() - value);
            change_upper = std::min(change_upper, coord.getRangeMax() - value);
        }

        // Limits that exclude each other (the value is already outside of
        // them): take the smaller step
        if (change_lower > change_upper) {
            change_lower = change_upper =
                    (std::abs(change_lower) < std::abs(change_upper))
                            ? change_lower : change_upper;
        }
        lower_bounds(p) = change_lower / _scale_delta_coord;
        upper_bounds(p) = change_upper / _scale_delta_coord;
        p++;
    }
    if (_nSlack > 0) {
        // the slack variables are free
        SimTK::Vector lower_all(_nParameters + _nSlack, -1e20);
        SimTK::Vector upper_all(_nParameters + _nSlack, 1e20);
        for (int i = 0; i < _nParameters; ++i) {
            lower_all[i] = lower_bounds[i];
            upper_all[i] = upper_bounds[i];
        }
        lower_bounds = lower_all;
        upper_bounds = upper_all;
    }
    setParameterLimits(lower_bounds, upper_bounds);
    if (_verbose > 1) {
        log_debug("{:<20} {:<15} {:<15} {:<15},",
                "Parameter Limits: ", "initial value", "lower bound",
                "upper bound");
        log_debug("------------------------------------------");

        for (int i = 0; i < _nParameters; ++i) {
            log_debug("{:<3} {:<17} {:<15} {:<15} {:<15}", i,
                    _parameter_names[i], _init_parameters[i], lower_bounds(i),
                    upper_bounds(i));
        }
    }
}

void ComakTarget::printPerformance(SimTK::Vector parameters) {
    bool notNeeded = false;
    SimTK::Real performance;
    objectiveFunc(parameters, notNeeded, performance);
    if (_verbose > 1) {
        log_info("Optimization Cost Function: {}", performance);
    }
}

